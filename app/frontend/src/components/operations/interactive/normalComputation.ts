import type { SessionData } from './types';

// Math helpers for client-side normal computation
export function computeToolEuler(normal_base: [number, number, number]): [number, number, number] {
  // Tool Z points towards surface: -normal_base
  let zx = -normal_base[0], zy = -normal_base[1], zz = -normal_base[2];
  const zNorm = Math.hypot(zx, zy, zz);
  if (zNorm < 1e-6) return [0, 90, 0];
  zx /= zNorm; zy /= zNorm; zz /= zNorm;

  // Approximate global X as reference
  let refX = 1.0, refY = 0.0, refZ = 0.0;
  if (Math.abs(zx) > 0.9) { refX = 0.0; refY = 1.0; refZ = 0.0; }

  // Y = normalize(cross(Z, ref))
  let yx = zy * refZ - zz * refY;
  let yy = zz * refX - zx * refZ;
  let yz = zx * refY - zy * refX;
  const yNorm = Math.hypot(yx, yy, yz);
  if (yNorm < 1e-6) return [0, 90, 0];
  yx /= yNorm; yy /= yNorm; yz /= yNorm;

  // X = cross(Y, Z)
  const xx = yy * zz - yz * zy;
  const xy = yz * zx - yx * zz;

  // Rotation matrix: columns are X, Y, Z
  // Extract standard intrinsic ZYX Euler angles
  let rx: number, ry: number, rz: number;
  const sy = Math.hypot(xx, xy);
  const singular = sy < 1e-6;

  if (!singular) {
    rx = Math.atan2(zy, zz);
    ry = Math.atan2(-zx, sy);
    rz = Math.atan2(xy, xx);
  } else {
    rx = Math.atan2(-yz, yy);
    ry = Math.atan2(-zx, sy);
    rz = 0;
  }

  const toDeg = (rad: number) => +(rad * 180 / Math.PI).toFixed(2);
  return [toDeg(rx), toDeg(ry), toDeg(rz)];
}

export function getRobustDepth(
  depth: Float32Array,
  width: number,
  height: number,
  u: number,
  v: number,
  maxR: number = 3
): number {
  u = Math.round(u);
  v = Math.round(v);
  if (u < 0 || u >= width || v < 0 || v >= height) return 0;
  const z0 = depth[v * width + u];
  if (z0 > 100 && z0 < 3000) return z0;

  for (let r = 1; r <= maxR; r++) {
    const valid: number[] = [];
    for (let du = -r; du <= r; du++) {
      for (let dv = -r; dv <= r; dv++) {
        if (Math.abs(du) === r || Math.abs(dv) === r) {
          const nu = u + du, nv = v + dv;
          if (nu >= 0 && nu < width && nv >= 0 && nv < height) {
            const val = depth[nv * width + nu];
            if (val > 100 && val < 3000) valid.push(val);
          }
        }
      }
    }
    if (valid.length >= 3) {
      valid.sort((a, b) => a - b);
      return valid[Math.floor(valid.length / 2)];
    }
  }
  return 0;
}

export interface ClientNormalResult {
  surf_cam: [number, number, number];
  surf_base: [number, number, number];
  normal_cam: [number, number, number];
  normal_base: [number, number, number];
  tcp_base: [number, number, number];
  euler_deg: [number, number, number];
  proj_dx: number;
  proj_dy: number;
}

export function computeNormalClientSide(
  session: SessionData,
  u: number,
  v: number,
  standoffDistMm: number = 150.0,
  kR: number = 12
): ClientNormalResult | null {
  const { width, height, depth, fx, fy, cx, cy, T } = session;
  const z = getRobustDepth(depth, width, height, u, v, 3);
  if (z <= 0) return null;

  // Camera coordinates of center pixel (in mm)
  const xc = (u - cx) * z / fx;
  const yc = (v - cy) * z / fy;
  const zc = z;

  // Local patch PCA plane fitting
  const uMin = Math.max(0, u - kR), uMax = Math.min(width - 1, u + kR);
  const vMin = Math.max(0, v - kR), vMax = Math.min(height - 1, v + kR);

  const pts: number[][] = [];
  let mx = 0, my = 0, mz = 0;

  for (let row = vMin; row <= vMax; row++) {
    for (let col = uMin; col <= uMax; col++) {
      const pz = depth[row * width + col];
      if (pz > 100 && pz < 3000 && Math.abs(pz - z) < 60) {
        const px = (col - cx) * pz / fx;
        const py = (row - cy) * pz / fy;
        pts.push([px, py, pz]);
        mx += px; my += py; mz += pz;
      }
    }
  }

  let ncx = 0, ncy = 0, ncz = -1;

  if (pts.length >= 6) {
    const N = pts.length;
    mx /= N; my /= N; mz /= N;

    let cxx = 0, cyy = 0, czz = 0, cxy = 0, cxz = 0, cyz = 0;
    for (const [px, py, pz] of pts) {
      const dx = px - mx, dy = py - my, dz = pz - mz;
      cxx += dx * dx; cyy += dy * dy; czz += dz * dz;
      cxy += dx * dy; cxz += dx * dz; cyz += dy * dz;
    }

    // Smallest eigenvector approximation
    const c0 = cyy * czz - cyz * cyz;
    const c1 = cxx * czz - cxz * cxz;
    const c2 = cxx * cyy - cxy * cxy;

    if (c2 >= c0 && c2 >= c1 && c2 > 1e-4) {
      ncx = -(cxy * czz - cxz * cyz) / c2;
      ncy = -(cxx * cyz - cxz * cxy) / c2;
      ncz = 1.0;
    } else if (c0 >= c1 && c0 > 1e-4) {
      ncx = 1.0;
      ncy = -(cyz * cxx - cxy * cxz) / c0;
      ncz = -(cyy * cxz - cxy * cyz) / c0;
    } else {
      ncx = -(cxz * cyy - cxy * cyz) / Math.max(c1, 1e-4);
      ncy = 1.0;
      ncz = -(cxx * cyz - cxz * cxy) / Math.max(c1, 1e-4);
    }

    const nNorm = Math.hypot(ncx, ncy, ncz);
    if (nNorm > 1e-6) {
      ncx /= nNorm; ncy /= nNorm; ncz /= nNorm;
    }
  }

  // Ensure normal points back towards the camera
  if (ncx * xc + ncy * yc + ncz * zc > 0) {
    ncx = -ncx; ncy = -ncy; ncz = -ncz;
  }

  // Transform to Robot Base coordinates via Hand-Eye matrix T
  const pBaseX = T[0] * xc + T[1] * yc + T[2] * zc + T[3];
  const pBaseY = T[4] * xc + T[5] * yc + T[6] * zc + T[7];
  const pBaseZ = T[8] * xc + T[9] * yc + T[10] * zc + T[11];

  const nBaseX = T[0] * ncx + T[1] * ncy + T[2] * ncz;
  const nBaseY = T[4] * ncx + T[5] * ncy + T[6] * ncz;
  const nBaseZ = T[8] * ncx + T[9] * ncy + T[10] * ncz;
  const nbNorm = Math.hypot(nBaseX, nBaseY, nBaseZ) || 1.0;
  const nbx = nBaseX / nbNorm, nby = nBaseY / nbNorm, nbz = nBaseZ / nbNorm;

  // Standoff TCP target in robot base
  const tcpBaseX = pBaseX + standoffDistMm * nbx;
  const tcpBaseY = pBaseY + standoffDistMm * nby;
  const tcpBaseZ = pBaseZ + standoffDistMm * nbz;

  const euler = computeToolEuler([nbx, nby, nbz]);

  // Project Normal Arrow into 2D camera image pixels (perspective projection)
  const tipZ = zc + standoffDistMm * ncz;
  let projDx = 0, projDy = 0;
  if (tipZ > 50) {
    const tipU = (xc + standoffDistMm * ncx) * fx / tipZ + cx;
    const tipV = (yc + standoffDistMm * ncy) * fy / tipZ + cy;
    projDx = +(tipU - u).toFixed(1);
    projDy = +(tipV - v).toFixed(1);
  }

  const round2 = (v: number) => +v.toFixed(2);
  const round4 = (v: number) => +v.toFixed(4);

  return {
    surf_cam: [round2(xc), round2(yc), round2(zc)],
    surf_base: [round2(pBaseX), round2(pBaseY), round2(pBaseZ)],
    normal_cam: [round4(ncx), round4(ncy), round4(ncz)],
    normal_base: [round4(nbx), round4(nby), round4(nbz)],
    tcp_base: [round2(tcpBaseX), round2(tcpBaseY), round2(tcpBaseZ)],
    euler_deg: euler,
    proj_dx: projDx,
    proj_dy: projDy
  };
}
