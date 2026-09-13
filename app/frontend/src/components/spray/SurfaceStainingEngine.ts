import {
  Mesh,
  BufferGeometry,
  BufferAttribute,
  Vector3,
  Matrix4,
  Color
} from 'three';
import type { SpraySimConfig, SprayCoverageStats } from './sprayTypes';

export class SurfaceStainingEngine {
  private mesh: Mesh | null = null;
  private geometry: BufferGeometry | null = null;
  private positions: Float32Array | null = null;
  private normals: Float32Array | null = null;
  private colors: Float32Array | null = null;
  private thicknessArray: Float32Array | null = null;
  private vertexCount: number = 0;

  // Spatial Grid Hash Partitioning for O(k) queries (R6)
  private readonly cellSize: number = 0.035; // 35mm cell
  // Numeric-keyed buckets: template-string keys would allocate ~1 string per queried
  // cell per frame (>100k allocs/s while spraying), causing GC churn on RK3588. Packing
  // the 3 signed cell indices into one collision-free integer keeps this Zero-GC.
  private static readonly GRID_BIAS = 2048;   // index range +-2048 cells (~ +-71m @ 35mm)
  private static readonly GRID_STRIDE = 4096;
  private gridBuckets: Map<number, number[]> = new Map();

  // Zero-GC Preallocated Scratch Objects (R5)
  private readonly _invMatScratch: Matrix4 = new Matrix4();
  private readonly _tcpPos: Vector3 = new Vector3();
  private readonly _tcpDir: Vector3 = new Vector3();
  private readonly _tempPaintColor: Color = new Color();

  // Scratch parsed colors
  private paintR: number = 0.15;
  private paintG: number = 0.39;
  private paintB: number = 0.92;
  private currentPaintHex: string = '';

  constructor(targetMesh?: Mesh) {
    if (targetMesh) {
      this.attachMesh(targetMesh);
    }
  }

  public attachMesh(targetMesh: Mesh): void {
    this.mesh = targetMesh;
    this.geometry = targetMesh.geometry;
    if (!this.geometry || !this.geometry.attributes.position) {
      this.vertexCount = 0;
      return;
    }

    this.positions = this.geometry.attributes.position.array as Float32Array;
    this.vertexCount = this.geometry.attributes.position.count;

    // Ensure normals exist for back-face rejection
    if (!this.geometry.attributes.normal) {
      this.geometry.computeVertexNormals();
    }
    this.normals = this.geometry.attributes.normal
      ? (this.geometry.attributes.normal.array as Float32Array)
      : null;

    // Allocate thickness accumulation buffer
    this.thicknessArray = new Float32Array(this.vertexCount);

    // Initialize or bind vertex colors (R1)
    if (!this.geometry.attributes.color) {
      this.colors = new Float32Array(this.vertexCount * 3);
      // Default substrate: clean light gray (0.88, 0.88, 0.88)
      this.colors.fill(0.88);
      const colorAttr = new BufferAttribute(this.colors, 3);
      this.geometry.setAttribute('color', colorAttr);
    } else {
      this.colors = this.geometry.attributes.color.array as Float32Array;
    }

    // Build spatial uniform grid acceleration index (R6)
    this.buildSpatialGrid();
  }

  private cellKey(gx: number, gy: number, gz: number): number {
    const b = SurfaceStainingEngine.GRID_BIAS;
    const s = SurfaceStainingEngine.GRID_STRIDE;
    return ((gx + b) * s + (gy + b)) * s + (gz + b);
  }

  private buildSpatialGrid(): void {
    this.gridBuckets.clear();
    if (!this.positions || this.vertexCount === 0) return;

    const pos = this.positions;
    const invCell = 1.0 / this.cellSize;

    for (let i = 0; i < this.vertexCount; i++) {
      const i3 = i * 3;
      const gx = Math.floor(pos[i3] * invCell);
      const gy = Math.floor(pos[i3 + 1] * invCell);
      const gz = Math.floor(pos[i3 + 2] * invCell);
      const key = this.cellKey(gx, gy, gz);

      let bucket = this.gridBuckets.get(key);
      if (!bucket) {
        bucket = [];
        this.gridBuckets.set(key, bucket);
      }
      bucket.push(i);
    }
  }

  private updateCachedPaintColor(hex: string): void {
    if (this.currentPaintHex !== hex) {
      this.currentPaintHex = hex;
      this._tempPaintColor.set(hex);
      this.paintR = this._tempPaintColor.r;
      this.paintG = this._tempPaintColor.g;
      this.paintB = this._tempPaintColor.b;
    }
  }

  /**
   * Apply one spray step onto the workpiece mesh.
   * Runs at 60Hz or throttled 30Hz with zero dynamic allocations.
   */
  public applySprayStep(
    tcpWorldPos: Vector3,
    tcpWorldDir: Vector3,
    config: SpraySimConfig,
    deltaSec: number
  ): boolean {
    if (!this.mesh || !this.geometry || !this.positions || !this.thicknessArray || !this.colors) {
      return false;
    }

    this.updateCachedPaintColor(config.paintColor);

    // 1. Ensure mesh world matrix is fresh, then transform TCP World Pose to Mesh Local Coordinates (R5)
    this.mesh.updateWorldMatrix(true, false);
    this._invMatScratch.copy(this.mesh.matrixWorld).invert();
    this._tcpPos.copy(tcpWorldPos).applyMatrix4(this._invMatScratch);
    this._tcpDir.copy(tcpWorldDir).transformDirection(this._invMatScratch).normalize();

    const distM = config.targetDistanceMm / 1000.0;
    const widthM = config.sprayWidthMm / 1000.0;
    const tanHalf = Math.max(0.30, (widthM / 2.0) / distM);
    const minD = 0.005; // 5mm standoff threshold
    const maxD = Math.max(0.35, distM * 2.0);

    // 2. Compute local AABB of the conical spray volume to query intersecting spatial grid buckets (R6)
    const coneRadiusAtMax = maxD * tanHalf;
    const endX = this._tcpPos.x + this._tcpDir.x * maxD;
    const endY = this._tcpPos.y + this._tcpDir.y * maxD;
    const endZ = this._tcpPos.z + this._tcpDir.z * maxD;

    const minX = Math.min(this._tcpPos.x, endX) - coneRadiusAtMax - 0.05;
    const maxX = Math.max(this._tcpPos.x, endX) + coneRadiusAtMax + 0.05;
    const minY = Math.min(this._tcpPos.y, endY) - coneRadiusAtMax - 0.05;
    const maxY = Math.max(this._tcpPos.y, endY) + coneRadiusAtMax + 0.05;
    const minZ = Math.min(this._tcpPos.z, endZ) - coneRadiusAtMax - 0.05;
    const maxZ = Math.max(this._tcpPos.z, endZ) + coneRadiusAtMax + 0.05;

    const invCell = 1.0 / this.cellSize;
    const minGx = Math.floor(minX * invCell);
    const maxGx = Math.floor(maxX * invCell);
    const minGy = Math.floor(minY * invCell);
    const maxGy = Math.floor(maxY * invCell);
    const minGz = Math.floor(minZ * invCell);
    const maxGz = Math.floor(maxZ * invCell);

    // Range tracking for GPU incremental buffer update (R6)
    let minModified = Number.MAX_SAFE_INTEGER;
    let maxModified = -1;

    const pos = this.positions;
    const norm = this.normals;
    const thk = this.thicknessArray;
    const flowRate = config.flowRateMicronsPerSec;
    const isHeatmap = config.visualMode === 'heatmap';
    const effectiveRate = Math.max(30.0, flowRate * 3.0);

    // 3. Iterate candidate vertices in intersecting buckets
    for (let gx = minGx; gx <= maxGx; gx++) {
      for (let gy = minGy; gy <= maxGy; gy++) {
        for (let gz = minGz; gz <= maxGz; gz++) {
          const bucket = this.gridBuckets.get(this.cellKey(gx, gy, gz));
          if (!bucket) continue;

          for (let b = 0; b < bucket.length; b++) {
            const i = bucket[b];
            const i3 = i * 3;

            const dx = pos[i3] - this._tcpPos.x;
            const dy = pos[i3 + 1] - this._tcpPos.y;
            const dz = pos[i3 + 2] - this._tcpPos.z;

            // Axial projection distance
            const dAxial = dx * this._tcpDir.x + dy * this._tcpDir.y + dz * this._tcpDir.z;
            if (dAxial < minD || dAxial > maxD) continue;

            // Radial distance to center axis
            const perpX = dx - dAxial * this._tcpDir.x;
            const perpY = dy - dAxial * this._tcpDir.y;
            const perpZ = dz - dAxial * this._tcpDir.z;
            const radSq = perpX * perpX + perpY * perpY + perpZ * perpZ;

            const coneR = dAxial * tanHalf;
            if (radSq > coneR * coneR) continue;

            // Normal cosine attenuation (using absolute dot to support double-sided scanned meshes)
            let cosAlpha = 1.0;
            if (norm) {
              const nx = norm[i3];
              const ny = norm[i3 + 1];
              const nz = norm[i3 + 2];
              const dot = -(this._tcpDir.x * nx + this._tcpDir.y * ny + this._tcpDir.z * nz);
              cosAlpha = Math.max(0.25, Math.abs(dot));
            }

            // Radial Gaussian profile
            const sigma = coneR / 2.5;
            const wRadial = Math.exp(-radSq / (2.0 * sigma * sigma));

            // Axial distance falloff
            const distNorm = (dAxial - minD) / (maxD - minD);
            const wDist = Math.max(0, 1.0 - distNorm * distNorm);

            const deltaT = effectiveRate * wRadial * wDist * cosAlpha * deltaSec;
            if (deltaT > 0.0005) {
              thk[i] += deltaT;
              this.applyColorToVertex(i, thk[i], isHeatmap, config);

              if (i < minModified) minModified = i;
              if (i > maxModified) maxModified = i;
            }
          }
        }
      }
    }

    // 4. Submit incremental GPU update range (R6)
    if (maxModified >= minModified) {
      const colorAttr = this.geometry.attributes.color as BufferAttribute;
      if (colorAttr) {
        if (colorAttr.addUpdateRange && colorAttr.clearUpdateRanges) {
          colorAttr.clearUpdateRanges();
          const start = minModified * 3;
          const count = (maxModified - minModified + 1) * 3;
          colorAttr.addUpdateRange(start, count);
        }
        colorAttr.needsUpdate = true;
      }
      return true;
    }

    return false;
  }

  private applyColorToVertex(
    idx: number,
    thickness: number,
    isHeatmap: boolean,
    config: SpraySimConfig
  ): void {
    if (!this.colors) return;
    const i3 = idx * 3;

    if (isHeatmap) {
      // Heatmap Color Scale
      const tMin = config.targetMinThickness;
      const tMax = config.targetMaxThickness;

      if (thickness <= 0.05) {
        // Substrate gray
        this.colors[i3] = 0.88;
        this.colors[i3 + 1] = 0.88;
        this.colors[i3 + 2] = 0.88;
      } else if (thickness < tMin) {
        // Under-sprayed: Blue to Cyan
        const t = Math.min(1.0, thickness / tMin);
        this.colors[i3] = 0.1 * (1 - t) + 0.05 * t;
        this.colors[i3 + 1] = 0.3 * (1 - t) + 0.85 * t;
        this.colors[i3 + 2] = 0.9 * (1 - t) + 0.90 * t;
      } else if (thickness <= tMax) {
        // Standard Target Range: Emerald to Bright Green
        const t = (thickness - tMin) / Math.max(1, tMax - tMin);
        this.colors[i3] = 0.05 * (1 - t) + 0.40 * t;
        this.colors[i3 + 1] = 0.85 * (1 - t) + 0.95 * t;
        this.colors[i3 + 2] = 0.30 * (1 - t) + 0.10 * t;
      } else if (thickness <= tMax * 1.35) {
        // Slight Over-spray: Amber Yellow
        const t = (thickness - tMax) / (tMax * 0.35);
        this.colors[i3] = 0.40 * (1 - t) + 0.98 * t;
        this.colors[i3 + 1] = 0.95 * (1 - t) + 0.65 * t;
        this.colors[i3 + 2] = 0.10 * (1 - t) + 0.05 * t;
      } else {
        // Severe Over-spray: Vivid Red
        this.colors[i3] = 0.92;
        this.colors[i3 + 1] = 0.12;
        this.colors[i3 + 2] = 0.15;
      }
    } else {
      // Realistic Coating Mode: Substrate to Paint Color with Opacity Build-up
      if (thickness <= 0.05) {
        this.colors[i3] = 0.88;
        this.colors[i3 + 1] = 0.88;
        this.colors[i3 + 2] = 0.88;
      } else {
        const coverFactor = Math.min(1.0, 0.35 + 0.65 * (1.0 - Math.exp(-2.5 * (thickness / 25.0))));
        const sub = 0.88;
        this.colors[i3] = (1.0 - coverFactor) * sub + coverFactor * this.paintR;
        this.colors[i3 + 1] = (1.0 - coverFactor) * sub + coverFactor * this.paintG;
        this.colors[i3 + 2] = (1.0 - coverFactor) * sub + coverFactor * this.paintB;
      }
    }
  }

  /**
   * Re-color all vertices when visual mode or paint color changes.
   */
  public refreshAllColors(config: SpraySimConfig): void {
    if (!this.colors || !this.thicknessArray) return;
    this.updateCachedPaintColor(config.paintColor);
    const isHeatmap = config.visualMode === 'heatmap';

    for (let i = 0; i < this.vertexCount; i++) {
      const thk = this.thicknessArray[i];
      if (thk > 0) {
        this.applyColorToVertex(i, thk, isHeatmap, config);
      } else {
        const i3 = i * 3;
        this.colors[i3] = 0.88;
        this.colors[i3 + 1] = 0.88;
        this.colors[i3 + 2] = 0.88;
      }
    }

    const colorAttr = this.geometry?.attributes.color as BufferAttribute;
    if (colorAttr) {
      if (colorAttr.clearUpdateRanges) colorAttr.clearUpdateRanges();
      colorAttr.needsUpdate = true;
    }
  }

  /**
   * Reset all accumulated paint and restore clean substrate.
   */
  public clearCoating(): void {
    if (this.thicknessArray) {
      this.thicknessArray.fill(0);
    }
    if (this.colors) {
      this.colors.fill(0.88);
    }
    const colorAttr = this.geometry?.attributes.color as BufferAttribute;
    if (colorAttr) {
      if (colorAttr.clearUpdateRanges) colorAttr.clearUpdateRanges();
      colorAttr.needsUpdate = true;
    }
  }

  /**
   * Compute coverage statistics (Indicative).
   */
  public computeStats(config: SpraySimConfig): SprayCoverageStats {
    if (!this.thicknessArray || this.vertexCount === 0) {
      return {
        totalVertices: 0,
        coveredVertices: 0,
        coveragePercentage: 0,
        averageThickness: 0,
        maxThickness: 0,
        standardComplianceRate: 0,
      };
    }

    const thk = this.thicknessArray;
    let covered = 0;
    let sumThk = 0;
    let maxThk = 0;
    let compliant = 0;
    const tMin = config.targetMinThickness;
    const tMax = config.targetMaxThickness;

    for (let i = 0; i < this.vertexCount; i++) {
      const t = thk[i];
      if (t > 0.5) {
        covered++;
        sumThk += t;
        if (t > maxThk) maxThk = t;
        if (t >= tMin && t <= tMax) compliant++;
      }
    }

    const coveragePct = (covered / this.vertexCount) * 100.0;
    const avgThk = covered > 0 ? sumThk / covered : 0.0;
    const complianceRate = covered > 0 ? (compliant / covered) * 100.0 : 0.0;

    return {
      totalVertices: this.vertexCount,
      coveredVertices: covered,
      coveragePercentage: Math.round(coveragePct * 10) / 10,
      averageThickness: Math.round(avgThk * 10) / 10,
      maxThickness: Math.round(maxThk * 10) / 10,
      standardComplianceRate: Math.round(complianceRate * 10) / 10,
    };
  }
}
