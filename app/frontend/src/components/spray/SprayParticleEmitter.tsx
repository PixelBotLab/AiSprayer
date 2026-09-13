import React, { useRef, useMemo, useEffect } from 'react';
import { useFrame } from '@react-three/fiber';
import {
  Object3D,
  Vector3,
  Quaternion,
  BufferGeometry,
  Points,
  Mesh,
  CylinderGeometry,
  SphereGeometry,
  MeshBasicMaterial,
  ShaderMaterial,
  CanvasTexture,
  NormalBlending,
  DoubleSide,
  Color
} from 'three';

interface SprayParticleEmitterProps {
  tcpLinkRef: React.MutableRefObject<Object3D | null>;
  isSpraying: boolean;
  sprayDistMm?: number;
  sprayWidthMm?: number;
  paintColor?: string;
}

// Total particle capacity for rich, continuous mist density
const MAX_PARTICLES = 1200;

// Create a soft radial Gaussian-like mist texture once on client
function createSoftMistTexture(): CanvasTexture {
  const size = 64;
  const canvas = document.createElement('canvas');
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext('2d')!;

  const grad = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  grad.addColorStop(0, 'rgba(255, 255, 255, 1.0)');
  grad.addColorStop(0.25, 'rgba(255, 255, 255, 0.85)');
  grad.addColorStop(0.55, 'rgba(255, 255, 255, 0.35)');
  grad.addColorStop(0.85, 'rgba(255, 255, 255, 0.08)');
  grad.addColorStop(1.0, 'rgba(255, 255, 255, 0.0)');

  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, size, size);

  const texture = new CanvasTexture(canvas);
  texture.needsUpdate = true;
  return texture;
}

export const SprayParticleEmitter: React.FC<SprayParticleEmitterProps> = ({
  tcpLinkRef,
  isSpraying,
  sprayDistMm = 150,
  sprayWidthMm = 60,
  paintColor = '#2563eb',
}) => {
  const pointsRef = useRef<Points | null>(null);
  const geomRef = useRef<BufferGeometry | null>(null);
  const coneMeshRef = useRef<Mesh | null>(null);
  const nozzleMeshRef = useRef<Mesh | null>(null);

  // Pre-allocated particle simulation arrays (Zero-GC)
  const positions = useMemo(() => new Float32Array(MAX_PARTICLES * 3), []);
  const velocities = useMemo(() => new Float32Array(MAX_PARTICLES * 3), []);
  const ages = useMemo(() => new Float32Array(MAX_PARTICLES), []);
  const lifetimes = useMemo(() => new Float32Array(MAX_PARTICLES), []);
  const progressArr = useMemo(() => new Float32Array(MAX_PARTICLES), []);
  const particleActive = useMemo(() => new Uint8Array(MAX_PARTICLES), []);

  // Pre-allocated scratch objects for Zero-GC matrix/vector operations
  const scratchTcpPos = useMemo(() => new Vector3(), []);
  const scratchTcpDir = useMemo(() => new Vector3(), []);
  const scratchTcpRight = useMemo(() => new Vector3(), []);
  const scratchTcpUp = useMemo(() => new Vector3(), []);
  const scratchTcpQuat = useMemo(() => new Quaternion(), []);

  // Shared soft radial mist texture
  const mistTexture = useMemo(() => createSoftMistTexture(), []);

  // Ring buffer cursor for continuous, deterministic particle spawning
  const spawnCursorRef = useRef<number>(0);

  // Base cone geometry: Unit height (1.0), unit base radius (1.0), nozzle apex (0.04)
  // Aligned along local +Z axis from z = 0 (apex) to z = +1.0 (base)
  const baseConeGeom = useMemo(() => {
    // CylinderGeometry(radiusTop, radiusBottom, height, radialSegments, heightSegments, openEnded)
    const geom = new CylinderGeometry(0.04, 1.0, 1.0, 32, 4, true);
    // Move apex to origin (0, 0, 0)
    geom.translate(0, -0.5, 0);
    // Rotate so apex points along +Z
    geom.rotateX(-Math.PI / 2);
    return geom;
  }, []);

  // Custom Volumetric Mist Cone Shader Material
  const coneMat = useMemo(() => {
    return new ShaderMaterial({
      uniforms: {
        uColor: { value: new Color(paintColor) },
        uOpacity: { value: 0.28 },
        uTime: { value: 0.0 },
      },
      vertexShader: `
        varying vec2 vUv;
        varying vec3 vNormal;
        varying vec3 vViewPos;
        void main() {
          vUv = uv;
          vNormal = normalize(normalMatrix * normal);
          vec4 mvPos = modelViewMatrix * vec4(position, 1.0);
          vViewPos = -mvPos.xyz;
          gl_Position = projectionMatrix * mvPos;
        }
      `,
      fragmentShader: `
        uniform vec3 uColor;
        uniform float uOpacity;
        uniform float uTime;
        varying vec2 vUv;
        varying vec3 vNormal;
        varying vec3 vViewPos;

        void main() {
          // Longitudinal density profile: densest at nozzle (vUv.y = 0.0), diffusing at target (1.0)
          float longitudinal = pow(1.0 - vUv.y * 0.72, 1.3);

          // Volumetric rim glow (Fresnel effect along cylinder curvature)
          vec3 viewDir = normalize(vViewPos);
          float rim = 1.0 - abs(dot(vNormal, viewDir));
          float rimGlow = pow(rim, 1.4);

          // Animated high-speed atomized paint stream striations
          float stream1 = sin(vUv.x * 24.0 + uTime * 12.0);
          float stream2 = sin(vUv.x * 48.0 - uTime * 7.0 + vUv.y * 8.0);
          float flow = 0.85 + 0.15 * (stream1 * 0.6 + stream2 * 0.4);

          // Mist alpha
          float alpha = uOpacity * longitudinal * (0.35 + 0.65 * rimGlow) * flow;

          // Luminous atomization core highlight near nozzle orifice
          vec3 coreColor = mix(uColor, vec3(1.0), 0.28 * (1.0 - vUv.y));

          gl_FragColor = vec4(coreColor, clamp(alpha, 0.0, 1.0));
        }
      `,
      transparent: true,
      depthWrite: false,
      side: DoubleSide,
      blending: NormalBlending,
    });
  }, []);

  // Custom Particle Mist Points Shader Material
  const particleMat = useMemo(() => {
    return new ShaderMaterial({
      uniforms: {
        uTexture: { value: mistTexture },
        uColor: { value: new Color(paintColor) },
        uOpacity: { value: 0.75 },
        uSizeStart: { value: 0.006 }, // 6mm droplet at nozzle
        uSizeEnd: { value: 0.032 },   // 32mm expanding mist cloud at target
        uScale: { value: 400.0 },     // Perspective scale factor
      },
      vertexShader: `
        attribute float aProgress;
        varying float vProgress;

        uniform float uSizeStart;
        uniform float uSizeEnd;
        uniform float uScale;

        void main() {
          vProgress = aProgress;
          vec4 mvPos = modelViewMatrix * vec4(position, 1.0);

          // Mist droplet size expands from nozzle to target
          float pSize = mix(uSizeStart, uSizeEnd, aProgress);
          gl_PointSize = clamp(pSize * (uScale / -mvPos.z), 2.0, 64.0);
          gl_Position = projectionMatrix * mvPos;
        }
      `,
      fragmentShader: `
        uniform sampler2D uTexture;
        uniform vec3 uColor;
        uniform float uOpacity;
        varying float vProgress;

        void main() {
          vec4 tex = texture2D(uTexture, gl_PointCoord);

          // Fast fade-in at nozzle exit, smooth fade-out as droplets deposit on workpiece
          float fadeIn = smoothstep(0.0, 0.10, vProgress);
          float fadeOut = 1.0 - smoothstep(0.72, 1.0, vProgress);
          float alpha = tex.a * uOpacity * fadeIn * fadeOut;

          if (alpha < 0.008) discard;

          // Luminous core for liquid atomization look
          vec3 dropletColor = mix(uColor, vec3(1.0), 0.22 * (1.0 - vProgress) * tex.a);
          gl_FragColor = vec4(dropletColor, alpha);
        }
      `,
      transparent: true,
      depthWrite: false,
      blending: NormalBlending,
    });
  }, [mistTexture]);

  // Small luminous nozzle focal point geometry & material
  const nozzleGeom = useMemo(() => new SphereGeometry(0.004, 12, 12), []);
  const nozzleMat = useMemo(() => {
    return new MeshBasicMaterial({
      color: new Color('#ffffff'),
      transparent: true,
      opacity: 0.85,
      depthWrite: false,
    });
  }, []);

  // Initialize particle positions off-screen
  useEffect(() => {
    positions.fill(9999.0);
    progressArr.fill(0);
    particleActive.fill(0);
    if (geomRef.current) {
      if (geomRef.current.attributes.position) {
        geomRef.current.attributes.position.needsUpdate = true;
      }
      if (geomRef.current.attributes.aProgress) {
        geomRef.current.attributes.aProgress.needsUpdate = true;
      }
    }
  }, [positions, progressArr, particleActive]);

  // Synchronize color changes with materials
  useEffect(() => {
    const c = new Color(paintColor);
    coneMat.uniforms.uColor.value.copy(c);
    particleMat.uniforms.uColor.value.copy(c);
  }, [paintColor, coneMat, particleMat]);

  // Frame-rate particle simulation loop
  useFrame((state, delta) => {
    const clampedDelta = Math.min(delta, 0.05);

    // Update shader animated time
    coneMat.uniforms.uTime.value = state.clock.elapsedTime;
    // Adapt particle scale to viewport height for consistent on-screen size
    particleMat.uniforms.uScale.value = Math.max(300.0, state.size.height / 2.0);

    // 1. Resolve TCP World Position & Local Coordinate Frame
    let hasTcp = false;
    const tcpLink = tcpLinkRef.current;
    if (tcpLink) {
      tcpLink.getWorldPosition(scratchTcpPos);
      tcpLink.getWorldQuaternion(scratchTcpQuat);

      // Extract local tool orthonormal triad directly from world quaternion (Zero-GC)
      scratchTcpDir.set(0, 0, 1).applyQuaternion(scratchTcpQuat).normalize();
      scratchTcpRight.set(1, 0, 0).applyQuaternion(scratchTcpQuat).normalize();
      scratchTcpUp.set(0, 1, 0).applyQuaternion(scratchTcpQuat).normalize();
      hasTcp = true;
    }

    const distM = sprayDistMm / 1000.0;
    const widthM = sprayWidthMm / 1000.0;
    const radiusM = widthM / 2.0;
    const halfAngle = Math.atan2(radiusM, distM);

    // 2. Update Volumetric Mist Cone Envelope
    if (coneMeshRef.current) {
      if (isSpraying && hasTcp) {
        coneMeshRef.current.visible = true;
        coneMeshRef.current.position.copy(scratchTcpPos);
        coneMeshRef.current.quaternion.copy(scratchTcpQuat);
        coneMeshRef.current.scale.set(radiusM, radiusM, distM);
      } else {
        coneMeshRef.current.visible = false;
      }
    }

    // 2b. Update Nozzle Atomization Core Mesh
    if (nozzleMeshRef.current) {
      if (isSpraying && hasTcp) {
        nozzleMeshRef.current.visible = true;
        nozzleMeshRef.current.position.copy(scratchTcpPos);
      } else {
        nozzleMeshRef.current.visible = false;
      }
    }

    if (!pointsRef.current || !geomRef.current) return;

    // 3. Dense Continuous Particle Mist Simulation
    // Mist droplets have exit speed ~1.1 - 1.5 m/s, reaching 150mm in ~0.12 - 0.18s
    const nominalSpeed = 1.15;
    const avgLifetime = distM / nominalSpeed;

    // Calculate spawn quota for this frame: steady stream of ~6000 particles/sec
    const particlesToSpawn = (isSpraying && hasTcp)
      ? Math.min(120, Math.floor(clampedDelta * (MAX_PARTICLES / avgLifetime) * 1.1) + 1)
      : 0;

    // Spawn new particles into circular ring buffer
    for (let k = 0; k < particlesToSpawn; k++) {
      const idx = spawnCursorRef.current;
      spawnCursorRef.current = (idx + 1) % MAX_PARTICLES;
      const i3 = idx * 3;

      particleActive[idx] = 1;
      ages[idx] = 0;

      // Central bias for cone density (realistic Gaussian-like atomization)
      const radialFrac = Math.pow(Math.random(), 0.65);
      const coneAngle = halfAngle * radialFrac;
      const phi = Math.random() * Math.PI * 2;

      const sinA = Math.sin(coneAngle);
      const cosA = Math.cos(coneAngle);

      // Conical direction vector (intrinsically unit length)
      const dirX = scratchTcpDir.x * cosA + (scratchTcpRight.x * Math.cos(phi) + scratchTcpUp.x * Math.sin(phi)) * sinA;
      const dirY = scratchTcpDir.y * cosA + (scratchTcpRight.y * Math.cos(phi) + scratchTcpUp.y * Math.sin(phi)) * sinA;
      const dirZ = scratchTcpDir.z * cosA + (scratchTcpRight.z * Math.cos(phi) + scratchTcpUp.z * Math.sin(phi)) * sinA;

      // Speed is slightly higher in the central core, tapering toward cone boundary
      const pSpeed = nominalSpeed * (1.08 - 0.20 * radialFrac + (Math.random() - 0.5) * 0.12);

      velocities[i3] = dirX * pSpeed;
      velocities[i3 + 1] = dirY * pSpeed;
      velocities[i3 + 2] = dirZ * pSpeed;

      // Exact time required to reach target standoff distance
      lifetimes[idx] = (distM / (pSpeed * Math.max(0.1, cosA))) * (0.95 + Math.random() * 0.12);

      // Nozzle orifice exit with tiny jitter
      const rNozzle = 0.0018 * Math.sqrt(Math.random());
      const thNozzle = Math.random() * Math.PI * 2;
      positions[i3] = scratchTcpPos.x + (scratchTcpRight.x * Math.cos(thNozzle) + scratchTcpUp.x * Math.sin(thNozzle)) * rNozzle;
      positions[i3 + 1] = scratchTcpPos.y + (scratchTcpRight.y * Math.cos(thNozzle) + scratchTcpUp.y * Math.sin(thNozzle)) * rNozzle;
      positions[i3 + 2] = scratchTcpPos.z + (scratchTcpRight.z * Math.cos(thNozzle) + scratchTcpUp.z * Math.sin(thNozzle)) * rNozzle;
      progressArr[idx] = 0.0;
    }

    // Advance all active particles
    let anyActive = false;
    for (let i = 0; i < MAX_PARTICLES; i++) {
      if (particleActive[i] === 1) {
        ages[i] += clampedDelta;
        const prog = ages[i] / lifetimes[i];

        if (prog >= 1.0) {
          // Particle reached the workpiece: deactivate
          particleActive[i] = 0;
          positions[i * 3] = 9999.0;
          positions[i * 3 + 1] = 9999.0;
          positions[i * 3 + 2] = 9999.0;
          progressArr[i] = 1.0;
        } else {
          // Integrate velocity
          const i3 = i * 3;
          positions[i3] += velocities[i3] * clampedDelta;
          positions[i3 + 1] += velocities[i3 + 1] * clampedDelta;
          positions[i3 + 2] += velocities[i3 + 2] * clampedDelta;
          progressArr[i] = prog;
          anyActive = true;
        }
      }
    }

    // Flag GPU buffer update when particles are in flight
    if (anyActive || isSpraying || particlesToSpawn > 0) {
      geomRef.current.attributes.position.needsUpdate = true;
      (geomRef.current.attributes as any).aProgress.needsUpdate = true;
    }
  });

  return (
    <group>
      {/* 1. Volumetric Mist Cone Envelope */}
      <mesh
        ref={coneMeshRef}
        geometry={baseConeGeom}
        material={coneMat}
        renderOrder={1998}
      />

      {/* 2. Nozzle Atomization Focal Point */}
      <mesh
        ref={nozzleMeshRef}
        geometry={nozzleGeom}
        material={nozzleMat}
        renderOrder={1999}
      />

      {/* 3. Dense Continuous Atomized Particle Mist Stream */}
      <points ref={pointsRef} material={particleMat} renderOrder={2000}>
        <bufferGeometry ref={geomRef}>
          <bufferAttribute
            attach="attributes-position"
            args={[positions, 3]}
          />
          <bufferAttribute
            attach="attributes-aProgress"
            args={[progressArr, 1]}
          />
        </bufferGeometry>
      </points>
    </group>
  );
};
