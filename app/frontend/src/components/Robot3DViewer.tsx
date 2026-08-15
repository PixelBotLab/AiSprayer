import React, { useEffect, useState, useRef } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import URDFLoader from 'urdf-loader';
import { Maximize2, Minimize2, Eye, EyeOff, Box, Grid3X3, Route } from 'lucide-react';
import {
  Object3D,
  Group,
  LoadingManager,
  Light,
  Mesh,
  Line,
  LineBasicMaterial,
  MeshPhongMaterial,
  MeshStandardMaterial,
  SphereGeometry,
  ConeGeometry,
  Color,
  DoubleSide,
  BufferGeometry,
  Vector3,
  Quaternion,
  Sprite,
  SpriteMaterial,
  CanvasTexture
} from 'three';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';

interface RobotModelProps {
  jointAngles: number[];
  activeTemplate: string | null;
  meshVersion: number;
  pathsVersion: number;
  isMeshVisible: boolean;
  isWireframe: boolean;
  isPathsVisible: boolean;
  onMeshLoaded?: (vertexCount: number) => void;
  onPathsLoaded?: (pathCount: number, pointCount: number) => void;
}

// Joint names in CR5 URDF matching the order J1..J6
const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

function stripEmbeddedLights(root: Object3D) {
  const lights: Light[] = [];
  root.traverse((obj) => {
    if ((obj as Light).isLight) lights.push(obj as Light);
  });
  lights.forEach((light) => light.parent?.remove(light));
}

function loadUrdfMesh(
  path: string,
  manager: LoadingManager,
  doneOrMaterial: any,
  doneCallback?: (obj: Object3D, err?: Error) => void,
) {
  const done = typeof doneOrMaterial === 'function' ? doneOrMaterial : doneCallback;
  const material = typeof doneOrMaterial !== 'function' ? doneOrMaterial : undefined;
  if (!done) return;

  if (/\.stl$/i.test(path)) {
    const loader = new STLLoader(manager);
    loader.load(
      path,
      (geom) => {
        done(new Mesh(geom, material ?? new MeshPhongMaterial({ color: 0xb0b0b0, side: DoubleSide })));
      },
      undefined,
      (err) => done(new Object3D(), err as Error),
    );
    return;
  }

  if (/\.dae$/i.test(path)) {
    const loader = new ColladaLoader(manager);
    loader.load(
      path,
      (dae) => {
        if (dae && dae.scene) {
          stripEmbeddedLights(dae.scene);
          done(dae.scene);
        } else {
          done(new Object3D(), new Error(`Invalid DAE file: ${path}`));
        }
      },
      undefined,
      (err) => done(new Object3D(), err as Error),
    );
    return;
  }

  console.warn(`URDFLoader: no mesh loader for ${path}`);
  done(new Object3D(), new Error(`Unsupported mesh type: ${path}`));
}

const RobotModel: React.FC<RobotModelProps> = ({ 
  jointAngles,
  activeTemplate,
  meshVersion,
  pathsVersion,
  isMeshVisible,
  isWireframe,
  isPathsVisible,
  onMeshLoaded,
  onPathsLoaded
}) => {
  const [robot, setRobot] = useState<Object3D | null>(null);
  const surfaceMeshRef = useRef<Mesh | null>(null);
  const surfaceMaterialRef = useRef<MeshStandardMaterial | null>(null);
  const pathsGroupRef = useRef<Group | null>(null);

  // 1. Load Robot URDF Model
  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);

    loader.packages = {
      dobot_rviz: 'http://localhost:8000/urdf',
      dobot_gazebo_sim: 'http://localhost:8000/urdf',
    };
    (loader as any).loadMeshCb = loadUrdfMesh;

    loader.load(`http://localhost:8000/urdf/cr5_robot.urdf?v=${Date.now()}`, (r: any) => {
      stripEmbeddedLights(r);
      r.rotation.x = -Math.PI / 2;
      r.traverse((obj: Object3D) => {
        const mesh = obj as Mesh;
        if (!mesh.isMesh) return;
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        mats.forEach((m: any) => {
          if (!m) return;
          m.side = DoubleSide;
          m.needsUpdate = true;
          if (m.color && m.color instanceof Color) {
            const { r: cr, g: cg, b: cb } = m.color;
            if (cr + cg + cb < 0.05) m.color.setRGB(0.08, 0.08, 0.08);
          }
        });
      });
      setRobot(r);
    });
  }, []);

  // 2. Update Joint Angles Dynamically
  useEffect(() => {
    if (!robot) return;
    const r = robot as any;
    JOINT_NAMES.forEach((name, idx) => {
      if (r.setJointValue) {
        r.setJointValue(name, (jointAngles[idx] ?? 0) * (Math.PI / 180));
      }
    });
  }, [robot, jointAngles]);

  // 3. Load and Attach Reconstructed Surface Mesh to Robot base_link
  useEffect(() => {
    if (!robot) return;
    const r = robot as any;
    const baseLink = r.links?.['base_link'] || r.getObjectByName('base_link') || robot;

    // Clean up any existing surface mesh
    if (surfaceMeshRef.current) {
      if (surfaceMeshRef.current.parent) {
        surfaceMeshRef.current.parent.remove(surfaceMeshRef.current);
      }
      surfaceMeshRef.current.geometry?.dispose();
      surfaceMeshRef.current = null;
    }

    if (!activeTemplate) {
      if (onMeshLoaded) onMeshLoaded(0);
      return;
    }

    const plyUrl = `http://localhost:8000/templates/${activeTemplate}/scan.mesh.ply?v=${meshVersion}`;
    const stlUrl = `http://localhost:8000/templates/${activeTemplate}/scan.mesh.stl?v=${meshVersion}`;

    if (!surfaceMaterialRef.current) {
      surfaceMaterialRef.current = new MeshStandardMaterial({
        color: new Color('#ffffff'),
        roughness: 0.35,
        metalness: 0.20,
        wireframe: isWireframe,
        side: DoubleSide,
        transparent: true,
        opacity: 0.92,
        polygonOffset: true,
        polygonOffsetFactor: 1.0,
        polygonOffsetUnits: 1.0,
      });
    }

    const attachGeometry = (geom: BufferGeometry) => {
      geom.computeVertexNormals();
      const count = geom.attributes.position ? geom.attributes.position.count : 0;
      
      const mesh = new Mesh(geom, surfaceMaterialRef.current!);
      mesh.name = "reconstructed_surface_mesh";
      mesh.visible = isMeshVisible;
      mesh.castShadow = true;
      mesh.receiveShadow = true;

      baseLink.add(mesh);
      surfaceMeshRef.current = mesh;

      if (onMeshLoaded) onMeshLoaded(count);
    };

    const plyLoader = new PLYLoader();
    plyLoader.load(
      plyUrl,
      (geom) => {
        attachGeometry(geom);
      },
      undefined,
      () => {
        const stlLoader = new STLLoader();
        stlLoader.load(
          stlUrl,
          (geom) => {
            attachGeometry(geom);
          },
          undefined,
          () => {
            if (onMeshLoaded) onMeshLoaded(0);
          }
        );
      }
    );

    return () => {
      if (surfaceMeshRef.current && surfaceMeshRef.current.parent) {
        surfaceMeshRef.current.parent.remove(surfaceMeshRef.current);
        surfaceMeshRef.current.geometry?.dispose();
        surfaceMeshRef.current = null;
      }
    };
  }, [robot, activeTemplate, meshVersion]);

  // 4. Load and Attach 3D Manual TCP Paths to Robot base_link
  useEffect(() => {
    if (!robot) return;
    const r = robot as any;
    const baseLink = r.links?.['base_link'] || r.getObjectByName('base_link') || robot;

    // Clean up previous path group
    if (pathsGroupRef.current) {
      if (pathsGroupRef.current.parent) {
        pathsGroupRef.current.parent.remove(pathsGroupRef.current);
      }
      pathsGroupRef.current = null;
    }

    if (!activeTemplate) {
      if (onPathsLoaded) onPathsLoaded(0, 0);
      return;
    }

    const fetchPaths = async () => {
      try {
        const res = await fetch(`http://localhost:8000/api/interactive/templates/${activeTemplate}/manual_paths?t=${pathsVersion}`);
        if (!res.ok) {
          if (onPathsLoaded) onPathsLoaded(0, 0);
          return;
        }

        const data = await res.json();
        const paths = data.paths || [];
        if (paths.length === 0) {
          if (onPathsLoaded) onPathsLoaded(0, 0);
          return;
        }

        const group = new Group();
        group.name = "manual_tcp_paths_group";
        group.visible = isPathsVisible;

        let totalPts = 0;

        paths.forEach((pathItem: any) => {
          const pts = pathItem.points || [];
          if (pts.length === 0) return;
          totalPts += pts.length;

          // A. 3D Surface Contour Line (Conforms along surface with normal offset)
          if (pts.length > 1) {
            const linePoints: Vector3[] = [];
            const densePts = pathItem.dense_surface_points_base_mm || [];
            
            if (densePts.length > 1) {
              // 1. Use high-density surface point cloud from depth map
              densePts.forEach((dp: [number, number, number]) => {
                linePoints.push(new Vector3(
                  dp[0] / 1000.0,
                  dp[1] / 1000.0,
                  dp[2] / 1000.0 + 0.0015
                ));
              });
            } else {
              // 2. Dense normal-interpolated contour segments lifted along surface normal
              for (let i = 0; i < pts.length - 1; i++) {
                const p1 = pts[i];
                const p2 = pts[i + 1];
                const pos1 = new Vector3(
                  p1.surface_point_base_mm[0] / 1000.0,
                  p1.surface_point_base_mm[1] / 1000.0,
                  p1.surface_point_base_mm[2] / 1000.0
                );
                const pos2 = new Vector3(
                  p2.surface_point_base_mm[0] / 1000.0,
                  p2.surface_point_base_mm[1] / 1000.0,
                  p2.surface_point_base_mm[2] / 1000.0
                );
                const norm1 = new Vector3(
                  p1.surface_normal_base[0],
                  p1.surface_normal_base[1],
                  p1.surface_normal_base[2]
                ).normalize();
                const norm2 = new Vector3(
                  p2.surface_normal_base[0],
                  p2.surface_normal_base[1],
                  p2.surface_normal_base[2]
                ).normalize();

                const steps = 16;
                for (let s = 0; s <= steps; s++) {
                  if (i > 0 && s === 0) continue;
                  const t = s / steps;
                  const pt = new Vector3().lerpVectors(pos1, pos2, t);
                  const nt = new Vector3().lerpVectors(norm1, norm2, t).normalize();
                  // Lift +2.0mm outward along local surface normal so it is 100% visible on surface
                  pt.addScaledVector(nt, 0.002);
                  linePoints.push(pt);
                }
              }
            }

            const lineGeom = new BufferGeometry().setFromPoints(linePoints);
            // Deep rich red surface trajectory
            const lineMat = new LineBasicMaterial({ color: 0xb91c1c, linewidth: 3 });
            const lineMesh = new Line(lineGeom, lineMat);
            lineMesh.renderOrder = 999;
            group.add(lineMesh);

            // Also draw TCP-level connecting line (dashed appearance via segments)
            const tcpLinePoints: Vector3[] = [];
            pts.forEach((p: any) => {
              tcpLinePoints.push(new Vector3(
                p.tcp_pose_base.x / 1000.0,
                p.tcp_pose_base.y / 1000.0,
                p.tcp_pose_base.z / 1000.0
              ));
            });
            const tcpLineGeom = new BufferGeometry().setFromPoints(tcpLinePoints);
            const tcpLineMat = new LineBasicMaterial({ color: 0x22c55e, linewidth: 1 });
            const tcpLineMesh = new Line(tcpLineGeom, tcpLineMat);
            tcpLineMesh.renderOrder = 1000;
            group.add(tcpLineMesh);
          }

          // Helper to create a crisp 2D billboard sprite with point number (Dark Green Badge)
          const createNumberSprite = (num: number): Sprite => {
            const canvas = document.createElement('canvas');
            canvas.width = 64;
            canvas.height = 64;
            const ctx = canvas.getContext('2d');
            if (ctx) {
              ctx.beginPath();
              ctx.arc(32, 32, 28, 0, Math.PI * 2);
              ctx.fillStyle = '#064e3b'; // Deep forest green for high contrast
              ctx.fill();
              ctx.lineWidth = 4;
              ctx.strokeStyle = '#ffffff'; // Crisp white border
              ctx.stroke();

              ctx.fillStyle = '#ffffff';
              ctx.font = 'bold 36px monospace, sans-serif';
              ctx.textAlign = 'center';
              ctx.textBaseline = 'middle';
              ctx.fillText(`${num}`, 32, 33);
            }
            const texture = new CanvasTexture(canvas);
            const spriteMat = new SpriteMaterial({ map: texture, depthTest: false });
            const sprite = new Sprite(spriteMat);
            sprite.scale.set(0.018, 0.018, 1);
            return sprite;
          };

          // B. Each Waypoint Details
          pts.forEach((p: any, idx: number) => {
            const surfPos = new Vector3(
              p.surface_point_base_mm[0] / 1000.0,
              p.surface_point_base_mm[1] / 1000.0,
              p.surface_point_base_mm[2] / 1000.0
            );

            const tcpPos = new Vector3(
              p.tcp_pose_base.x / 1000.0,
              p.tcp_pose_base.y / 1000.0,
              p.tcp_pose_base.z / 1000.0
            );

            // 1. Surface Point Sphere — deep red
            const surfSphereGeom = new SphereGeometry(0.005, 12, 12);
            const surfSphereMat = new MeshStandardMaterial({ color: 0xef4444, roughness: 0.3 });
            const surfSphere = new Mesh(surfSphereGeom, surfSphereMat);
            surfSphere.position.copy(surfPos);
            group.add(surfSphere);

            // 2. Normal Line from surface to TCP — pure red
            const normalLineGeom = new BufferGeometry().setFromPoints([surfPos, tcpPos]);
            const normalLineMat = new LineBasicMaterial({ color: 0xef4444, linewidth: 2 });
            const normalLine = new Line(normalLineGeom, normalLineMat);
            group.add(normalLine);

            // 3. TCP Target Point Sphere — bright green, clearly distinct
            const tcpSphereGeom = new SphereGeometry(0.007, 12, 12);
            const tcpSphereMat = new MeshStandardMaterial({ color: 0x22c55e, roughness: 0.2, metalness: 0.5, emissive: 0x166534, emissiveIntensity: 0.3 });
            const tcpSphere = new Mesh(tcpSphereGeom, tcpSphereMat);
            tcpSphere.position.copy(tcpPos);
            group.add(tcpSphere);

            // 3.5 Numbered Digital Badge Billboard on TCP Point
            const numSprite = createNumberSprite(p.index || (idx + 1));
            numSprite.position.set(tcpPos.x, tcpPos.y, tcpPos.z + 0.012);
            group.add(numSprite);

            // 4. Spray approach cone at TCP pointing towards surface — pure red
            const dir = new Vector3().subVectors(surfPos, tcpPos).normalize();
            const coneGeom = new ConeGeometry(0.004, 0.014, 8);
            coneGeom.translate(0, -0.007, 0);
            const coneMat = new MeshStandardMaterial({ color: 0xf87171 });
            const cone = new Mesh(coneGeom, coneMat);
            cone.position.copy(tcpPos);
            const up = new Vector3(0, -1, 0);
            const q = new Quaternion().setFromUnitVectors(up, dir);
            cone.setRotationFromQuaternion(q);
            group.add(cone);
          });
        });

        baseLink.add(group);
        pathsGroupRef.current = group;

        if (onPathsLoaded) onPathsLoaded(paths.length, totalPts);
      } catch (e) {
        console.error("Failed to render 3D manual paths in Robot3DViewer:", e);
        if (onPathsLoaded) onPathsLoaded(0, 0);
      }
    };

    fetchPaths();

    return () => {
      if (pathsGroupRef.current && pathsGroupRef.current.parent) {
        pathsGroupRef.current.parent.remove(pathsGroupRef.current);
        pathsGroupRef.current = null;
      }
    };
  }, [robot, activeTemplate, pathsVersion]);

  // 5. Handle Visibility and Wireframe Changes
  useEffect(() => {
    if (surfaceMeshRef.current) {
      surfaceMeshRef.current.visible = isMeshVisible;
    }
  }, [isMeshVisible]);

  useEffect(() => {
    if (pathsGroupRef.current) {
      pathsGroupRef.current.visible = isPathsVisible;
    }
  }, [isPathsVisible]);

  useEffect(() => {
    if (surfaceMaterialRef.current) {
      surfaceMaterialRef.current.wireframe = isWireframe;
      surfaceMaterialRef.current.needsUpdate = true;
    }
  }, [isWireframe]);

  if (!robot) return null;
  return <primitive object={robot} />;
};

interface Robot3DViewerProps {
  jointAngles?: number[];
  activeTemplate?: string | null;
  meshVersion?: number;
  pathsVersion?: number;
}

const TOOLTIP_CLASSES = "bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap z-50 pointer-events-none";

const Robot3DViewer: React.FC<Robot3DViewerProps> = ({ 
  jointAngles = [0, 0, 0, 0, 0, 0],
  activeTemplate = null,
  meshVersion = 0,
  pathsVersion = 0
}) => {
  const [isMaximized, setIsMaximized] = useState(false);
  const [isMeshVisible, setIsMeshVisible] = useState(true);
  const [isWireframe, setIsWireframe] = useState(false);
  const [isPathsVisible, setIsPathsVisible] = useState(true);
  const [meshVertexCount, setMeshVertexCount] = useState<number>(0);
  const [pathsCount, setPathsCount] = useState<number>(0);
  const [pointsCount, setPointsCount] = useState<number>(0);

  const containerClasses = isMaximized
    ? "fixed inset-0 z-[100] bg-slate-950/95 backdrop-blur-md p-4 flex flex-col items-center justify-center animate-in fade-in duration-200"
    : "w-full h-full bg-slate-900/50 rounded-xl overflow-hidden relative border border-slate-800 shadow-inner flex flex-col items-center justify-center";

  return (
    <div className={containerClasses}>
      <Canvas shadows camera={{ position: [1.8, 1.4, 1.8], fov: 45 }}>
        <color attach="background" args={['#0f172a']} />

        {/* Scene Lighting */}
        <ambientLight intensity={0.65} />
        <hemisphereLight args={['#e2e8f0', '#0f172a', 0.5]} />
        <directionalLight position={[3, 6, 4]} intensity={0.9} castShadow />
        <directionalLight position={[-4, 2, -3]} intensity={0.35} />

        {/* Robot Model with Base-Aligned Reconstructed Surface Mesh & 3D TCP Trajectories */}
        <RobotModel 
          jointAngles={jointAngles} 
          activeTemplate={activeTemplate}
          meshVersion={meshVersion}
          pathsVersion={pathsVersion}
          isMeshVisible={isMeshVisible}
          isWireframe={isWireframe}
          isPathsVisible={isPathsVisible}
          onMeshLoaded={(count) => setMeshVertexCount(count)}
          onPathsLoaded={(pCount, ptCount) => {
            setPathsCount(pCount);
            setPointsCount(ptCount);
          }}
        />

        {/* Base Floor Grid */}
        <Grid
          infiniteGrid
          fadeDistance={10}
          sectionColor="#64748b"
          cellColor="#334155"
          position={[0, -0.01, 0]}
        />
        <OrbitControls makeDefault target={[0.3, 0.4, 0]} />
      </Canvas>

      {/* Top Left: Compact Model Badges (CR5, Mesh, TCP Info) */}
      <div className="absolute top-2.5 left-2.5 flex items-center gap-1.5 z-10 pointer-events-none">
        <div className="h-6 px-2 rounded-full text-[9px] font-mono text-slate-300 bg-slate-950/70 backdrop-blur-md border border-white/10 shadow-sm flex items-center gap-1.5">
          <span className="w-1.5 h-1.5 rounded-full bg-blue-400"></span>
          <span>CR5</span>
        </div>

        {meshVertexCount > 0 && (
          <div className="h-6 px-2 rounded-full text-[9px] font-mono text-slate-200 bg-slate-900/80 backdrop-blur-md border border-slate-700/50 shadow-sm flex items-center gap-1">
            <Box size={10} className="text-slate-300" />
            <span>Mesh: {(meshVertexCount / 1000).toFixed(1)}k</span>
          </div>
        )}

        {pathsCount > 0 && (
          <div className="h-6 px-2 rounded-full text-[9px] font-mono text-amber-300 bg-amber-950/60 backdrop-blur-md border border-amber-500/40 shadow-sm flex items-center gap-1">
            <Route size={10} className="text-amber-400" />
            <span>TCP: {pathsCount}P ({pointsCount} pts)</span>
          </div>
        )}
      </div>

      {/* Top Right: Compact Controls (Mesh Visibility, Wireframe, TCP Paths, Fullscreen) */}
      <div className="absolute top-2.5 right-2.5 flex items-center gap-1 z-10">
        {/* TCP Paths Toggle Button */}
        {pathsCount > 0 && (
          <div className="relative group flex items-center">
            <button
              onClick={() => setIsPathsVisible(!isPathsVisible)}
              className={`h-6 px-2 rounded-full text-[9px] font-medium border flex items-center gap-1 backdrop-blur-md transition-all shadow-sm ${
                isPathsVisible
                  ? 'bg-amber-950/60 hover:bg-amber-900/70 border-amber-500/50 text-amber-300 shadow-amber-950/30'
                  : 'bg-slate-950/50 hover:bg-slate-900/70 border-white/10 text-slate-400 hover:text-slate-200'
              }`}
            >
              <Route size={11} className={isPathsVisible ? "text-amber-400" : "text-slate-400"} />
              <span>TCP</span>
            </button>
            <div className="absolute top-full mt-1.5 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className={TOOLTIP_CLASSES}>
                {isPathsVisible ? 'Hide 3D TCP Trajectories' : 'Show 3D TCP Trajectories'}
              </div>
            </div>
          </div>
        )}

        {/* Mesh Toggle Button */}
        {meshVertexCount > 0 && (
          <div className="relative group flex items-center">
            <button
              onClick={() => setIsMeshVisible(!isMeshVisible)}
              className={`h-6 px-2 rounded-full text-[9px] font-medium border flex items-center gap-1 backdrop-blur-md transition-all shadow-sm ${
                isMeshVisible
                  ? 'bg-slate-800/90 hover:bg-slate-700/90 border-slate-500/50 text-slate-200 shadow-slate-950/30'
                  : 'bg-slate-950/50 hover:bg-slate-900/70 border-white/10 text-slate-400 hover:text-slate-200'
              }`}
            >
              {isMeshVisible ? <Eye size={11} className="text-slate-300" /> : <EyeOff size={11} className="text-slate-400" />}
              <span>Mesh</span>
            </button>
            <div className="absolute top-full mt-1.5 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className={TOOLTIP_CLASSES}>
                {isMeshVisible ? 'Hide Reconstructed Surface Mesh' : 'Show Reconstructed Surface Mesh'}
              </div>
            </div>
          </div>
        )}

        {/* Wireframe Toggle Button */}
        {meshVertexCount > 0 && isMeshVisible && (
          <div className="relative group flex items-center">
            <button
              onClick={() => setIsWireframe(!isWireframe)}
              className={`h-6 px-2 rounded-full text-[9px] font-medium border flex items-center gap-1 backdrop-blur-md transition-all shadow-sm ${
                isWireframe
                  ? 'bg-cyan-950/60 hover:bg-cyan-900/70 border-cyan-500/40 text-cyan-300 shadow-cyan-950/30'
                  : 'bg-slate-950/50 hover:bg-slate-900/70 border-white/10 text-slate-400 hover:text-slate-200'
              }`}
            >
              <Grid3X3 size={11} className={isWireframe ? 'text-cyan-400' : 'text-slate-400'} />
              <span>Wire</span>
            </button>
            <div className="absolute top-full mt-1.5 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
              <div className={TOOLTIP_CLASSES}>
                {isWireframe ? 'Shaded View' : 'Wireframe View'}
              </div>
            </div>
          </div>
        )}

        {/* Fullscreen Button */}
        <div className="relative group flex items-center">
          <button
            onClick={() => setIsMaximized(!isMaximized)}
            className="h-6 w-6 rounded-full bg-slate-950/50 hover:bg-slate-900/70 text-slate-400 hover:text-white border border-white/10 backdrop-blur-md shadow-sm transition-all flex items-center justify-center"
          >
            {isMaximized ? <Minimize2 size={11} /> : <Maximize2 size={11} />}
          </button>
          <div className="absolute top-full mt-1.5 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
            <div className={TOOLTIP_CLASSES}>
              {isMaximized ? 'Exit Fullscreen' : 'Fullscreen 3D Viewer'}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Robot3DViewer;
