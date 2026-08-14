import React, { useEffect, useState, useRef } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import URDFLoader from 'urdf-loader';
import { Maximize2, Minimize2, Eye, EyeOff, Box, Grid3X3 } from 'lucide-react';
import {
  Object3D,
  LoadingManager,
  Light,
  Mesh,
  Material,
  MeshPhongMaterial,
  MeshStandardMaterial,
  Color,
  DoubleSide,
  BufferGeometry,
} from 'three';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';

interface RobotModelProps {
  jointAngles: number[];
  activeTemplate: string | null;
  meshVersion: number;
  isMeshVisible: boolean;
  isWireframe: boolean;
  onMeshLoaded?: (vertexCount: number) => void;
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
  material: Material,
  done: (obj: Object3D, err?: Error) => void,
) {
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
  isMeshVisible,
  isWireframe,
  onMeshLoaded
}) => {
  const [robot, setRobot] = useState<Object3D | null>(null);
  const surfaceMeshRef = useRef<Mesh | null>(null);
  const surfaceMaterialRef = useRef<MeshStandardMaterial | null>(null);

  // 1. Load Robot URDF Model
  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);

    loader.packages = {
      dobot_rviz: 'http://localhost:8000/urdf',
      dobot_gazebo_sim: 'http://localhost:8000/urdf',
    };
    loader.loadMeshCb = loadUrdfMesh;

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

  // 2. Update live joint angles
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
        color: new Color('#ffffff'), // Pure bright silver-white
        roughness: 0.35,
        metalness: 0.20,
        wireframe: isWireframe,
        side: DoubleSide,
        transparent: true,
        opacity: 0.92,
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

      // Because vertices in scan.mesh.ply are in robot base_link frame (meters),
      // attaching directly to base_link automatically aligns height & rotation with the robot arm!
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
        // Fallback to STL loader
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

  // 4. Handle Visibility and Wireframe Changes
  useEffect(() => {
    if (surfaceMeshRef.current) {
      surfaceMeshRef.current.visible = isMeshVisible;
    }
  }, [isMeshVisible]);

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
}

const TOOLTIP_CLASSES = "bg-slate-950/90 backdrop-blur-md text-slate-200 text-[10px] font-medium px-2.5 py-1 rounded-md shadow-2xl border border-white/10 whitespace-nowrap z-50 pointer-events-none";

const Robot3DViewer: React.FC<Robot3DViewerProps> = ({ 
  jointAngles = [0, 0, 0, 0, 0, 0],
  activeTemplate = null,
  meshVersion = 0
}) => {
  const [isMaximized, setIsMaximized] = useState(false);
  const [isMeshVisible, setIsMeshVisible] = useState(true);
  const [isWireframe, setIsWireframe] = useState(false);
  const [meshVertexCount, setMeshVertexCount] = useState<number>(0);

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

        {/* Robot Model with Base-Aligned Reconstructed Surface Mesh */}
        <RobotModel 
          jointAngles={jointAngles} 
          activeTemplate={activeTemplate}
          meshVersion={meshVersion}
          isMeshVisible={isMeshVisible}
          isWireframe={isWireframe}
          onMeshLoaded={(count) => setMeshVertexCount(count)}
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

      {/* Top Left: Digital Twin Title & Reconstructed Mesh Badge */}
      <div className="absolute top-3 left-3 flex items-center gap-2 z-10">
        <div className="h-7 px-2.5 rounded-full text-[10px] font-mono text-slate-300 bg-slate-950/60 backdrop-blur-md border border-white/10 shadow-md flex items-center gap-1.5">
          <span className="w-1.5 h-1.5 rounded-full bg-blue-400"></span>
          <span>DOBOT CR5 Digital Twin</span>
        </div>

        {meshVertexCount > 0 && (
          <div className="h-7 px-2.5 rounded-full text-[10px] font-mono text-slate-200 bg-slate-800/70 backdrop-blur-md border border-slate-600/40 shadow-md flex items-center gap-1.5">
            <Box size={12} className="text-slate-300" />
            <span>Mesh: {(meshVertexCount / 1000).toFixed(1)}k Verts</span>
          </div>
        )}
      </div>

      {/* Top Right: Controls (Mesh Visibility, Wireframe, Fullscreen) */}
      <div className="absolute top-3 right-3 flex items-center gap-1.5 z-10">
        {/* Mesh Toggle Button */}
        {meshVertexCount > 0 && (
          <div className="relative group flex items-center">
            <button
              onClick={() => setIsMeshVisible(!isMeshVisible)}
              className={`h-7 px-2.5 rounded-full text-[10px] font-medium border flex items-center gap-1.5 backdrop-blur-md transition-all shadow-md ${
                isMeshVisible
                  ? 'bg-slate-800/90 hover:bg-slate-700/90 border-slate-500/50 text-slate-200 shadow-slate-950/30'
                  : 'bg-slate-950/40 hover:bg-slate-900/60 border-white/10 text-slate-400 hover:text-slate-200'
              }`}
            >
              {isMeshVisible ? <Eye size={12} className="text-slate-300" /> : <EyeOff size={12} className="text-slate-400" />}
              <span>Surface Mesh</span>
            </button>
            <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
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
              className={`h-7 px-2.5 rounded-full text-[10px] font-medium border flex items-center gap-1.5 backdrop-blur-md transition-all shadow-md ${
                isWireframe
                  ? 'bg-cyan-950/50 hover:bg-cyan-900/70 border-cyan-500/40 text-cyan-300 shadow-cyan-950/30'
                  : 'bg-slate-950/40 hover:bg-slate-900/60 border-white/10 text-slate-400 hover:text-slate-200'
              }`}
            >
              <Grid3X3 size={12} className={isWireframe ? 'text-cyan-400' : 'text-slate-400'} />
              <span>Wireframe</span>
            </button>
            <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
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
            className="h-7 w-7 rounded-full bg-slate-950/40 hover:bg-slate-900/60 text-slate-400 hover:text-white border border-white/10 backdrop-blur-md shadow-md transition-all flex items-center justify-center"
          >
            {isMaximized ? <Minimize2 size={13} /> : <Maximize2 size={13} />}
          </button>
          <div className="absolute top-full mt-2 right-0 hidden group-hover:flex flex-col items-center pointer-events-none z-50">
            <div className={TOOLTIP_CLASSES}>
              {isMaximized ? 'Exit Fullscreen' : 'Fullscreen 3D Digital Twin'}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Robot3DViewer;
