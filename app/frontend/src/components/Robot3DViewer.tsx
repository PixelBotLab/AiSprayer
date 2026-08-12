import React, { useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import URDFLoader from 'urdf-loader';
import {
  Object3D,
  LoadingManager,
  Light,
  Mesh,
  Material,
  MeshPhongMaterial,
  Color,
  DoubleSide,
} from 'three';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

interface RobotModelProps {
  jointAngles: number[];
}

// Joint names in CR5 URDF matching the order J1..J6
const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

/**
 * CR5 的 .dae 由 Blender 导出，场景里嵌了 color=1000,1000,1000 的点光源。
 * three@0.185 的 ColladaLoader 会把这些灯一起挂进场景，整机被照成高亮白，
 * 连杆自带的黑/白/粉材质反而看不见。加载后必须剥掉 DAE 内嵌灯光。
 */
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
        stripEmbeddedLights(dae.scene);
        done(dae.scene);
      },
      undefined,
      (err) => done(new Object3D(), err as Error),
    );
    return;
  }

  console.warn(`URDFLoader: no mesh loader for ${path}`);
  done(new Object3D(), new Error(`Unsupported mesh type: ${path}`));
}

const RobotModel: React.FC<RobotModelProps> = ({ jointAngles }) => {
  const [robot, setRobot] = useState<Object3D | null>(null);

  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);

    loader.packages = {
      dobot_rviz: 'http://localhost:8000/urdf',
      dobot_gazebo_sim: 'http://localhost:8000/urdf',
    };
    loader.loadMeshCb = loadUrdfMesh;

    loader.load(`http://localhost:8000/urdf/cr5_robot.urdf?v=${Date.now()}`, (r: any) => {
      // 兜底再扫一遍整机，防止个别 mesh 漏网
      stripEmbeddedLights(r);
      r.rotation.x = -Math.PI / 2;
      // 确保材质接收场景灯光（部分 Collada 材质默认 flat / 无 specular 时偏灰白）
      r.traverse((obj: Object3D) => {
        const mesh = obj as Mesh;
        if (!mesh.isMesh) return;
        const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
        mats.forEach((m: any) => {
          if (!m) return;
          m.side = DoubleSide;
          m.needsUpdate = true;
          // 纯黑 diffuse 在弱光下几乎看不见，略提一点以免“丢色”观感
          if (m.color && m.color instanceof Color) {
            const { r: cr, g: cg, b: cb } = m.color;
            if (cr + cg + cb < 0.05) m.color.setRGB(0.08, 0.08, 0.08);
          }
        });
      });
      setRobot(r);
    });
  }, []);

  // Update joint angles whenever they change
  useEffect(() => {
    if (!robot) return;
    const r = robot as any;
    JOINT_NAMES.forEach((name, idx) => {
      if (r.setJointValue) {
        // URDF loader expects joint values in radians, but our backend sends degrees
        r.setJointValue(name, (jointAngles[idx] ?? 0) * (Math.PI / 180));
      }
    });
  }, [robot, jointAngles]);

  if (!robot) return null;
  return <primitive object={robot} />;
};

interface Robot3DViewerProps {
  jointAngles?: number[];
}

const Robot3DViewer: React.FC<Robot3DViewerProps> = ({ jointAngles = [0, 0, 0, 0, 0, 0] }) => {
  return (
    <div className="w-full h-full bg-slate-900/50 rounded-xl overflow-hidden relative border border-slate-800 shadow-inner flex flex-col items-center justify-center">
      <Canvas shadows camera={{ position: [2, 1.5, 2], fov: 45 }}>
        <color attach="background" args={['#1e293b']} />

        {/* 场景自管灯光；不要依赖 DAE 内嵌灯 */}
        <ambientLight intensity={0.55} />
        <hemisphereLight args={['#e2e8f0', '#1e293b', 0.45]} />
        <directionalLight position={[3, 6, 4]} intensity={0.85} castShadow />
        <directionalLight position={[-4, 2, -3]} intensity={0.25} />

        <RobotModel jointAngles={jointAngles} />

        <Grid
          infiniteGrid
          fadeDistance={10}
          sectionColor="#94a3b8"
          cellColor="#475569"
          position={[0, -0.01, 0]}
        />
        <OrbitControls makeDefault />
      </Canvas>
      <div className="absolute top-3 left-3 text-[10px] font-mono text-slate-400 bg-slate-950/80 px-2 py-1 rounded shadow-md pointer-events-none">
        DOBOT CR5 Digital Twin
      </div>
    </div>
  );
};

export default Robot3DViewer;
