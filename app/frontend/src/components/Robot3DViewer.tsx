import React, { useEffect, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import URDFLoader from 'urdf-loader';
import { Object3D, LoadingManager } from 'three';

interface RobotModelProps {
  jointAngles: number[];
}

// Joint names in CR5 URDF matching the order J1..J6
const JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

const RobotModel: React.FC<RobotModelProps> = ({ jointAngles }) => {
  const [robot, setRobot] = useState<Object3D | null>(null);

  useEffect(() => {
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    
    loader.packages = {
        'dobot_rviz': 'http://localhost:8000/urdf',
        'dobot_gazebo_sim': 'http://localhost:8000/urdf'
    };

    loader.load(`http://localhost:8000/urdf/cr5_robot.urdf`, (r: any) => {
      r.rotation.x = -Math.PI / 2;
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

const Robot3DViewer: React.FC<Robot3DViewerProps> = ({ jointAngles = [0,0,0,0,0,0] }) => {
  return (
    <div className="w-full h-full bg-slate-900/50 rounded-xl overflow-hidden relative border border-slate-800 shadow-inner flex flex-col items-center justify-center">
      <Canvas shadows camera={{ position: [2, 1.5, 2], fov: 45 }}>
        <color attach="background" args={['#1e293b']} />
        
        {/* Dim, flat lighting to match RViz appearance */}
        <ambientLight intensity={0.6} />
        <directionalLight position={[5, 5, 5]} intensity={0.2} />
        
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
