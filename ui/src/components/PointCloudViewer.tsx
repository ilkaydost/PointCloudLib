'use client';

import { Canvas } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera } from '@react-three/drei';
import { PointCloud } from './PointCloud';
import { usePointCloudStore } from '../store/pointCloudStore';

export function PointCloudViewer() {
  const positions = usePointCloudStore((state) => state.positions);
  const colors = usePointCloudStore((state) => state.colors);

  return (
    <div className="w-full h-full bg-gray-900 rounded-lg overflow-hidden">
      <Canvas>
        <PerspectiveCamera makeDefault position={[0, 0, 5]} />
        <OrbitControls enableDamping dampingFactor={0.05} />
        
        {/* Lighting */}
        <ambientLight intensity={0.5} />
        <directionalLight position={[10, 10, 5]} intensity={1} />
        
        {/* Point Cloud */}
        {positions && <PointCloud positions={positions} colors={colors} />}
        
        {/* Grid helper */}
        <gridHelper args={[10, 10]} />
        <axesHelper args={[2]} />
      </Canvas>
    </div>
  );
}
