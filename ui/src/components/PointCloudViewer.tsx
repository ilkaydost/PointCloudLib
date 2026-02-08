'use client';

import { useEffect, useRef } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, PerspectiveCamera } from '@react-three/drei';
import { PointCloud } from './PointCloud';
import { usePointCloudStore } from '../store/pointCloudStore';
import * as THREE from 'three';
import styles from './css/PointCloudViewer.module.css';

function NormalsLines({ positions, normals, length = 0.05 }: { positions: Float32Array; normals: Float32Array; length?: number }) {
  // Build line segments: for each point create start(x,y,z) and end(x+nx*L, y+ny*L, z+nz*L)
  const count = positions.length / 3;
  const lineVerts = new Float32Array(count * 2 * 3);
  for (let i = 0; i < count; i++) {
    const px = positions[i * 3 + 0];
    const py = positions[i * 3 + 1];
    const pz = positions[i * 3 + 2];
    const nx = normals[i * 3 + 0];
    const ny = normals[i * 3 + 1];
    const nz = normals[i * 3 + 2];
    // start
    lineVerts[i * 6 + 0] = px;
    lineVerts[i * 6 + 1] = py;
    lineVerts[i * 6 + 2] = pz;
    // end
    lineVerts[i * 6 + 3] = px + nx * length;
    lineVerts[i * 6 + 4] = py + ny * length;
    lineVerts[i * 6 + 5] = pz + nz * length;
  }

  return (
    <lineSegments>
      <bufferGeometry>
        <bufferAttribute attach="attributes-position" array={lineVerts} count={lineVerts.length / 3} itemSize={3} />
      </bufferGeometry>
      <lineBasicMaterial color={0x00ffff} linewidth={1} />
    </lineSegments>
  );
}

// Component to auto-fit camera to point cloud bounds
function CameraController() {
  const { camera } = useThree();
  const stats = usePointCloudStore((state) => state.stats);
  const normals = usePointCloudStore((state) => state.normals);
  const showNormals = usePointCloudStore((state) => state.showNormals);
  const controlsRef = useRef<any>(null);

  useEffect(() => {
    if (stats && stats.bounds) {
      const { minX, maxX, minY, maxY, minZ, maxZ } = stats.bounds;
      
      // Calculate center of the point cloud
      // TO:DO - This currently centers on the bounding box center, which may not be ideal for very sparse point clouds. Consider implementing a more robust method if needed.
      const centerX = (minX + maxX) / 2;
      const centerY = (minY + maxY) / 2;
      const centerZ = (minZ + maxZ) / 2;
      
      // Calculate the size of the bounding box
      const sizeX = maxX - minX;
      const sizeY = maxY - minY;
      const sizeZ = maxZ - minZ;
      const maxSize = Math.max(sizeX, sizeY, sizeZ);
      
      // Position camera to see the entire point cloud
      const distance = maxSize * 1.5;
      camera.position.set(centerX + distance, centerY + distance * 0.5, centerZ + distance);
      camera.lookAt(centerX, centerY, centerZ);
      
      // Update near/far planes based on scene size
      if (camera instanceof THREE.PerspectiveCamera) {
        camera.near = maxSize * 0.001;
        camera.far = maxSize * 100;
        camera.updateProjectionMatrix();
      }
      
      // Update orbit controls target
      if (controlsRef.current) {
        controlsRef.current.target.set(centerX, centerY, centerZ);
        controlsRef.current.update();
      }
    }
  }, [stats, camera]);

  return (
    <OrbitControls
      ref={controlsRef}
      enableDamping
      dampingFactor={0.05}
      enablePan={true}
      enableZoom={true}
      enableRotate={true}
      panSpeed={1}
      zoomSpeed={1.2}
      rotateSpeed={0.8}
      minDistance={0.1}
      maxDistance={10000}
      mouseButtons={{
        LEFT: THREE.MOUSE.ROTATE,
        MIDDLE: THREE.MOUSE.DOLLY,
        RIGHT: THREE.MOUSE.PAN
      }}
    />
  );
}

export function PointCloudViewer() {
  const positions = usePointCloudStore((state) => state.positions);
  const colors = usePointCloudStore((state) => state.colors);
  const stats = usePointCloudStore((state) => state.stats);
  const normals = usePointCloudStore((state) => state.normals);
  const showNormals = usePointCloudStore((state) => state.showNormals);

  // Calculate grid size based on point cloud bounds
  const gridSize = stats?.bounds 
    ? Math.max(
        stats.bounds.maxX - stats.bounds.minX,
        stats.bounds.maxY - stats.bounds.minY,
        stats.bounds.maxZ - stats.bounds.minZ
      ) * 2
    : 10;

  return (
    <div className={styles.container}>
      <Canvas>
        <PerspectiveCamera makeDefault position={[5, 5, 5]} fov={60} />
        <CameraController />
        
        {/* Lighting */}
        <ambientLight intensity={0.6} />
        <directionalLight position={[10, 10, 5]} intensity={0.8} />
        <directionalLight position={[-10, -10, -5]} intensity={0.3} />
        
        {/* Point Cloud */}
        {positions && <PointCloud positions={positions} colors={colors} />}
        {/* Normals visualization */}
        {positions && normals && showNormals && normals.length === positions.length && (
          <NormalsLines positions={positions} normals={normals} length={(stats ? Math.max(stats.bounds.maxX - stats.bounds.minX, stats.bounds.maxY - stats.bounds.minY, stats.bounds.maxZ - stats.bounds.minZ) : 1) * 0.02} />
        )}
        
        {/* Grid helper - scaled to point cloud size */}
        <gridHelper args={[gridSize, 20]} />
        <axesHelper args={[gridSize / 4]} />
      </Canvas>
      
      {/* Controls help overlay */}
      <div className={styles.controlsHelp}>
        <span>Left: Rotate | Right: Pan | Scroll: Zoom</span>
      </div>
    </div>
  );
}
