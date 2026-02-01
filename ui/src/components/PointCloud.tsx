'use client';

import { useMemo, useRef } from 'react';
import { BufferGeometry } from 'three';

interface PointCloudProps {
  positions: Float32Array;
  colors?: Uint8Array | null;
}

export function PointCloud({ positions, colors }: PointCloudProps) {
  const geometryRef = useRef<BufferGeometry>(null);

  const normalizedColors = useMemo(() => {
    if (!colors) {
      // Default white if no colors provided
      const defaultColors = new Float32Array(positions.length);
      defaultColors.fill(1);
      return defaultColors;
    }
    // Convert from Uint8Array (0-255) to Float32Array (0-1)
    const normalized = new Float32Array(colors.length);
    for (let i = 0; i < colors.length; i++) {
      normalized[i] = colors[i] / 255;
    }
    return normalized;
  }, [colors, positions]);

  return (
    <points>
      <bufferGeometry ref={geometryRef}>
        <bufferAttribute
          attach="attributes-position"
          count={positions.length / 3}
          array={positions}
          itemSize={3}
        />
        <bufferAttribute
          attach="attributes-color"
          count={normalizedColors.length / 3}
          array={normalizedColors}
          itemSize={3}
        />
      </bufferGeometry>
      <pointsMaterial
        size={0.02}
        vertexColors
        sizeAttenuation
        transparent
        opacity={0.8}
      />
    </points>
  );
}
