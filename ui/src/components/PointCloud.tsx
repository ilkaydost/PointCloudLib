'use client';

import { useEffect, useMemo, useRef } from 'react';
import { BufferGeometry, BufferAttribute } from 'three';

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
    console.log('Normalized colors - first 5:', normalized.slice(0, 15));
    return normalized;
  }, [colors, positions.length]);

  // Force geometry update when data changes
  useEffect(() => {
    if (geometryRef.current) {
      const posAttr = new BufferAttribute(positions, 3);
      const colorAttr = new BufferAttribute(normalizedColors, 3);
      geometryRef.current.setAttribute('position', posAttr);
      geometryRef.current.setAttribute('color', colorAttr);
      geometryRef.current.attributes.position.needsUpdate = true;
      geometryRef.current.attributes.color.needsUpdate = true;
      geometryRef.current.computeBoundingSphere();
    }
  }, [positions, normalizedColors]);

  return (
    <points>
      <bufferGeometry ref={geometryRef} />
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
