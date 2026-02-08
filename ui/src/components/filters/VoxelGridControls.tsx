'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';
import styles from './css/VoxelGridControls.module.css';

export function VoxelGridControls() {
  const applyVoxelGridFilter = usePointCloudStore((state) => state.applyVoxelGridFilter);
  const [leafSize, setLeafSize] = useState(0.01);
  const [isLoading, setIsLoading] = useState(false);

  const handleApply = async () => {
    setIsLoading(true);
    try {
      await applyVoxelGridFilter(leafSize);
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    setLeafSize(0.01);
  };

  return (
    <div className={styles.container}>
      <h4 className={styles.title}>
        VoxelGrid Filter Settings
      </h4>

      {/* Leaf Size */}
      <div className={styles.fieldGroup}>
        <label className={styles.label}>
          Leaf Size (voxel dimension)
        </label>
        <input
          type="number"
          step="0.001"
          min="0.001"
          value={leafSize}
          onChange={(e) => setLeafSize(parseFloat(e.target.value))}
          className={styles.input}
          placeholder="Leaf size"
        />
        <p className={styles.hint}>
          Smaller values = more points (less downsampling)
        </p>
      </div>

      {/* Actions */}
      <div className={styles.actions}>
        <button
          onClick={handleApply}
          disabled={isLoading}
          className={styles.applyButton}
        >
          {isLoading ? 'Applying...' : 'Apply'}
        </button>
        <button
          onClick={handleReset}
          disabled={isLoading}
          className={styles.resetButton}
        >
          Reset
        </button>
      </div>
      {/* Info */}
      <p className={styles.info}>
        VoxelGrid filter downsamples the point cloud by creating a 3D voxel grid and replacing points within each voxel with their centroid.
      </p>
    </div>
  );
}
