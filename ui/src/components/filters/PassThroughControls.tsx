'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';
import styles from './css/PassThroughControls.module.css';

export function PassThroughControls() {
  const stats = usePointCloudStore((state) => state.stats);
  const applyPassThroughFilter = usePointCloudStore((state) => state.applyPassThroughFilter);

  const [minX, setMinX] = useState(stats?.bounds.minX ?? -1);
  const [maxX, setMaxX] = useState(stats?.bounds.maxX ?? 1);
  const [minY, setMinY] = useState(stats?.bounds.minY ?? -1);
  const [maxY, setMaxY] = useState(stats?.bounds.maxY ?? 1);
  const [minZ, setMinZ] = useState(stats?.bounds.minZ ?? -1);
  const [maxZ, setMaxZ] = useState(stats?.bounds.maxZ ?? 1);
  const [isLoading, setIsLoading] = useState(false);

  const handleApply = async () => {
    setIsLoading(true);
    try {
      await Promise.all([
        applyPassThroughFilter('x', minX, maxX),
        applyPassThroughFilter('y', minY, maxY),
        applyPassThroughFilter('z', minZ, maxZ),
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleReset = () => {
    if (stats) {
      setMinX(stats.bounds.minX);
      setMaxX(stats.bounds.maxX);
      setMinY(stats.bounds.minY);
      setMaxY(stats.bounds.maxY);
      setMinZ(stats.bounds.minZ);
      setMaxZ(stats.bounds.maxZ);
    }
  };

  return (
    <div className={styles.container}>
      <h4 className={styles.title}>
        PassThrough Filter Settings
      </h4>

      {/* X Axis */}
      <div className={styles.fieldGroup}>
        <label className={styles.label}>
          X Range
        </label>
        <div className={styles.inputGrid}>
          <input
            type="number"
            step="0.1"
            value={minX}
            onChange={(e) => setMinX(parseFloat(e.target.value))}
            className={styles.input}
            placeholder="Min X"
          />
          <input
            type="number"
            step="0.1"
            value={maxX}
            onChange={(e) => setMaxX(parseFloat(e.target.value))}
            className={styles.input}
            placeholder="Max X"
          />
        </div>
      </div>

      {/* Y Axis */}
      <div className={styles.fieldGroup}>
        <label className={styles.label}>
          Y Range
        </label>
        <div className={styles.inputGrid}>
          <input
            type="number"
            step="0.1"
            value={minY}
            onChange={(e) => setMinY(parseFloat(e.target.value))}
            className={styles.input}
            placeholder="Min Y"
          />
          <input
            type="number"
            step="0.1"
            value={maxY}
            onChange={(e) => setMaxY(parseFloat(e.target.value))}
            className={styles.input}
            placeholder="Max Y"
          />
        </div>
      </div>

      {/* Z Axis */}
      <div className={styles.fieldGroup}>
        <label className={styles.label}>
          Z Range
        </label>
        <div className={styles.inputGrid}>
          <input
            type="number"
            step="0.1"
            value={minZ}
            onChange={(e) => setMinZ(parseFloat(e.target.value))}
            className={styles.input}
            placeholder="Min Z"
          />
          <input
            type="number"
            step="0.1"
            value={maxZ}
            onChange={(e) => setMaxZ(parseFloat(e.target.value))}
            className={styles.input}
            placeholder="Max Z"
          />
        </div>
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
    </div>
  );
}
