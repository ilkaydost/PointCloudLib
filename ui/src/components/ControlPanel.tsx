'use client';

import { useState } from 'react';
import { PassThroughControls } from './filters/PassThroughControls';
import { VoxelGridControls } from './filters/VoxelGridControls';
import { RansacControls } from './segmentation/RansacControls';
import { usePointCloudStore } from '../store/pointCloudStore';
import styles from './css/ControlPanel.module.css';

export function ControlPanel() {
  const [activeFilter, setActiveFilter] = useState<'passthrough' | 'voxelgrid' | 'ransac' | null>(null);
  const stats = usePointCloudStore((state) => state.stats);

  return (
    <div className={styles.container}>
      <div className={styles.section}>
        <h2 className={styles.title}>
          Point Cloud Controls
        </h2>
        
        {/* Stats */}
        {stats && (
          <div className={styles.statsContainer}>
            <div className={styles.statRow}>
              <span className={styles.statLabel}>Points:</span>
              <span className={styles.statValue}>
                {stats.pointCount.toLocaleString()}
              </span>
            </div>
            <div className={styles.statRow}>
              <span className={styles.statLabel}>Bounds:</span>
              <span className={styles.statValueSmall}>
                X: [{stats.bounds.minX.toFixed(2)}, {stats.bounds.maxX.toFixed(2)}]
              </span>
            </div>
            <div className={styles.statRow}>
              <span className={styles.statLabel}></span>
              <span className={styles.statValueSmall}>
                Y: [{stats.bounds.minY.toFixed(2)}, {stats.bounds.maxY.toFixed(2)}]
              </span>
            </div>
            <div className={styles.statRow}>
              <span className={styles.statLabel}></span>
              <span className={styles.statValueSmall}>
                Z: [{stats.bounds.minZ.toFixed(2)}, {stats.bounds.maxZ.toFixed(2)}]
              </span>
            </div>
          </div>
        )}
      </div>

      {/* Filter Selection */}
      <div className={styles.section}>
        <h3 className={styles.sectionTitle}>
          Filters
        </h3>
        <div className={styles.buttonList}>
          <button
            onClick={() => setActiveFilter(activeFilter === 'passthrough' ? null : 'passthrough')}
            className={activeFilter === 'passthrough' 
              ? `${styles.filterButton} ${styles.filterButtonActive}` 
              : styles.filterButton}
          >
            PassThrough Filter
          </button>
          <button
            onClick={() => setActiveFilter(activeFilter === 'voxelgrid' ? null : 'voxelgrid')}
            className={activeFilter === 'voxelgrid' 
              ? `${styles.filterButton} ${styles.filterButtonActive}` 
              : styles.filterButton}
          >
            VoxelGrid Filter
          </button>
        </div>
      </div>

      {/* Segmentation Selection */}
      <div className={styles.section}>
        <h3 className={styles.sectionTitle}>
          Segmentation
        </h3>
        <div className={styles.buttonList}>
          <button
            onClick={() => setActiveFilter(activeFilter === 'ransac' ? null : 'ransac')}
            className={activeFilter === 'ransac' 
              ? `${styles.segmentButton} ${styles.segmentButtonActive}` 
              : styles.segmentButton}
          >
            RANSAC Plane
          </button>
        </div>
      </div>

      {/* Filter Controls */}
      {activeFilter === 'passthrough' && <PassThroughControls />}
      {activeFilter === 'voxelgrid' && <VoxelGridControls />}
      {activeFilter === 'ransac' && <RansacControls />}
    </div>
  );
}
