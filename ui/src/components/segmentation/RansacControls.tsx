'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';
import styles from './css/RansacControls.module.css';

export function RansacControls() {
  const [distanceThreshold, setDistanceThreshold] = useState(0.01);
  const [maxIterations, setMaxIterations] = useState(1000);
  const [extractInliers, setExtractInliers] = useState(true);
  const applyRansacSegmentation = usePointCloudStore((state) => state.applyRansacSegmentation);
  const isLoading = usePointCloudStore((state) => state.isLoading);

  const handleApply = () => {
    applyRansacSegmentation(distanceThreshold, maxIterations, extractInliers);
  };

  return (
    <div className={styles.container}>
      <h4 className={styles.title}>
        RANSAC Plane Segmentation
      </h4>

      {/* Distance Threshold */}
      <div className={styles.fieldGroup}>
        <label className={styles.label}>
          Distance Threshold (m): {distanceThreshold.toFixed(3)}
        </label>
        <input
          type="range"
          min="0.001"
          max="0.1"
          step="0.001"
          value={distanceThreshold}
          onChange={(e) => setDistanceThreshold(parseFloat(e.target.value))}
          className={styles.rangeInput}
        />
        <div className={styles.rangeLabels}>
          <span>0.001</span>
          <span>0.1</span>
        </div>
      </div>

      {/* Max Iterations */}
      <div className={styles.fieldGroup}>
        <label className={styles.label}>
          Max Iterations: {maxIterations}
        </label>
        <input
          type="range"
          min="100"
          max="5000"
          step="100"
          value={maxIterations}
          onChange={(e) => setMaxIterations(parseInt(e.target.value))}
          className={styles.rangeInput}
        />
        <div className={styles.rangeLabels}>
          <span>100</span>
          <span>5000</span>
        </div>
      </div>

      {/* Extract Inliers Toggle */}
      <div className={styles.toggleContainer}>
        <label className={styles.toggleLabel}>
          Extract: {extractInliers ? 'Plane (inliers)' : 'Remaining (outliers)'}
        </label>
        <button
          onClick={() => setExtractInliers(!extractInliers)}
          className={`${styles.toggleButton} ${extractInliers ? styles.toggleButtonOn : styles.toggleButtonOff}`}
        >
          <span
            className={`${styles.toggleThumb} ${extractInliers ? styles.toggleThumbOn : styles.toggleThumbOff}`}
          />
        </button>
      </div>

      {/* Apply Button */}
      <button
        onClick={handleApply}
        disabled={isLoading}
        className={styles.applyButton}
      >
        {isLoading ? 'Processing...' : 'Apply RANSAC'}
      </button>

      {/* Info */}
      <p className={styles.info}>
        RANSAC finds the dominant plane in the point cloud. Toggle to extract either the plane or the remaining points.
      </p>
    </div>
  );
}
