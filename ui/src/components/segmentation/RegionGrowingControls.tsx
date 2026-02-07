'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';
import styles from './css/RegionGrowingControls.module.css';

export function RegionGrowingControls() {
  const [smoothnessThreshold, setSmoothnessThreshold] = useState(5.0);
  const [curvatureThreshold, setCurvatureThreshold] = useState(1.0);
  const [minClusterSize, setMinClusterSize] = useState(50);
  const [maxClusterSize, setMaxClusterSize] = useState(1000000);
  const [numberOfNeighbours, setNumberOfNeighbours] = useState(30);
  const [normalKSearch, setNormalKSearch] = useState(30);
  
  const applyRegionGrowing = usePointCloudStore((state) => state.applyRegionGrowing);
  const isLoading = usePointCloudStore((state) => state.isLoading);

  const handleApply = () => {
    applyRegionGrowing({
      smoothnessThreshold,
      curvatureThreshold,
      minClusterSize,
      maxClusterSize,
      numberOfNeighbours,
      normalKSearch,
    });
  };

  return (
    <div className={styles.container}>
      <h4 className={styles.title}>
        Region Growing Segmentation
      </h4>

      {/* Smoothness Threshold */}
      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="smoothness-threshold">
          Smoothness Threshold (°): {smoothnessThreshold.toFixed(1)}
        </label>
        <input
          id="smoothness-threshold"
          type="range"
          min="1"
          max="30"
          step="0.5"
          value={smoothnessThreshold}
          onChange={(e) => setSmoothnessThreshold(parseFloat(e.target.value))}
          className={styles.rangeInput}
          aria-label="Smoothness threshold in degrees"
        />
        <div className={styles.rangeLabels}>
          <span>1°</span>
          <span>30°</span>
        </div>
      </div>

      {/* Curvature Threshold */}
      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="curvature-threshold">
          Curvature Threshold: {curvatureThreshold.toFixed(2)}
        </label>
        <input
          id="curvature-threshold"
          type="range"
          min="0.1"
          max="5.0"
          step="0.1"
          value={curvatureThreshold}
          onChange={(e) => setCurvatureThreshold(parseFloat(e.target.value))}
          className={styles.rangeInput}
          aria-label="Curvature threshold for smooth regions"
        />
        <div className={styles.rangeLabels}>
          <span>0.1</span>
          <span>5.0</span>
        </div>
      </div>

      {/* Min Cluster Size */}
      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="min-cluster-size">
          Min Cluster Size: {minClusterSize}
        </label>
        <input
          id="min-cluster-size"
          type="range"
          min="10"
          max="500"
          step="10"
          value={minClusterSize}
          onChange={(e) => setMinClusterSize(parseInt(e.target.value))}
          className={styles.rangeInput}
          aria-label="Minimum cluster size in points"
        />
        <div className={styles.rangeLabels}>
          <span>10</span>
          <span>500</span>
        </div>
      </div>

      {/* Number of Neighbours */}
      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="num-neighbours">
          Neighbours (KNN): {numberOfNeighbours}
        </label>
        <input
          id="num-neighbours"
          type="range"
          min="10"
          max="100"
          step="5"
          value={numberOfNeighbours}
          onChange={(e) => setNumberOfNeighbours(parseInt(e.target.value))}
          className={styles.rangeInput}
          aria-label="Number of neighbours for KNN search"
        />
        <div className={styles.rangeLabels}>
          <span>10</span>
          <span>100</span>
        </div>
      </div>

      {/* Normal K Search */}
      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="normal-k-search">
          Normal KNN: {normalKSearch}
        </label>
        <input
          id="normal-k-search"
          type="range"
          min="10"
          max="100"
          step="5"
          value={normalKSearch}
          onChange={(e) => setNormalKSearch(parseInt(e.target.value))}
          className={styles.rangeInput}
          aria-label="KNN for normal estimation"
        />
        <div className={styles.rangeLabels}>
          <span>10</span>
          <span>100</span>
        </div>
      </div>

      {/* Apply Button */}
      <button
        onClick={handleApply}
        disabled={isLoading}
        className={`${styles.button} ${isLoading ? styles.buttonDisabled : ''}`}
      >
        {isLoading ? 'Processing...' : 'Apply Region Growing'}
      </button>
    </div>
  );
}
