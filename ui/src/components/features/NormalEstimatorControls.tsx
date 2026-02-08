'use client';

import { useState } from 'react';
import { usePointCloudStore } from '@/store/pointCloudStore';
import styles from './css/NormalEstimatorControls.module.css';

export function NormalEstimatorControls() {
  const [kSearch, setKSearch] = useState(30);
  const [radiusSearch, setRadiusSearch] = useState(0.0);
  const applyNormalEstimation = usePointCloudStore((state) => state.applyNormalEstimation);
  const isLoading = usePointCloudStore((state) => state.isLoading);
  const fetchNormals = usePointCloudStore((state) => state.fetchNormals);
  const showNormals = usePointCloudStore((state) => state.showNormals);
  const setShowNormals = usePointCloudStore((state) => state.setShowNormals);

  const handleApply = () => {
    const config: any = {};
    if (kSearch > 0) config.kSearch = kSearch;
    if (radiusSearch > 0) config.radiusSearch = radiusSearch;
    applyNormalEstimation(config).then(() => {
      // fetch normals for visualization and enable display
      fetchNormals(config.kSearch, config.radiusSearch).catch(() => {});
      setShowNormals(true);
    });
  };

  return (
    <div className={styles.container}>
      <h4 className={styles.title}>Normal Estimation</h4>

      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="k-search">K Search: {kSearch}</label>
        <input
          id="k-search"
          type="range"
          min="1"
          max="200"
          step="1"
          value={kSearch}
          onChange={(e) => setKSearch(Number.parseInt(e.target.value))}
          className={styles.rangeInput}
          aria-label="K nearest neighbours for normal estimation"
        />
        <div className={styles.rangeLabels}><span>1</span><span>200</span></div>
      </div>

      <div className={styles.fieldGroup}>
        <label className={styles.label} htmlFor="radius-search">Radius Search (m): {radiusSearch.toFixed(3)}</label>
        <input
          id="radius-search"
          type="range"
          min="0"
          max="1"
          step="0.001"
          value={radiusSearch}
          onChange={(e) => setRadiusSearch(Number.parseFloat(e.target.value))}
          className={styles.rangeInput}
          aria-label="Radius search for normal estimation (meters)"
        />
        <div className={styles.rangeLabels}><span>0</span><span>1</span></div>
      </div>

      <button onClick={handleApply} 
              disabled={isLoading} 
              className={styles.applyButton}>
        {isLoading ? 'Computing...' : 'Compute Normals'}
      </button>

      <div style={{marginTop: '0.75rem', display: 'flex', alignItems: 'center', gap: '0.5rem'}}>
        <label style={{color:'#b0b0b0', fontSize:'0.875rem'}}>Show Normals</label>
        <input type="checkbox" checked={showNormals} onChange={(e) => setShowNormals(e.target.checked)} />
      </div>
      {/* Info */}
      <p className={styles.info}>
        Normal estimation computes surface normals for each point in the cloud using either K-nearest neighbors or radius search.
      </p>
    </div>
  );
}
