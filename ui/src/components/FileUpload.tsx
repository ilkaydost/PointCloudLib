'use client';

import { useState, useRef } from 'react';
import { usePointCloudStore } from '../store/pointCloudStore';
import styles from './css/FileUpload.module.css';

export function FileUpload() {
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const uploadPointCloud = usePointCloudStore((state) => state.uploadPointCloud);

  const handleFileSelect = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    setIsLoading(true);
    setError(null);

    try {
      await uploadPointCloud(file);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load point cloud');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.container}>
      <label className={styles.label}>
        Load Point Cloud
      </label>
      <div className={styles.inputWrapper}>
        <input
          ref={fileInputRef}
          type="file"
          accept=".pcd,.ply"
          onChange={handleFileSelect}
          disabled={isLoading}
          className={styles.fileInput}
        />
        {isLoading && (
          <div className={styles.spinner}></div>
        )}
      </div>
      {error && (
        <p className={styles.error}>{error}</p>
      )}
    </div>
  );
}
