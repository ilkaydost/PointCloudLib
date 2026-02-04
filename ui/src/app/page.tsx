'use client';

import { useEffect } from 'react';
import { Layout } from '@/components/Layout';
import { PointCloudViewer } from '@/components/PointCloudViewer';
import { FileUpload } from '@/components/FileUpload';
import { ControlPanel } from '@/components/ControlPanel';
import { usePointCloudStore } from '@/store/pointCloudStore';
import styles from './css/page.module.css';

export default function Home() {
  const serverConnected = usePointCloudStore((state) => state.serverConnected);
  const checkServerHealth = usePointCloudStore((state) => state.checkServerHealth);
  const error = usePointCloudStore((state) => state.error);

  // Check server health on mount and every 5 seconds
  useEffect(() => {
    checkServerHealth();
    const interval = setInterval(checkServerHealth, 5000);
    return () => clearInterval(interval);
  }, [checkServerHealth]);

  const sidebar = (
    <div className={styles.sidebarContainer}>
      <div className={styles.header}>
        <h1 className={styles.title}>
          PointCloudLib
        </h1>
        <p className={styles.subtitle}>
          Point Cloud Viewer
        </p>

        {/* Server Status */}
        <div className={styles.statusContainer}>
          <div
            className={`${styles.statusDot} ${
              serverConnected ? styles.statusConnected : styles.statusDisconnected
            }`}
          />
          <span className={styles.statusText}>
            Server: {serverConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>

        {/* Error Display */}
        {error && (
          <div className={styles.errorContainer}>
            {error}
          </div>
        )}

        {/* File Upload */}
        <FileUpload />
      </div>

      {/* Control Panel */}
      <ControlPanel />
    </div>
  );

  return (
    <Layout sidebar={sidebar}>
      <div className={styles.viewerContainer}>
        <PointCloudViewer />
      </div>
    </Layout>
  );
}
