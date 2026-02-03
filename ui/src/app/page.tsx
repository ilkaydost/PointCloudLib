'use client';

import { useEffect } from 'react';
import { Layout } from '@/components/Layout';
import { PointCloudViewer } from '@/components/PointCloudViewer';
import { FileUpload } from '@/components/FileUpload';
import { ControlPanel } from '@/components/ControlPanel';
import { usePointCloudStore } from '@/store/pointCloudStore';

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
    <div className="space-y-4 p-4">
      <div className="bg-white dark:bg-gray-800 rounded-lg shadow-lg p-6">
        <h1 className="text-xl font-bold text-gray-900 dark:text-white mb-2">
          PointCloudLib
        </h1>
        <p className="text-sm text-gray-600 dark:text-gray-400 mb-4">
          Point Cloud Viewer
        </p>

        {/* Server Status */}
        <div className="flex items-center gap-2 mb-4">
          <div
            className={`w-2 h-2 rounded-full ${
              serverConnected ? 'bg-green-500' : 'bg-red-500'
            }`}
          />
          <span className="text-sm text-gray-600 dark:text-gray-400">
            Server: {serverConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>

        {/* Error Display */}
        {error && (
          <div className="p-3 bg-red-50 dark:bg-red-900/30 border border-red-200 dark:border-red-700 rounded-lg text-sm text-red-600 dark:text-red-300 mb-4">
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
      <div className="h-full p-4">
        <PointCloudViewer />
      </div>
    </Layout>
  );
}
