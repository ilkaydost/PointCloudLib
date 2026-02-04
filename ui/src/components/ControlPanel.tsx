'use client';

import { useState } from 'react';
import { PassThroughControls } from './filters/PassThroughControls';
import { VoxelGridControls } from './filters/VoxelGridControls';
import { RansacControls } from './segmentation/RansacControls';
import { usePointCloudStore } from '../store/pointCloudStore';

export function ControlPanel() {
  const [activeFilter, setActiveFilter] = useState<'passthrough' | 'voxelgrid' | 'ransac' | null>(null);
  const stats = usePointCloudStore((state) => state.stats);

  return (
    <div className="bg-white dark:bg-gray-800 rounded-lg shadow-lg p-6 space-y-6">
      <div>
        <h2 className="text-lg font-semibold text-gray-900 dark:text-white mb-4">
          Point Cloud Controls
        </h2>
        
        {/* Stats */}
        {stats && (
          <div className="bg-gray-50 dark:bg-gray-700 rounded-md p-4 mb-4 space-y-2 text-sm">
            <div className="flex justify-between">
              <span className="text-gray-600 dark:text-gray-300">Points:</span>
              <span className="font-mono text-gray-900 dark:text-white">
                {stats.pointCount.toLocaleString()}
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600 dark:text-gray-300">Bounds:</span>
              <span className="font-mono text-xs text-gray-900 dark:text-white">
                X: [{stats.bounds.minX.toFixed(2)}, {stats.bounds.maxX.toFixed(2)}]
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600 dark:text-gray-300"></span>
              <span className="font-mono text-xs text-gray-900 dark:text-white">
                Y: [{stats.bounds.minY.toFixed(2)}, {stats.bounds.maxY.toFixed(2)}]
              </span>
            </div>
            <div className="flex justify-between">
              <span className="text-gray-600 dark:text-gray-300"></span>
              <span className="font-mono text-xs text-gray-900 dark:text-white">
                Z: [{stats.bounds.minZ.toFixed(2)}, {stats.bounds.maxZ.toFixed(2)}]
              </span>
            </div>
          </div>
        )}
      </div>

      {/* Filter Selection */}
      <div>
        <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-3">
          Filters
        </h3>
        <div className="space-y-2">
          <button
            onClick={() => setActiveFilter(activeFilter === 'passthrough' ? null : 'passthrough')}
            className={`w-full px-4 py-2 text-left rounded-md transition-colors ${
              activeFilter === 'passthrough'
                ? 'bg-blue-100 dark:bg-blue-900 text-blue-900 dark:text-blue-100'
                : 'bg-gray-100 dark:bg-gray-700 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'
            }`}
          >
            PassThrough Filter
          </button>
          <button
            onClick={() => setActiveFilter(activeFilter === 'voxelgrid' ? null : 'voxelgrid')}
            className={`w-full px-4 py-2 text-left rounded-md transition-colors ${
              activeFilter === 'voxelgrid'
                ? 'bg-blue-100 dark:bg-blue-900 text-blue-900 dark:text-blue-100'
                : 'bg-gray-100 dark:bg-gray-700 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'
            }`}
          >
            VoxelGrid Filter
          </button>
        </div>
      </div>

      {/* Segmentation Selection */}
      <div>
        <h3 className="text-sm font-medium text-gray-700 dark:text-gray-300 mb-3">
          Segmentation
        </h3>
        <div className="space-y-2">
          <button
            onClick={() => setActiveFilter(activeFilter === 'ransac' ? null : 'ransac')}
            className={`w-full px-4 py-2 text-left rounded-md transition-colors ${
              activeFilter === 'ransac'
                ? 'bg-green-100 dark:bg-green-900 text-green-900 dark:text-green-100'
                : 'bg-gray-100 dark:bg-gray-700 text-gray-700 dark:text-gray-300 hover:bg-gray-200 dark:hover:bg-gray-600'
            }`}
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
