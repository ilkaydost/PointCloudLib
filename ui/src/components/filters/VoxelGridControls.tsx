'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';

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
    <div className="space-y-4 p-4 bg-gray-50 dark:bg-gray-700 rounded-md">
      <h4 className="text-sm font-medium text-gray-900 dark:text-white">
        VoxelGrid Filter Settings
      </h4>

      {/* Leaf Size */}
      <div className="space-y-2">
        <label className="text-xs font-medium text-gray-700 dark:text-gray-300">
          Leaf Size (voxel dimension)
        </label>
        <input
          type="number"
          step="0.001"
          min="0.001"
          value={leafSize}
          onChange={(e) => setLeafSize(parseFloat(e.target.value))}
          className="w-full px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
          placeholder="Leaf size"
        />
        <p className="text-xs text-gray-500 dark:text-gray-400">
          Smaller values = more points (less downsampling)
        </p>
      </div>

      {/* Actions */}
      <div className="flex gap-2 pt-2">
        <button
          onClick={handleApply}
          disabled={isLoading}
          className="flex-1 px-4 py-2 bg-blue-600 text-white rounded-md hover:bg-blue-700 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
        >
          {isLoading ? 'Applying...' : 'Apply'}
        </button>
        <button
          onClick={handleReset}
          disabled={isLoading}
          className="px-4 py-2 bg-gray-200 dark:bg-gray-600 text-gray-700 dark:text-gray-200 rounded-md hover:bg-gray-300 dark:hover:bg-gray-500 disabled:opacity-50 transition-colors"
        >
          Reset
        </button>
      </div>
    </div>
  );
}
