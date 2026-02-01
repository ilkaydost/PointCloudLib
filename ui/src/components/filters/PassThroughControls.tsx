'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';

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
    <div className="space-y-4 p-4 bg-gray-50 dark:bg-gray-700 rounded-md">
      <h4 className="text-sm font-medium text-gray-900 dark:text-white">
        PassThrough Filter Settings
      </h4>

      {/* X Axis */}
      <div className="space-y-2">
        <label className="text-xs font-medium text-gray-700 dark:text-gray-300">
          X Range
        </label>
        <div className="grid grid-cols-2 gap-2">
          <input
            type="number"
            step="0.1"
            value={minX}
            onChange={(e) => setMinX(parseFloat(e.target.value))}
            className="px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
            placeholder="Min X"
          />
          <input
            type="number"
            step="0.1"
            value={maxX}
            onChange={(e) => setMaxX(parseFloat(e.target.value))}
            className="px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
            placeholder="Max X"
          />
        </div>
      </div>

      {/* Y Axis */}
      <div className="space-y-2">
        <label className="text-xs font-medium text-gray-700 dark:text-gray-300">
          Y Range
        </label>
        <div className="grid grid-cols-2 gap-2">
          <input
            type="number"
            step="0.1"
            value={minY}
            onChange={(e) => setMinY(parseFloat(e.target.value))}
            className="px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
            placeholder="Min Y"
          />
          <input
            type="number"
            step="0.1"
            value={maxY}
            onChange={(e) => setMaxY(parseFloat(e.target.value))}
            className="px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
            placeholder="Max Y"
          />
        </div>
      </div>

      {/* Z Axis */}
      <div className="space-y-2">
        <label className="text-xs font-medium text-gray-700 dark:text-gray-300">
          Z Range
        </label>
        <div className="grid grid-cols-2 gap-2">
          <input
            type="number"
            step="0.1"
            value={minZ}
            onChange={(e) => setMinZ(parseFloat(e.target.value))}
            className="px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
            placeholder="Min Z"
          />
          <input
            type="number"
            step="0.1"
            value={maxZ}
            onChange={(e) => setMaxZ(parseFloat(e.target.value))}
            className="px-2 py-1 text-sm border rounded dark:bg-gray-800 dark:border-gray-600"
            placeholder="Max Z"
          />
        </div>
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
