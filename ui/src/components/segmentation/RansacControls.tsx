'use client';

import { useState } from 'react';
import { usePointCloudStore } from '../../store/pointCloudStore';

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
    <div className="bg-gray-50 dark:bg-gray-700 rounded-md p-4 space-y-4">
      <h4 className="text-sm font-medium text-gray-900 dark:text-white">
        RANSAC Plane Segmentation
      </h4>

      {/* Distance Threshold */}
      <div>
        <label className="block text-xs text-gray-600 dark:text-gray-300 mb-1">
          Distance Threshold (m): {distanceThreshold.toFixed(3)}
        </label>
        <input
          type="range"
          min="0.001"
          max="0.1"
          step="0.001"
          value={distanceThreshold}
          onChange={(e) => setDistanceThreshold(parseFloat(e.target.value))}
          className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-600"
        />
        <div className="flex justify-between text-xs text-gray-500 dark:text-gray-400">
          <span>0.001</span>
          <span>0.1</span>
        </div>
      </div>

      {/* Max Iterations */}
      <div>
        <label className="block text-xs text-gray-600 dark:text-gray-300 mb-1">
          Max Iterations: {maxIterations}
        </label>
        <input
          type="range"
          min="100"
          max="5000"
          step="100"
          value={maxIterations}
          onChange={(e) => setMaxIterations(parseInt(e.target.value))}
          className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer dark:bg-gray-600"
        />
        <div className="flex justify-between text-xs text-gray-500 dark:text-gray-400">
          <span>100</span>
          <span>5000</span>
        </div>
      </div>

      {/* Extract Inliers Toggle */}
      <div className="flex items-center justify-between">
        <label className="text-xs text-gray-600 dark:text-gray-300">
          Extract: {extractInliers ? 'Plane (inliers)' : 'Remaining (outliers)'}
        </label>
        <button
          onClick={() => setExtractInliers(!extractInliers)}
          className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors ${
            extractInliers ? 'bg-blue-600' : 'bg-gray-300 dark:bg-gray-600'
          }`}
        >
          <span
            className={`inline-block h-4 w-4 transform rounded-full bg-white transition-transform ${
              extractInliers ? 'translate-x-6' : 'translate-x-1'
            }`}
          />
        </button>
      </div>

      {/* Apply Button */}
      <button
        onClick={handleApply}
        disabled={isLoading}
        className="w-full px-4 py-2 bg-green-600 text-white rounded-md hover:bg-green-700 
                   disabled:bg-gray-400 disabled:cursor-not-allowed transition-colors text-sm font-medium"
      >
        {isLoading ? 'Processing...' : 'Apply RANSAC'}
      </button>

      {/* Info */}
      <p className="text-xs text-gray-500 dark:text-gray-400">
        RANSAC finds the dominant plane in the point cloud. Toggle to extract either the plane or the remaining points.
      </p>
    </div>
  );
}
