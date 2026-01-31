'use client'

import { useEffect } from 'react'
import { Layout } from '@/components/Layout'
import { usePointCloudStore } from '@/store/pointCloudStore'

function Sidebar() {
  const { serverConnected, checkServerHealth, stats, isLoading, error } = usePointCloudStore()

  useEffect(() => {
    checkServerHealth()
    const interval = setInterval(checkServerHealth, 5000)
    return () => clearInterval(interval)
  }, [checkServerHealth])

  return (
    <div className="p-4 space-y-6">
      <div>
        <h1 className="text-xl font-bold text-white">PointCloudLib</h1>
        <p className="text-sm text-zinc-400">Point Cloud Viewer</p>
      </div>

      <div className="flex items-center gap-2">
        <div className={`w-2 h-2 rounded-full ${serverConnected ? 'bg-green-500' : 'bg-red-500'}`} />
        <span className="text-sm text-zinc-400">Server: {serverConnected ? 'Connected' : 'Disconnected'}</span>
      </div>

      {error && (
        <div className="p-3 bg-red-900/30 border border-red-700 rounded-lg text-sm text-red-300">{error}</div>
      )}

      {stats && (
        <div className="space-y-2">
          <h2 className="text-sm font-semibold text-zinc-300">Point Cloud Stats</h2>
          <div className="text-sm text-zinc-400 space-y-1">
            <p>Points: {stats.pointCount.toLocaleString()}</p>
            <p>X: [{stats.bounds.minX.toFixed(3)}, {stats.bounds.maxX.toFixed(3)}]</p>
            <p>Y: [{stats.bounds.minY.toFixed(3)}, {stats.bounds.maxY.toFixed(3)}]</p>
            <p>Z: [{stats.bounds.minZ.toFixed(3)}, {stats.bounds.maxZ.toFixed(3)}]</p>
          </div>
        </div>
      )}

      {isLoading && (
        <div className="flex items-center gap-2 text-sm text-zinc-400"><div className="w-4 h-4 border-2 border-zinc-600 border-t-zinc-300 rounded-full animate-spin" />Processing...</div>
      )}

      <div className="pt-4 border-t border-zinc-800"><p className="text-xs text-zinc-500">File upload and filter controls will be added in Step 6</p></div>
    </div>
  )
}

function Viewer() {
  return (
    <div className="w-full h-full flex items-center justify-center bg-zinc-900">
      <div className="text-center text-zinc-500">
        <p className="text-lg">3D Viewer</p>
        <p className="text-sm">Point cloud visualization will be added in Step 6</p>
      </div>
    </div>
  )
}

export default function Home() {
  return (
    <Layout sidebar={<Sidebar/>}>
      <Viewer />
    </Layout>
  )
}
