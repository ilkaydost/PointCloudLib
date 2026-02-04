const API_BASE = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:5050/api'

export interface PointCloudStats {
  pointCount: number
  bounds: {
    minX: number; maxX: number; minY: number; maxY: number; minZ: number; maxZ: number
  }
}

export interface LoadResponse { success: boolean; stats: PointCloudStats; error?: string }

export interface RansacResponse {
  success: boolean
  stats: PointCloudStats
  planeCoefficients?: number[]
  inlierCount?: number
  planePoints?: number
  remainingPoints?: number
  error?: string
}

export const api = {
  async healthCheck(): Promise<boolean> {
    try {
      const res = await fetch(`${API_BASE}/health`)
      return res.ok
    } catch {
      return false
    }
  },

  async loadPointCloud(filePath: string): Promise<LoadResponse> {
    const res = await fetch(`${API_BASE}/load`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ path: filePath }),
    })
    return res.json()
  },

  async uploadPointCloud(file: File): Promise<LoadResponse> {
    const formData = new FormData()
    formData.append('file', file)
    const res = await fetch(`${API_BASE}/upload`, {
      method: 'POST',
      body: formData,
    })
    return res.json()
  },

  async getPositionsBinary(): Promise<Float32Array> {
    const res = await fetch(`${API_BASE}/points/positions`)
    const buffer = await res.arrayBuffer()
    return new Float32Array(buffer)
  },

  async getColorsBinary(): Promise<Uint8Array> {
    const res = await fetch(`${API_BASE}/points/colors`)
    const buffer = await res.arrayBuffer()
    return new Uint8Array(buffer)
  },

  async applyPassThroughFilter(config: any) {
    const res = await fetch(`${API_BASE}/filter`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: 'passthrough', config }),
    })
    return res.json()
  },

  async applyVoxelGridFilter(config: any) {
    const res = await fetch(`${API_BASE}/filter`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type: 'voxelgrid', config }),
    })
    return res.json()
  },

  async applyRansacSegmentation(config: {
    distanceThreshold: number
    maxIterations: number
    extractInliers: boolean
  }) {
    const res = await fetch(`${API_BASE}/segment/ransac`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    })
    return res.json()
  }
}
