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

export interface RegionGrowingResponse {
  success: boolean
  stats: PointCloudStats
  numClusters: number
  clusters?: Array<{ id: number; pointCount: number }>
  error?: string
}

export interface NormalEstimationResponse {
  success: boolean
  stats: PointCloudStats
  error?: string
}

export interface ICPSetupResponse {
  success: boolean
  sourcePoints: number
  targetPoints: number
  error?: string
}

export interface ICPAlignResponse {
  success: boolean
  converged: boolean
  fitnessScore: number
  iterationsDone: number
  transformation: number[][]
  cumulativeTransformation: number[][]
  stats: PointCloudStats
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

  async getNormalsBinary(kSearch?: number, radiusSearch?: number): Promise<Float32Array> {
    const url = new URL(`${API_BASE}/features/normals/binary`);
    if (kSearch !== undefined) url.searchParams.set('kSearch', String(kSearch));
    if (radiusSearch !== undefined) url.searchParams.set('radiusSearch', String(radiusSearch));
    const res = await fetch(url.toString());
    const buffer = await res.arrayBuffer();
    return new Float32Array(buffer);
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
  },

  async applyRegionGrowing(config: {
    smoothnessThreshold: number
    curvatureThreshold: number
    minClusterSize: number
    maxClusterSize: number
    numberOfNeighbours: number
    normalKSearch: number
  }): Promise<RegionGrowingResponse> {
    const res = await fetch(`${API_BASE}/segment/regiongrowing`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    })
    return res.json()
  },

  async applyNormalEstimation(config: {
    kSearch?: number
    radiusSearch?: number
  }): Promise<NormalEstimationResponse> {
    const res = await fetch(`${API_BASE}/features/normals`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    })
    return res.json()
  },

  async setupICP(config: {
    mode?: string
    applyTransform?: boolean
    rotationAngle?: number
    translationZ?: number
    maxIterations?: number
    maxCorrespondenceDistance?: number
    transformationEpsilon?: number
    euclideanFitnessEpsilon?: number
  }): Promise<ICPSetupResponse> {
    const res = await fetch(`${API_BASE}/icp/setup`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(config),
    })
    return res.json()
  },

  async alignICP(iterations: number = 1): Promise<ICPAlignResponse> {
    const res = await fetch(`${API_BASE}/icp/align`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ iterations }),
    })
    return res.json()
  },

  async iterateICP(): Promise<ICPAlignResponse> {
    const res = await fetch(`${API_BASE}/icp/iterate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
    })
    return res.json()
  },

  async resetICP(): Promise<{ success: boolean }> {
    const res = await fetch(`${API_BASE}/icp/reset`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
    })
    return res.json()
  }
}
