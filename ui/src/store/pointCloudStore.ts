import { create } from 'zustand'
import { api, PointCloudStats } from '@/lib/api'

interface PointCloudState {
  positions: Float32Array | null
  colors: Uint8Array | null
  stats: PointCloudStats | null
  isLoading: boolean
  error: string | null
  serverConnected: boolean
  checkServerHealth: () => Promise<void>
  loadPointCloud: (filePath: string) => Promise<void>
  uploadPointCloud: (file: File) => Promise<void>
  fetchPoints: () => Promise<void>
  applyPassThroughFilter: (field: 'x'|'y'|'z', min:number, max:number) => Promise<void>
  applyVoxelGridFilter: (leafSize:number) => Promise<void>
  applyRansacSegmentation: (distanceThreshold: number, maxIterations: number, extractInliers: boolean) => Promise<void>
  applyRegionGrowing: (config: {
    smoothnessThreshold: number
    curvatureThreshold: number
    minClusterSize: number
    maxClusterSize: number
    numberOfNeighbours: number
    normalKSearch: number
  }) => Promise<void>
  reset: () => void
}

export const usePointCloudStore = create<PointCloudState>((set, get) => ({
  positions: null,
  colors: null,
  stats: null,
  isLoading: false,
  error: null,
  serverConnected: false,

  checkServerHealth: async () => {
    const connected = await api.healthCheck()
    set({ serverConnected: connected })
  },

  loadPointCloud: async (filePath: string) => {
    set({ isLoading: true, error: null })
    try {
      const response = await api.loadPointCloud(filePath)
      if (response.success) {
        set({ stats: response.stats })
        await get().fetchPoints()
      } else {
        set({ error: response.error || 'Failed to load point cloud' })
      }
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Unknown error' })
    } finally {
      set({ isLoading: false })
    }
  },

  uploadPointCloud: async (file: File) => {
    set({ isLoading: true, error: null })
    try {
      const response = await api.uploadPointCloud(file)
      if (response.success) {
        set({ stats: response.stats })
        await get().fetchPoints()
      } else {
        set({ error: response.error || 'Failed to upload point cloud' })
      }
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Unknown error' })
    } finally {
      set({ isLoading: false })
    }
  },

  fetchPoints: async () => {
    try {
      const [positions, colors] = await Promise.all([
        api.getPositionsBinary(),
        api.getColorsBinary(),
      ])
      set({ positions, colors })
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Failed to fetch points' })
    }
  },

  applyPassThroughFilter: async (field, min, max) => {
    set({ isLoading: true, error: null })
    try {
      const response = await api.applyPassThroughFilter({ fieldName: field, minLimit: min, maxLimit: max })
      if (response.success) {
        set({ stats: response.stats })
        await get().fetchPoints()
      } else {
        set({ error: response.error || 'Filter failed' })
      }
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Unknown error' })
    } finally { set({ isLoading: false }) }
  },

  applyVoxelGridFilter: async (leafSize) => {
    set({ isLoading: true, error: null })
    try {
      const response = await api.applyVoxelGridFilter({ leafSize })
      if (response.success) {
        set({ stats: response.stats })
        await get().fetchPoints()
      } else {
        set({ error: response.error || 'Filter failed' })
      }
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Unknown error' })
    } finally { set({ isLoading: false }) }
  },

  applyRansacSegmentation: async (distanceThreshold, maxIterations, extractInliers) => {
    set({ isLoading: true, error: null })
    try {
      const response = await api.applyRansacSegmentation({ distanceThreshold, maxIterations, extractInliers })
      if (response.success) {
        set({ stats: response.stats })
        await get().fetchPoints()
      } else {
        set({ error: response.error || 'RANSAC segmentation failed' })
      }
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Unknown error' })
    } finally { set({ isLoading: false }) }
  },

  applyRegionGrowing: async (config) => {
    set({ isLoading: true, error: null })
    try {
      const response = await api.applyRegionGrowing(config)
      if (response.success) {
        set({ stats: response.stats })
        await get().fetchPoints()
      } else {
        set({ error: response.error || 'Region growing segmentation failed' })
      }
    } catch (e) {
      set({ error: e instanceof Error ? e.message : 'Unknown error' })
    } finally { set({ isLoading: false }) }
  },

  reset: () => set({ positions: null, colors: null, stats: null, error: null }),
}))
