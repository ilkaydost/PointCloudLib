'use client'

import { useState } from 'react'
import { usePointCloudStore } from '@/store/pointCloudStore'
import styles from './css/ICPRegistrationControls.module.css'

export default function ICPRegistrationControls() {
  const { 
    setupICP, 
    alignICP, 
    iterateICP, 
    resetICP,
    icpSetup,
    icpIterations,
    icpFitnessScore,
    icpTransformation,
    isLoading 
  } = usePointCloudStore()

  const [applyTransform, setApplyTransform] = useState(true)
  const [rotationAngle, setRotationAngle] = useState(Math.PI / 8) // 22.5 degrees
  const [translationZ, setTranslationZ] = useState(0.4)
  const [maxIterations, setMaxIterations] = useState(1)
  const [maxCorrespondenceDistance, setMaxCorrespondenceDistance] = useState(0.05)
  const [transformationEpsilon, setTransformationEpsilon] = useState(1e-8)
  const [euclideanFitnessEpsilon, setEuclideanFitnessEpsilon] = useState(1e-6)
  const [batchIterations, setBatchIterations] = useState(10)

  const handleSetup = async () => {
    await setupICP({
      mode: 'current_as_target',
      applyTransform,
      rotationAngle,
      translationZ,
      maxIterations,
      maxCorrespondenceDistance,
      transformationEpsilon,
      euclideanFitnessEpsilon
    })
  }

  const handleAlign = async () => {
    await alignICP(batchIterations)
  }

  const handleIterate = async () => {
    await iterateICP()
  }

  const handleReset = async () => {
    await resetICP()
  }

  const formatMatrix = (matrix: number[][] | null) => {
    if (!matrix) return 'No transformation yet'
    
    return matrix.map((row, i) => {
      const formattedRow = row.map(val => val.toFixed(3)).join('  ')
      if (i === 0) return `R = | ${formattedRow} |`
      if (i === 1) return `    | ${formattedRow} |`
      if (i === 2) return `    | ${formattedRow} |`
      return `t = < ${row.slice(0, 3).map(v => v.toFixed(3)).join(', ')} >`
    }).join('\n')
  }

  return (
    <div className={styles.container}>
      <h3 className={styles.title}>ICP Registration</h3>
      
      {!icpSetup ? (
        <div className={styles.setupSection}>
          <h4 className={styles.subtitle}>Setup</h4>
          
          <div className={styles.paramGroup}>
            <label className={styles.checkbox}>
              <input 
                type="checkbox" 
                checked={applyTransform}
                onChange={(e) => setApplyTransform(e.target.checked)}
              />
              <span>Apply initial transformation (for demo)</span>
            </label>
          </div>

          {applyTransform && (
            <>
              <div className={styles.paramGroup}>
                <label>
                  Rotation Angle (radians): {rotationAngle.toFixed(3)}
                  <input 
                    type="range"
                    min="0"
                    max={Math.PI}
                    step="0.01"
                    value={rotationAngle}
                    onChange={(e) => setRotationAngle(parseFloat(e.target.value))}
                    className={styles.slider}
                  />
                </label>
              </div>

              <div className={styles.paramGroup}>
                <label>
                  Translation Z: {translationZ.toFixed(2)}
                  <input 
                    type="range"
                    min="-1"
                    max="1"
                    step="0.01"
                    value={translationZ}
                    onChange={(e) => setTranslationZ(parseFloat(e.target.value))}
                    className={styles.slider}
                  />
                </label>
              </div>
            </>
          )}

          <div className={styles.paramGroup}>
            <label>
              Max Correspondence Distance: {maxCorrespondenceDistance.toFixed(3)}
              <input 
                type="range"
                min="0.01"
                max="0.5"
                step="0.01"
                value={maxCorrespondenceDistance}
                onChange={(e) => setMaxCorrespondenceDistance(parseFloat(e.target.value))}
                className={styles.slider}
              />
            </label>
          </div>

          <div className={styles.paramGroup}>
            <label>
              Transformation Epsilon: {transformationEpsilon.toExponential(1)}
              <input 
                type="range"
                min="-10"
                max="-6"
                step="0.1"
                value={Math.log10(transformationEpsilon)}
                onChange={(e) => setTransformationEpsilon(Math.pow(10, parseFloat(e.target.value)))}
                className={styles.slider}
              />
            </label>
          </div>

          <div className={styles.paramGroup}>
            <label>
              Euclidean Fitness Epsilon: {euclideanFitnessEpsilon.toExponential(1)}
              <input 
                type="range"
                min="-8"
                max="-4"
                step="0.1"
                value={Math.log10(euclideanFitnessEpsilon)}
                onChange={(e) => setEuclideanFitnessEpsilon(Math.pow(10, parseFloat(e.target.value)))}
                className={styles.slider}
              />
            </label>
          </div>

          <button 
            onClick={handleSetup}
            disabled={isLoading}
            className={styles.button}
          >
            Initialize ICP
          </button>
        </div>
      ) : (
        <div className={styles.alignSection}>
          <h4 className={styles.subtitle}>Alignment</h4>
          
          <div className={styles.stats}>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Iterations:</span>
              <span className={styles.statValue}>{icpIterations}</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statLabel}>Fitness Score:</span>
              <span className={styles.statValue}>
                {icpFitnessScore !== null ? icpFitnessScore.toExponential(4) : 'N/A'}
              </span>
            </div>
          </div>

          <div className={styles.transformMatrix}>
            <pre className={styles.matrixText}>{formatMatrix(icpTransformation)}</pre>
          </div>

          <div className={styles.controlButtons}>
            <div className={styles.paramGroup}>
              <label>
                Batch Iterations: {batchIterations}
                <input 
                  type="range"
                  min="1"
                  max="50"
                  step="1"
                  value={batchIterations}
                  onChange={(e) => setBatchIterations(parseInt(e.target.value))}
                  className={styles.slider}
                />
              </label>
            </div>

            <button 
              onClick={handleAlign}
              disabled={isLoading}
              className={styles.button}
            >
              Align ({batchIterations} iterations)
            </button>

            <button 
              onClick={handleIterate}
              disabled={isLoading}
              className={`${styles.button} ${styles.iterateButton}`}
            >
              Iterate Once (Space)
            </button>

            <button 
              onClick={handleReset}
              disabled={isLoading}
              className={`${styles.button} ${styles.resetButton}`}
            >
              Reset ICP
            </button>
          </div>
        </div>
      )}
    </div>
  )
}
