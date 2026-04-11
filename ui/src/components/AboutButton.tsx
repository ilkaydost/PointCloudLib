'use client';

import { useState } from 'react';
import styles from './css/AboutModal.module.css';

export function AboutButton(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      <button
        onClick={() => setIsOpen(true)}
        className={styles.button}
        title="About & Ideas"
      >
        <svg
          className={styles.icon}
          viewBox="0 0 24 24"
          fill="currentColor"
          aria-hidden="true"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-6h2v6zm0-8h-2V7h2v2z" />
        </svg>
        About & Ideas
      </button>

      {isOpen && (
        <div className={styles.overlay} onClick={() => setIsOpen(false)}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <button
              onClick={() => setIsOpen(false)}
              className={styles.closeButton}
              aria-label="Close"
            >
              <svg className={styles.closeIcon} viewBox="0 0 24 24" fill="currentColor">
                <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
              </svg>
            </button>

            <h2 className={styles.title}>About PointCloudLib</h2>

            <div className={styles.section}>
              <h3 className={styles.sectionTitle}>What is this?</h3>
              <p className={styles.text}>
                PointCloudLib is a web-based point cloud visualization and processing application
                built on PCL (Point Cloud Library). It provides interactive 3D viewing with
                real-time filtering, segmentation, and analysis tools — all from your browser.
              </p>
            </div>

            <div className={styles.section}>
              <h3 className={styles.sectionTitle}>Tech Stack</h3>
              <ul className={styles.featureList}>
                <li>C++ core with PCL for point cloud processing</li>
                <li>React + Next.js + Tailwind CSS for the UI</li>
                <li>Three.js (react-three-fiber) for 3D visualization</li>
                <li>HTTP/WebSocket bridge between C++ backend and web frontend</li>
              </ul>
            </div>

            <div className={styles.divider} />

            <h2 className={styles.title}>Further Ideas & Requests</h2>

            <div className={styles.section}>
              <h3 className={styles.sectionTitle}>Planned Features</h3>
              <ul className={styles.featureList}>
                <li>NDT (Normal Distributions Transform) registration</li>
                <li>PFH / FPFH feature descriptors</li>
                <li>Euclidean cluster extraction</li>
                <li>Cylinder segmentation</li>
                <li>MLS surface smoothing & mesh reconstruction</li>
                <li>PLY file format export</li>
                <li>Point cloud comparison / diff view</li>
                <li>Measurement tools (distances, angles)</li>
              </ul>
            </div>

            <div className={styles.section}>
              <h3 className={styles.sectionTitle}>Have an idea?</h3>
              <p className={styles.text}>
                Feature requests and bug reports are welcome! Open an issue on the{' '}
                <a
                  href="https://github.com/ilkaydost/PointCloudLib/issues"
                  target="_blank"
                  rel="noopener noreferrer"
                  className={styles.link}
                >
                  GitHub repository
                </a>{' '}
                or contribute directly via pull requests.
              </p>
              <p className={styles.ideasNote}>
                Community contributions help shape the roadmap!
              </p>
            </div>
          </div>
        </div>
      )}
    </>
  );
}
