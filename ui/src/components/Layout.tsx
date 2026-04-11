'use client'

import { ReactNode } from 'react'
import styles from './css/Layout.module.css'
import { DonationButton } from './DonationButton'
import { AboutButton } from './AboutButton'

interface LayoutProps { sidebar: ReactNode; children: ReactNode }

export function Layout({ sidebar, children }: LayoutProps) {
  return (
    <div className={styles.layout}>
      <aside className={styles.sidebar}>{sidebar}</aside>
      <main className={styles.main}>
        <div className={styles.topActions}>
          <AboutButton />
          <DonationButton />
        </div>
        {children}
      </main>
    </div>
  )
}
