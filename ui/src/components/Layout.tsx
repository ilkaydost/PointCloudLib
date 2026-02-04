'use client'

import { ReactNode } from 'react'
import styles from './css/Layout.module.css'

interface LayoutProps { sidebar: ReactNode; children: ReactNode }

export function Layout({ sidebar, children }: LayoutProps) {
  return (
    <div className={styles.layout}>
      <aside className={styles.sidebar}>{sidebar}</aside>
      <main className={styles.main}>{children}</main>
    </div>
  )
}
