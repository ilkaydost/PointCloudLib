'use client'

import { ReactNode } from 'react'

interface LayoutProps { sidebar: ReactNode; children: ReactNode }

export function Layout({ sidebar, children }: LayoutProps) {
  return (
    <div className="flex h-screen bg-zinc-950 text-zinc-100">
      <aside className="w-80 flex-shrink-0 border-r border-zinc-800 bg-zinc-900 overflow-y-auto">{sidebar}</aside>
      <main className="flex-1 relative overflow-hidden">{children}</main>
    </div>
  )
}
