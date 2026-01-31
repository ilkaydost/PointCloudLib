import './globals.css'
import type { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'PointCloudLib',
  description: 'Point Cloud Visualization and Processing',
}

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body className="antialiased">
        {children}
      </body>
    </html>
  )
}
