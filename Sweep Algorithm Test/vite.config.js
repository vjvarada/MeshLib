import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  css: {
    // Inline PostCSS config — prevents Vite from searching parent directories
    // for a postcss.config.js and hitting BOM-corrupted JSON files upstream
    postcss: {
      plugins: [],
    },
  },
})
