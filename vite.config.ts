import { defineConfig } from 'vite';
import { resolve } from 'path';

export default defineConfig({
  root: 'examples',
  resolve: {
    alias: {
      '@lib': resolve(__dirname, 'src')
    }
  },
  server: {
    open: true
  },
  build: {
    outDir: '../dist-example',
    emptyOutDir: true
  }
});
