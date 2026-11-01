import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';
// Two pages: the ATLAS twin (index.html, LIVE ROS / DEMO) and the Phase 5 diagnostics page.
const page = (name: string) => fileURLToPath(new URL(name, import.meta.url));
export default defineConfig({
  server: { host: '127.0.0.1', port: 5173, strictPort: true },
  build: { rollupOptions: { input: { main: page('index.html'), diagnostics: page('diagnostics.html') } } },
});
