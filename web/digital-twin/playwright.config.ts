import { defineConfig } from '@playwright/test';
export default defineConfig({
  testDir: './test', testMatch: '**/*.browser.ts', workers: 1,
  timeout: 180000, use: { baseURL: 'http://127.0.0.1:5173', headless: true },
});
