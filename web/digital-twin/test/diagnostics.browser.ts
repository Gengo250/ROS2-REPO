import { test, expect } from '@playwright/test';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
import { writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';

const evidence = process.env.TWIN_EVIDENCE_DIR;
test('real ROS odometry reaches the browser and follows controlled motion', async ({ page }) => {
  test.skip(!evidence, 'Run the real integration harness to start Gazebo and the Gateway.');
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/');
  await expect(page.locator('#gateway')).toHaveText('Connected', { timeout: 90000 });
  await expect(page.locator('#ros')).toHaveText('Online');
  await expect(page.locator('#gazebo')).toHaveText('Running', { timeout: 90000 });
  await expect(page.locator('#count')).toHaveText('1', { timeout: 90000 });
  await expect(page.locator('[data-value="frames"]')).toContainText('Pose frame: odom');
  const before = Number(await page.locator('[data-value="x"]').textContent());
  await page.screenshot({ path: resolve(evidence!, 'browser-before.png'), fullPage: true });
  const root = resolve('../..');
  await promisify(execFile)(resolve(root, '.venv/bin/python'), [
    resolve(root, 'src/digital_twin_gateway/scripts/validate_motion.py'),
    '--output', resolve(evidence!, 'motion.json'),
  ], { timeout: 120000 });
  await expect.poll(async () => Number(await page.locator('[data-value="x"]').textContent()) - before,
    { timeout: 15000 }).toBeGreaterThan(.03);
  const after = Number(await page.locator('[data-value="x"]').textContent());
  await page.screenshot({ path: resolve(evidence!, 'browser-after.png'), fullPage: true });
  await writeFile(resolve(evidence!, 'browser.json'), JSON.stringify({ before_x: before, after_x: after,
    text: await page.locator('main').innerText(), page_errors: errors }, null, 2));
  expect(errors).toEqual([]);
  // Browser network loss and recovery against the same real backend.
  await page.context().setOffline(true);
  await expect(page.locator('#gateway')).toHaveText(/Disconnected|Connecting/, { timeout: 20000 });
  await expect(page.locator('#count')).toHaveText('0');
  await page.screenshot({ path: resolve(evidence!, 'browser-disconnected.png'), fullPage: true });
  await page.context().setOffline(false);
  await expect(page.locator('#gateway')).toHaveText('Connected', { timeout: 20000 });
  await expect(page.locator('#count')).toHaveText('1');
});

test('frontend starts and remains usable with no backend', async ({ page }) => {
  await page.route('http://127.0.0.1:8000/**', route => route.abort());
  await page.routeWebSocket('ws://127.0.0.1:8000/**', socket => socket.close());
  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'Digital Twin', exact: true })).toBeVisible();
  await expect(page.locator('#gateway')).toHaveText(/Disconnected|Connecting/);
  await expect(page.locator('#count')).toHaveText('0');
  await expect(page.locator('#robots')).toContainText('Waiting for real robot telemetry');
});
