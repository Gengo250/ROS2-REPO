import { test, expect, type Page } from '@playwright/test';
import { mkdir, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';

// Evidence run of the ATLAS twin. DEMO needs only Vite; LIVE needs the lite stack and Gateway
// (ros2 launch amr_lite_sim lite_navigation.launch.py). Set TWIN_ATLAS_EVIDENCE_DIR to run.
const evidence = process.env.TWIN_ATLAS_EVIDENCE_DIR;
const API = process.env.VITE_API_URL ?? 'http://127.0.0.1:8000';

async function metrics(page: Page) {
  return JSON.parse((await page.locator('body').getAttribute('data-metrics')) ?? '{}');
}
async function shot(page: Page, name: string) { await page.screenshot({ path: resolve(evidence!, `${name}.png`) }); }

test('DEMO: prototype behaviour, cameras, follow and frame rate', async ({ page }) => {
  test.skip(!evidence, 'Set TWIN_ATLAS_EVIDENCE_DIR');
  await mkdir(evidence!, { recursive: true });
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.setViewportSize({ width: 1600, height: 900 });
  await page.goto('/?mode=demo');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true', { timeout: 30000 });
  await page.waitForTimeout(6000);
  await shot(page, 'demo_overview');
  await page.locator('[data-view="storage"]').click(); await page.waitForTimeout(2500); await shot(page, 'demo_storage');
  await page.keyboard.press('KeyF'); await page.waitForTimeout(3500); await shot(page, 'demo_follow_worker');
  // First person: a demo AMR's onboard camera (V), an industrial camera (C), back with Escape.
  await page.keyboard.press('KeyV'); await page.waitForTimeout(2500); await shot(page, 'demo_onboard_camera');
  expect((await metrics(page)).view).toBe('onboard');
  await page.keyboard.press('KeyC'); await page.waitForTimeout(2000); await shot(page, 'demo_industrial_camera');
  expect((await metrics(page)).view).toBe('industrial');
  await page.keyboard.press('Escape'); await page.waitForTimeout(1500);
  const m = await metrics(page);
  expect(m.view).toBe('overview');
  await writeFile(resolve(evidence!, 'demo_metrics.json'), JSON.stringify(m, null, 1));
  expect(m.mode).toBe('demo');
  expect(m.fps).toBeGreaterThan(30);
  expect(errors).toEqual([]);
});

test('LIVE ROS: backend state, obstacle round trip, cameras and missions', async ({ page, request }) => {
  test.skip(!evidence, 'Set TWIN_ATLAS_EVIDENCE_DIR');
  await mkdir(evidence!, { recursive: true });
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.setViewportSize({ width: 1600, height: 900 });
  await page.goto('/');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true', { timeout: 30000 });
  await expect(page.locator('#statusText')).toHaveText('AMBIENTE ATIVO', { timeout: 20000 });
  await page.waitForTimeout(4000);
  await shot(page, 'live_overview');
  // Close view of the AMR (spawn in the central hall).
  await page.locator('[data-view="storage"]').click(); await page.waitForTimeout(500);
  await page.evaluate(() => window.dispatchEvent(new KeyboardEvent('keydown', { code: 'Escape' })));
  // Obstacle placed through the UI: nothing appears until the backend accepts it.
  const before = (await (await request.get(`${API}/api/v1/world`)).json()).obstacles.length;
  await page.keyboard.press('KeyO');
  await page.locator('[data-tool="box"]').click();
  const box = await page.locator('#scene').boundingBox();
  await page.mouse.click(box!.x + box!.width * .52, box!.y + box!.height * .62);
  await page.waitForTimeout(1500);
  const after = (await (await request.get(`${API}/api/v1/world`)).json()).obstacles;
  await writeFile(resolve(evidence!, 'live_obstacle_round_trip.json'), JSON.stringify({ before, after, toast: await page.locator('#toast').textContent() }, null, 1));
  await shot(page, 'live_obstacle_placed');
  await page.locator('[data-tool="box"]').click();
  // Industrial camera in first person (no coverage geometry is drawn any more).
  await page.locator('[data-cctv="cam_06"]').click(); await page.waitForTimeout(1200);
  await page.locator('#viewCameraBtn').click(); await page.waitForTimeout(2000); await shot(page, 'live_industrial_cam06');
  const industrial = await metrics(page);
  expect([industrial.view, industrial.view_id]).toEqual(['industrial', 'cam_06']);
  // Onboard camera: the view sits on the URDF lens of the drawn AMR (same frame).
  await page.locator('#missionOnboard').click(); await page.waitForTimeout(2500); await shot(page, 'live_onboard_camera');
  const onboard = await metrics(page), base = onboard.onboard_base;
  expect(onboard.view).toBe('onboard');
  const lens = [base.x + Math.cos(base.yaw) * .477, base.z + .264, -(base.y + Math.sin(base.yaw) * .477)];
  lens.forEach((v: number, i: number) => expect(Math.abs(v - onboard.camera_position[i])).toBeLessThan(1e-3));
  await page.keyboard.press('Escape'); await page.waitForTimeout(1500);
  expect((await metrics(page)).view).toBe('overview');
  await page.locator('[data-view="overview"]').click(); await page.waitForTimeout(2500);
  // Follow a worker (backend-driven animation).
  await page.keyboard.press('KeyF'); await page.waitForTimeout(4000); await shot(page, 'live_follow_worker');
  const m = await metrics(page);
  await writeFile(resolve(evidence!, 'live_metrics.json'), JSON.stringify(m, null, 1));
  expect(m.mode).toBe('live');
  expect(m.connection).toBe('connected');
  expect(m.fps).toBeGreaterThan(30);
  expect(errors).toEqual([]);
});
