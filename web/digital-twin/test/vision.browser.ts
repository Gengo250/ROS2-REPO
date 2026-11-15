import { test, expect, type Page } from '@playwright/test';
import { mkdir, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';

// LIVE, three AMRs: each robot's onboard camera follows that robot, and its computer-vision view
// shows the frames the perception service processed for that robot, with the boxes on the image.
// Needs the lite twin + Gateway (3 robots) and vision/scripts/serve_perception.py.
// Set TWIN_VISION_EVIDENCE_DIR to run (screenshots and a JSON report go there).
const evidence = process.env.TWIN_VISION_EVIDENCE_DIR;
const API = process.env.VITE_API_URL ?? 'http://127.0.0.1:8000';
const ROBOTS = ['amr_01', 'amr_02', 'amr_03'];

async function metrics(page: Page) { return JSON.parse((await page.locator('body').getAttribute('data-metrics')) ?? '{}'); }
async function overlayBox(page: Page) { return page.locator('canvas.vision-overlay').boundingBox(); }

test('LIVE: three AMRs, onboard cameras and per-robot detections', async ({ page }) => {
  test.skip(!evidence, 'Set TWIN_VISION_EVIDENCE_DIR');
  test.setTimeout(240000);
  await mkdir(evidence!, { recursive: true });
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  await page.setViewportSize({ width: 1600, height: 900 });
  await page.goto('/');
  await expect(page.locator('body')).toHaveAttribute('data-ready', 'true', { timeout: 60000 });
  await expect(page.locator('#robotCount')).toHaveText('03', { timeout: 30000 });
  await expect(page.locator('#missionRobot option')).toHaveCount(3);
  await page.waitForTimeout(2000);
  await page.screenshot({ path: resolve(evidence!, 'live_overview_three_amrs.png') });
  const report: Record<string, unknown> = {};
  for (const [i, robot] of ROBOTS.entries()) {
    await page.locator('#missionRobot').selectOption(robot);
    await page.locator('#missionOnboard').click();
    await page.waitForTimeout(1500);
    const m = await metrics(page);
    expect(m.view).toBe('onboard'); expect(m.view_id).toBe(robot);
    // The lens follows this robot: the pose it was placed from is this robot's backend pose.
    const world = await (await fetch(`${API}/api/v1/world`)).json() as { entities: { id: string; x: number; y: number }[] };
    const entity = world.entities.find(e => e.id === robot)!;
    expect(Math.hypot(m.onboard_base.x - entity.x, m.onboard_base.y - entity.y)).toBeLessThan(.5);
    const panel = page.locator('.vision-panel');
    await expect(panel).toBeVisible();
    if (i === 0) await panel.locator('[data-v="enable"]').check();
    if (i === 0) await panel.locator('[data-v="confidence"]').check();
    await expect(panel.locator('[data-v="pill"]')).toHaveText('EXECUTANDO', { timeout: 30000 });
    await expect(panel.locator('[data-v="meta"]')).toContainText(`Robô: ${robot}`);
    await page.waitForTimeout(2500);
    const box = await overlayBox(page), viewport = (await metrics(page)).viewport;
    // The processed frame is drawn exactly over the onboard viewport.
    expect(Math.abs(box!.x - viewport.x)).toBeLessThan(1.5); expect(Math.abs(box!.width - viewport.width)).toBeLessThan(1.5);
    expect(Math.abs(box!.y - viewport.y)).toBeLessThan(1.5); expect(Math.abs(box!.height - viewport.height)).toBeLessThan(1.5);
    report[robot] = { onboard_base: m.onboard_base, entity, viewport, meta: await panel.locator('[data-v="meta"]').innerText() };
    await page.screenshot({ path: resolve(evidence!, `live_${robot}_detections.png`) });
  }
  // Resize: the overlay follows the new viewport (boxes are drawn in frame coordinates, scaled with it).
  await page.setViewportSize({ width: 1280, height: 760 });
  await page.waitForTimeout(2500);
  const box = await overlayBox(page), viewport = (await metrics(page)).viewport;
  expect(Math.abs(box!.width - viewport.width)).toBeLessThan(1.5); expect(Math.abs(box!.height - viewport.height)).toBeLessThan(1.5);
  await page.screenshot({ path: resolve(evidence!, 'live_amr_03_detections_resized.png') });
  // Detection off: the overlay disappears and the 3D onboard view is back.
  await page.locator('.vision-panel [data-v="enable"]').uncheck();
  await page.waitForTimeout(800);
  await expect(page.locator('canvas.vision-overlay')).toBeHidden();
  await page.keyboard.press('Escape'); await page.waitForTimeout(1200);
  await expect(page.locator('.vision-panel')).toBeHidden();
  report.final_metrics = await metrics(page);
  report.page_errors = errors;
  await writeFile(resolve(evidence!, 'vision_browser.json'), JSON.stringify(report, null, 1));
  expect(errors).toEqual([]);
});
