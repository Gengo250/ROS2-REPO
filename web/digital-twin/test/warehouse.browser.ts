import { test, expect } from '@playwright/test';
import { mkdir, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
const evidence = process.env.TWIN_SCENE_EVIDENCE_DIR;

test('real scene API, four scenarios, original GLB, cameras and resize', async ({ page }) => {
  test.skip(!evidence, 'Set TWIN_SCENE_EVIDENCE_DIR with real Gateway and Vite running');
  await mkdir(evidence!, { recursive: true });
  const errors: string[] = [];
  page.on('pageerror', e => errors.push(e.message));
  page.on('console', m => { if (m.type() === 'error') errors.push(m.text()); });
  await page.setViewportSize({width:1440,height:1100});
  await page.goto('/');
  const canvas = page.locator('#warehouse');
  const metrics: Record<string, unknown> = {};
  for (const scenario of ['normal','person_crossing','obstacle_in_aisle','corridor_blocked']) {
    await page.locator('#scene-select').selectOption(scenario);
    await expect(canvas).toHaveAttribute('data-loaded','true', {timeout:30000});
    await expect(canvas).toHaveAttribute('data-scenario',scenario);
    await expect(page.locator('#scene-status')).toContainText('Configured spawn');
    await page.waitForTimeout(1500);
    await page.screenshot({path:resolve(evidence!,`${scenario}.png`),fullPage:true});
    metrics[scenario] = JSON.parse((await canvas.getAttribute('data-metrics'))!);
    expect((metrics[scenario] as {geometries:number}).geometries).toBeLessThanOrEqual(52);
  }
  await page.locator('#scene-select').selectOption('normal');
  await expect(canvas).toHaveAttribute('data-loaded','true');
  for (const [button,name] of [['Overview','overview'],['Robot focus','robot-close-up'],['Top-down','top-down']]) {
    await page.getByRole('button',{name:button,exact:true}).click();
    await page.waitForTimeout(1200);
    await page.screenshot({path:resolve(evidence!,`${name}.png`),fullPage:true});
  }
  await page.getByRole('button',{name:'Overview',exact:true}).click();
  await page.waitForTimeout(1200);
  const before = JSON.parse((await canvas.getAttribute('data-metrics'))!).camera;
  await page.mouse.move(700,400); await page.mouse.down(); await page.mouse.move(850,460,{steps:10}); await page.mouse.up();
  await page.mouse.wheel(0,-200); await page.waitForTimeout(1500);
  expect(JSON.parse((await canvas.getAttribute('data-metrics'))!).camera).not.toEqual(before);
  await page.setViewportSize({width:900,height:800});
  await page.waitForTimeout(500);
  expect(await page.locator('canvas').evaluate(c => c.clientWidth)).toBeLessThan(900);
  await page.evaluate(() => window.dispatchEvent(new PageTransitionEvent('pagehide')));
  await expect(page.locator('#warehouse canvas')).toHaveCount(0);
  expect(errors).toEqual([]);
  await writeFile(resolve(evidence!,'rendering.json'),JSON.stringify({metrics,errors},null,2));
});

test('asset failure stays explicit and preserves diagnostics', async ({page}) => {
  test.skip(!evidence, 'Requires real scene API');
  await page.route('**/warehouse_amr.glb',r=>r.fulfill({status:404,body:''}));
  await page.goto('/');
  await expect(page.locator('#scene-status')).toContainText('AMR GLB unavailable',{timeout:30000});
  await expect(page.locator('#gateway')).toHaveText('Connected');
});
