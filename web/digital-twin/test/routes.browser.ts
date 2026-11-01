import { test, expect } from '@playwright/test';
import { mkdir, writeFile, access } from 'node:fs/promises';
import { resolve } from 'node:path';

// Read-only observer of a real owned route run: never publishes commands or edits poses.
const evidence = process.env.ROUTE_EVIDENCE_DIR;
test.use({ video: evidence ? { mode: 'on', size: { width: 1280, height: 720 } } : 'off' });
test('shows the planned route and real AMR movement in the existing viewer', async ({ page }) => {
  test.skip(!evidence, 'Requires the real owned route integration run');
  // Observe until the harness writes navigation.done, within a wall-clock window it sizes per run.
  const seconds = Number(process.env.ROUTE_OBSERVE_SECONDS || 300);
  test.setTimeout((seconds+300)*1000);
  await mkdir(evidence!, { recursive: true });
  const errors: string[] = []; page.on('pageerror', e => errors.push(e.message));
  await page.goto('/diagnostics.html'); const host = page.locator('#warehouse');
  await expect(host).toHaveAttribute('data-loaded', 'true', { timeout: 60000 });
  await expect.poll(async () => Object.keys(JSON.parse(await host.getAttribute('data-transforms') || '{}')).length).toBe(4);
  await page.getByRole('button', { name: 'Top-down', exact: true }).click();
  await writeFile(resolve(evidence!, 'browser.ready'), 'ready');
  const samples: any[] = []; const shots = new Set<string>();
  try {
    const deadline = Date.now()+seconds*1000;
    for (let i = 0; Date.now() < deadline; i++) {
      const s: any = await host.evaluate(e => ({ wall: Date.now(), route: JSON.parse(e.dataset.route || '{}'), roots: JSON.parse(e.dataset.transforms || '{}'), metrics: JSON.parse(e.dataset.metrics || '{}') }));
      s.panel = await page.locator('#route-status').innerText();
      samples.push(s);
      const key = `${s.route.route_id}-${s.route.status}`;
      if (s.route.status && !shots.has(key)) { shots.add(key); await page.screenshot({ path: resolve(evidence!, `route-${shots.size}-${s.route.status}.png`), fullPage: true }); }
      if (i % 30 === 15) await page.screenshot({ path: resolve(evidence!, `route-progress-${i}.png`) });
      await page.waitForTimeout(1000);
      if (i > 10 && await access(resolve(evidence!, 'navigation.done')).then(() => true, () => false)) break;
    }
    await page.screenshot({ path: resolve(evidence!, 'route-final.png'), fullPage: true });
    expect(errors).toEqual([]);
    expect(samples.some(s => s.route.visible)).toBe(true);
    expect(samples.at(-1).metrics.human_models).toBe(3);
    const robot: number[][] = samples.map(s => s.roots.amr_01?.position).filter(Boolean);
    const first = robot[0]!;
    expect(Math.max(...robot.map(p => Math.hypot(p[0]! - first[0]!, p[1]! - first[1]!)))).toBeGreaterThan(.5);
    const final = await (await page.request.get('http://127.0.0.1:8000/api/v2/state')).json();
    const truth = final.simulation_robots[0].pose.position;
    expect(Math.hypot(samples.at(-1).roots.amr_01.position[0] - truth.x, samples.at(-1).roots.amr_01.position[1] - truth.y)).toBeLessThan(.6);
  } finally {
    await writeFile(resolve(evidence!, 'browser.json'), JSON.stringify({ samples, errors }, null, 2));
    const video = page.video(); await page.close(); if (video) await video.saveAs(resolve(evidence!, 'route.webm'));
  }
});
