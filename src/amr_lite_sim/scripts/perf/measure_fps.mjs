// Browser frame-rate probe: real GPU (ANGLE/EGL), rAF-based FPS, long-frame count, WebGL renderer.
// Usage: node measure_fps.mjs <url> <seconds> <out.json> [screenshot.png] [--headed] [--size=1600x900] [--vision=amr_02]
// --vision=<robot> (LIVE, Phase 7): selects that AMR, opens its onboard camera and enables object
// detection (VISÃO COMPUTACIONAL panel) before measuring; the panel's text is recorded at the end.
// Node >= 22. Playwright is the frontend's dev dependency:
// PLAYWRIGHT_MODULE=$PWD/web/digital-twin/node_modules/playwright/index.mjs node measure_fps.mjs ...
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE ?? 'playwright');
import { writeFileSync } from 'node:fs';

const [url, seconds = '20', out, shot] = process.argv.slice(2).filter(a => !a.startsWith('--'));
const headed = process.argv.includes('--headed');
const vision = process.argv.find(a => a.startsWith('--vision='))?.slice(9);
const size = (process.argv.find(a => a.startsWith('--size=')) ?? '--size=1600x900').slice(7).split('x').map(Number);
const browser = await chromium.launch({
  headless: !headed,
  args: ['--ignore-gpu-blocklist',
    '--disable-background-timer-throttling', '--disable-renderer-backgrounding'],
});
const page = await browser.newPage({ viewport: { width: size[0], height: size[1] } });
const logs = [];
page.on('console', m => logs.push(`${m.type()}: ${m.text()}`.slice(0, 300)));
page.on('pageerror', e => logs.push(`pageerror: ${e.message}`.slice(0, 300)));
await page.goto(url, { waitUntil: 'load', timeout: 60000 });
await page.waitForTimeout(6000);
if (vision) {
  await page.locator('#missionRobot').selectOption(vision);
  await page.locator('#missionOnboard').click();
  const panel = page.locator('.vision-panel');
  await panel.waitFor({ state: 'visible', timeout: 15000 });
  await panel.locator('[data-v="enable"]').check();
  await panel.locator('[data-v="confidence"]').check();
  await page.waitForFunction(() => document.querySelector('.vision-panel [data-v="pill"]')?.textContent === 'EXECUTANDO', null, { timeout: 30000 });
  await page.waitForTimeout(3000);
}
const renderer = await page.evaluate(() => {
  const c = document.createElement('canvas');
  const gl = c.getContext('webgl2') || c.getContext('webgl');
  if (!gl) return 'no webgl';
  const ext = gl.getExtension('WEBGL_debug_renderer_info');
  return ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
});
const result = await page.evaluate(async (ms) => new Promise(resolve => {
  const frames = [];
  const start = performance.now();
  const tick = (now) => {
    frames.push(now);
    if (now - start < ms) requestAnimationFrame(tick);
    else {
      const gaps = frames.slice(1).map((t, i) => t - frames[i]).sort((a, b) => a - b);
      const pick = q => gaps[Math.min(gaps.length - 1, Math.floor(q * (gaps.length - 1)))];
      resolve({ frames: frames.length, fps: (frames.length - 1) * 1000 / (frames.at(-1) - frames[0]),
        frame_ms_p50: pick(.5), frame_ms_p95: pick(.95), frame_ms_p99: pick(.99), frame_ms_max: gaps.at(-1),
        long_frames_over_50ms: gaps.filter(g => g > 50).length, long_frames_over_100ms: gaps.filter(g => g > 100).length,
        app_metrics: document.querySelector('#warehouse')?.dataset?.metrics ?? document.querySelector('[data-metrics]')?.dataset?.metrics ?? null,
        js_heap_mb: performance.memory ? performance.memory.usedJSHeapSize / 1e6 : null });
    }
  };
  requestAnimationFrame(tick);
}), Number(seconds) * 1000);
if (shot) await page.screenshot({ path: shot });
const visionPanel = vision ? { robot: vision, pill: await page.locator('.vision-panel [data-v="pill"]').innerText(),
  meta: await page.locator('.vision-panel [data-v="meta"]').innerText() } : null;
const report = { url, viewport: size, headed, renderer, vision: visionPanel, ...result, console: logs.slice(-20) };
writeFileSync(out, JSON.stringify(report, null, 1));
console.log(JSON.stringify({ renderer, fps: result.fps, p95: result.frame_ms_p95, long: result.long_frames_over_50ms }));
await browser.close();
