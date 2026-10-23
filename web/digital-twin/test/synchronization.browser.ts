import { test, expect } from '@playwright/test';
import { mkdir, writeFile } from 'node:fs/promises';
import { resolve } from 'node:path';
import { execFile } from 'node:child_process';
import { promisify } from 'node:util';
const execute = promisify(execFile);
const evidence = process.env.TWIN_SYNC_EVIDENCE_DIR;
test.use({ video: process.env.TWIN_SYNC_VIDEO === '1' ? {mode:'on',size:{width:1280,height:720}} : 'off' });
test.afterEach(async ({page}, info) => {
  if (evidence && page.video()) {
    const video=page.video(); await page.close();
    await video!.saveAs(resolve(evidence, `${info.title.startsWith('actual AMR') ? 'amr' : info.title.startsWith('real workers') ? 'workers' : 'restart'}.webm`));
  }
});

test('actual AMR world pose drives the loaded GLB', async ({page}) => {
  test.skip(!evidence, 'Requires owned live Gazebo validation');
  await mkdir(evidence!, {recursive:true});
  const errors: string[]=[]; page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/');
  const host=page.locator('#warehouse');
  await expect(host).toHaveAttribute('data-loaded','true',{timeout:30000});
  await expect(page.locator('#scene-status')).toContainText('Gazebo ground truth');
  await expect.poll(async()=>JSON.parse(await host.getAttribute('data-transforms')||'{}').amr_01?.position[0]).toBeLessThan(-10);
  await page.locator('#warehouse canvas').screenshot({path:resolve(evidence!,'warehouse-canvas.png')});
  await execute('python3',['-c', 'from PIL import Image; import sys; im=Image.open(sys.argv[1]).convert("RGB"); pixels=list(im.getdata()); changed=sum(max(abs(c-b) for c,b in zip(p,(191,203,211)))>12 for p in pixels); assert changed/len(pixels)>.05, "Warehouse missing on first render"',resolve(evidence!,'warehouse-canvas.png')]);
  await page.getByRole('button',{name:'Robot focus',exact:true}).click();
  await page.waitForTimeout(1200);
  const before=JSON.parse((await host.getAttribute('data-transforms'))!);
  await page.screenshot({path:resolve(evidence!,'amr-initial.png'),fullPage:true});
  const samples:unknown[]=[];
  const motion=execute(resolve('../../.venv/bin/python'),[
    resolve('../../src/digital_twin_gateway/scripts/validate_motion.py'), '--output',resolve(evidence!,'motion.json'),'--angular','0.25'],{timeout:120000});
  for(let i=0;i<12;i++) { await page.waitForTimeout(1000); samples.push(JSON.parse((await host.getAttribute('data-transforms'))!)); }
  await motion;
  await page.waitForTimeout(1500);
  const after=JSON.parse((await host.getAttribute('data-transforms'))!);
  expect(after.amr_01.position[0]-before.amr_01.position[0]).toBeGreaterThan(.03);
  expect(Math.abs(after.amr_01.quaternion[2]-before.amr_01.quaternion[2])).toBeGreaterThan(.1);
  const ground=await (await page.request.get('http://127.0.0.1:8000/api/v2/state')).json();
  expect(after.amr_01.position[0]).toBeCloseTo(ground.simulation_robots[0].pose.position.x,2);
  await page.screenshot({path:resolve(evidence!,'amr-moved.png'),fullPage:true});
  expect(errors).toEqual([]);
  await writeFile(resolve(evidence!,'amr-browser.json'),JSON.stringify({before,after,samples,ground,errors},null,2));
});

test('real workers, visual gait, pause, reset and socket recovery', async ({page,context}) => {
  test.skip(!evidence, 'Requires owned live Gazebo validation');
  const errors:string[]=[];page.on('pageerror',e=>errors.push(e.message));
  const frames:any[]=[];
  page.on('websocket',socket=>socket.on('framereceived',event=>{
    const s=JSON.parse(String(event.payload)); if(s.protocol_version==='2.0') { frames.push({...s,browser_received_at:Date.now()}); if(frames.length>1000) frames.shift(); }
  }));
  await page.goto('/'); const host=page.locator('#warehouse');
  await expect(host).toHaveAttribute('data-loaded','true',{timeout:30000});
  await expect.poll(async()=>Object.keys(JSON.parse(await host.getAttribute('data-transforms')||'{}')).length).toBe(4);
  const samples:any[]=[];
  for(let i=0;i<18;i++) {
    await page.waitForTimeout(1000);
    // Read the completed render sample atomically. Separate protocol round trips
    // can straddle the next diagnostics tick and mix roots with different limbs.
    samples.push(await host.evaluate(element => ({
      ...JSON.parse(element.dataset.transforms!),
      visuals: JSON.parse(element.dataset.visualTransforms!),
      metrics: JSON.parse(element.dataset.metrics!), browser_wall: Date.now(),
    })));
    if([0,8,17].includes(i)) await page.screenshot({path:resolve(evidence!,`workers-${i}.png`),fullPage:true});
  }
  const distance=(a:number[],b:number[])=>Math.hypot(...a.map((v,i)=>v-b[i]!));
  for(const id of ['human_01','human_02']) {
    expect(Math.max(...samples.map(s=>distance(s[id].position,samples[0][id].position)))).toBeGreaterThan(.05);
    const joints=frames.map(s=>s.humans.find((h:any)=>h.id===id)?.components.find((c:any)=>c.id.endsWith('/leg_1_visual'))?.pose.orientation.y).filter(v=>v!==undefined);
    expect(Math.max(...joints)-Math.min(...joints)).toBeGreaterThan(.03);
  }
  expect(Math.max(...samples.map(s=>distance(s.human_03.position,samples[0].human_03.position)))).toBeLessThan(1e-9);
  const graphics=await page.locator('#warehouse canvas').evaluate(canvas=>{
    const gl=(canvas as HTMLCanvasElement).getContext('webgl2')!;
    const ext=gl.getExtension('WEBGL_debug_renderer_info');
    return {renderer:ext?gl.getParameter(ext.UNMASKED_RENDERER_WEBGL):null,version:gl.getParameter(gl.VERSION)};
  });
  const metrics=JSON.parse((await host.getAttribute('data-metrics'))!);
  expect(metrics.human_models).toBe(3); expect(metrics.human_visuals).toBe(30);
  const control=async(req:string)=>execute('gz',['service','-s','/world/warehouse/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','5000','--req',req],{timeout:15000});
  const posesOnly=(state:Record<string,any>)=>Object.fromEntries(Object.entries(state).map(
    ([id,{status: _status,...pose}])=>[id,pose]));
  const readTransforms=()=>host.evaluate(element=>({
    roots:JSON.parse(element.dataset.transforms!),visuals:JSON.parse(element.dataset.visualTransforms!),
  }));
  let pausedBefore:any, pausedAfter:any;
  try {
    expect((await control('pause: true')).stdout).toContain('data: true');
    await page.waitForTimeout(3000);
    pausedBefore=await readTransforms();
    await page.waitForTimeout(2500);
    pausedAfter=await readTransforms();
    // Availability legitimately ages from online to stale while physics is
    // paused. Physical transforms, including gait, must stay exactly frozen.
    expect(posesOnly(pausedAfter.roots)).toEqual(posesOnly(pausedBefore.roots));
    expect(pausedAfter.visuals).toEqual(pausedBefore.visuals);
    await page.screenshot({path:resolve(evidence!,'paused.png'),fullPage:true});
  } finally { await control('pause: false'); }
  await expect.poll(()=>frames.at(-1)?.simulation.status,{timeout:15000}).toBe('running');
  const oldEpoch=frames.at(-1).simulation.epoch;
  expect((await control('reset: {all: true}')).stdout).toContain('data: true');
  await expect.poll(()=>frames.at(-1)?.simulation.epoch,{timeout:30000}).toBeGreaterThan(oldEpoch);
  // Gazebo full reset removes the robot inserted by ros_gz_sim create.
  // Keep its last pose visibly unavailable; never invent a respawn.
  await expect.poll(()=>frames.at(-1)?.simulation_robots[0]?.status,{timeout:20000}).toBe('offline');
  await expect.poll(()=>frames.at(-1)?.humans[0]?.status,{timeout:15000}).toBe('online');
  await context.setOffline(true);
  await expect(page.locator('#gateway')).not.toHaveText('Connected',{timeout:15000});
  await page.waitForTimeout(1200);
  const disconnected=JSON.parse((await host.getAttribute('data-transforms'))!);
  await page.waitForTimeout(1500);
  expect(JSON.parse((await host.getAttribute('data-transforms'))!)).toEqual(disconnected);
  await context.setOffline(false);
  await expect(page.locator('#gateway')).toHaveText('Connected',{timeout:30000});
  await expect.poll(()=>frames.at(-1)?.simulation.status,{timeout:30000}).toBe('running');
  expect(errors).toEqual([]);
  await writeFile(resolve(evidence!,'workers-browser.json'),JSON.stringify({samples,frames,metrics,graphics,pausedBefore,pausedAfter,errors},null,2));
});


test('browser recovers when the actual simulator is stopped and restarted', async ({page}) => {
  test.skip(!evidence || !process.env.TWIN_SYNC_RESTART, 'Requires the owning restart harness');
  await page.goto('/');
  const host=page.locator('#warehouse');
  await expect(host).toHaveAttribute('data-loaded','true',{timeout:30000});
  const state=async()=> (await page.request.get('http://127.0.0.1:8000/api/v2/state')).json();
  const before=await state();
  await writeFile(resolve(evidence!,'restart.request'),'restart owned simulator');
  await expect.poll(async()=>(await state()).humans[0].status,{timeout:45000}).toBe('offline');
  await page.waitForTimeout(1300);
  const stopped=JSON.parse((await host.getAttribute('data-transforms'))!);
  await expect.poll(async()=>(await state()).simulation_robots[0].status,{timeout:120000}).toBe('online');
  await expect.poll(async()=>JSON.parse(await host.getAttribute('data-transforms')||'{}').amr_01?.position[0],{timeout:30000}).toBeCloseTo(-11,2);
  const after=await state();
  expect(after.session_id).toBe(before.session_id);
  expect(after.simulation.epoch).toBeGreaterThan(before.simulation.epoch);
  await page.screenshot({path:resolve(evidence!,'restarted.png'),fullPage:true});
  await writeFile(resolve(evidence!,'restart-browser.json'),JSON.stringify({before,stopped,after,transforms:JSON.parse((await host.getAttribute('data-transforms'))!)},null,2));
});
