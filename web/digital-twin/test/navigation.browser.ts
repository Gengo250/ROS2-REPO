import { test, expect } from '@playwright/test';
import { mkdir, writeFile, access } from 'node:fs/promises';
import { resolve } from 'node:path';

// Read-only observer: never publishes a command or changes an entity pose.
const evidence=process.env.NAV_EVIDENCE_DIR;
test.use({video: evidence ? {mode:'on',size:{width:1280,height:720}} : 'off'});
test('observes physical Nav2 navigation through the existing v2 scene',async({page})=>{
  test.skip(!evidence,'Requires the real owned navigation integration run');
  test.setTimeout(420000);
  await mkdir(evidence!,{recursive:true});
  const errors:string[]=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('/diagnostics.html');const host=page.locator('#warehouse');
  await expect(host).toHaveAttribute('data-loaded','true',{timeout:45000});
  await expect.poll(async()=>Object.keys(JSON.parse(await host.getAttribute('data-transforms')||'{}')).length).toBe(4);
  await page.getByRole('button',{name:'Robot focus',exact:true}).click();
  const samples:any[]=[];
  await writeFile(resolve(evidence!,'browser.ready'),'ready');
  try {
    const seconds=Number(process.env.NAV_OBSERVE_SECONDS||120);
    for(let i=0;i<seconds;i++){
      const s=await host.evaluate(e=>({wall:Date.now(),roots:JSON.parse(e.dataset.transforms!),visuals:JSON.parse(e.dataset.visualTransforms!),metrics:JSON.parse(e.dataset.metrics!)}));
      samples.push(s);
      if([0,Math.floor(seconds/2),seconds-1].includes(i))await page.screenshot({path:resolve(evidence!,`navigation-${i}.png`),fullPage:true});
      await page.waitForTimeout(1000);
      if(i>10 && await access(resolve(evidence!,'navigation.done')).then(()=>true,()=>false))break;
    }
    await page.screenshot({path:resolve(evidence!,'navigation-final.png'),fullPage:true});
    const displacement=Math.max(...samples.map(s=>Math.hypot(...s.roots.amr_01.position.map((v:number,i:number)=>v-samples[0].roots.amr_01.position[i]))));
    expect(displacement).toBeGreaterThan(.5);
    expect(errors).toEqual([]);
    expect(samples.at(-1).metrics.human_models).toBe(3);
    const final=await(await page.request.get('http://127.0.0.1:8000/api/v2/state')).json();
    const robot=final.simulation_robots[0].pose.position;
    // Match final browser to the world pipeline, allowing bounded interpolation delay.
    expect(Math.hypot(samples.at(-1).roots.amr_01.position[0]-robot.x,samples.at(-1).roots.amr_01.position[1]-robot.y)).toBeLessThan(.6);
  } finally {
    await writeFile(resolve(evidence!,'browser.json'),JSON.stringify({samples,errors},null,2));
    const video=page.video();await page.close();if(video)await video.saveAs(resolve(evidence!,'navigation.webm'));
  }
});
