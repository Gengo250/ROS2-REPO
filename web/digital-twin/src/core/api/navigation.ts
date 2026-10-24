/** Independent read-only navigation extension; never modifies v1/v2 snapshots. */
export function observeNavigation(base: string, host: HTMLElement, signal: AbortSignal) {
  async function poll() {
    if (signal.aborted) return;
    try {
      const response=await fetch(`${base}/api/v1/navigation`,{signal:AbortSignal.any([signal,AbortSignal.timeout(2000)])});
      if(!response.ok)throw new Error('Navigation diagnostics unavailable');
      const data=await response.json();
      if(data.protocol_version!=='1.0'||data.message_type!=='navigation_status'||!Array.isArray(data.diagnostics))throw new Error('Unsupported diagnostics');
      const rows:HTMLElement[]=data.diagnostics.map((d:Record<string,unknown>)=>{
        const row=document.createElement('p');
        row.textContent=`${d.name==='navigation/interlock'?'Motion interlock':'Last navigation action'} · ${String(d.availability)} · ${String(d.message)}`;
        if(d.name==='navigation/action' && d.values && typeof d.values==='object'){
          const values=d.values as Record<string,string>;
          if(values.goal_map)row.textContent+=` · Goal (map): ${values.goal_map}`;
          try {const detail=JSON.parse(values.details||'{}');if(typeof detail.distance_remaining_m==='number')row.textContent+=` · Remaining: ${detail.distance_remaining_m.toFixed(2)} m`;}catch { /* Terminal details may be a ROS error code. */ }
        }
        return row;
      });
      if(rows.length)host.replaceChildren(...rows);else host.textContent='Navigation has not reported diagnostics.';
    } catch {if(!signal.aborted)host.textContent='Navigation diagnostics unavailable.';}
    if(!signal.aborted)setTimeout(()=>void poll(),1000);
  }
  void poll();
}
