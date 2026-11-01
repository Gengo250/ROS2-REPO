// DEMO mode: the ATLAS prototype's browser-side traffic, workers and loose objects, kept as
// the prototype wrote them. Static geometry, lanes and the crew come from the compiled campus
// scene (converted back to the prototype's WebGL x/z axes). The caller advances step(dt) on
// a fixed-step timer, independent of rendering. This simulation never runs in LIVE ROS mode.
'use strict';
import {TAU,clamp,smooth,wrapA,LT,LS,drawHuman,drawHumanFar,drawRobot,drawForklift,drawFields,drawLooseObject,drawFlash,drawPlacement,animateGait} from '../rig/atlasRig.js';

const VSPEC={amr:{L:1.64,W:1.3,cx:0,accel:.9,brake:1.05,emerg:2.8,aLat:.75,gap:.45,k:.7,mass:260,name:'Robô móvel autônomo'},
 lift:{L:4.55,W:1.98,cx:.8,accel:.55,brake:.85,emerg:2.3,aLat:.5,gap:.7,k:1.1,mass:3800,name:'Empilhadeira'}};
const FLEET=[['west','amr',1.85],['west','amr',1.7],['east','amr',1.9],['east','amr',1.75],['north','amr',2.0],['front','amr',1.6],['dock','amr',1.55],['dock','amr',1.65],['north','lift',1.45],['front','lift',1.3],['dock','lift',1.35]];
const LOOSE={cone:{w:.42,d:.42,mass:4,mu:2.2,label:'Cone',g:'o',art:'um'},box:{w:.62,d:.52,mass:18,mu:3.2,label:'Caixa',g:'a',art:'uma'},pallet:{w:1.22,d:1.02,mass:420,mu:4.5,label:'Palete',g:'o',art:'um'},parcel:{w:.4,d:.38,mass:9,mu:3,label:'Volume',g:'o',art:'um'}};
export const DEMO_LOOSE=LOOSE;

export function createDemo(scene,{clock=()=> '00:00',toast=()=>{}}={}){
 let seed=1137;const rand=()=>((seed=(Math.imul(1664525,seed)+1013904223)>>>0)/4294967296);const rr=(a,b)=>a+(b-a)*rand();
 const ACTS=scene.agents.acts,ROLES=scene.agents.roles;
 const opts={people:true,vehicles:true};
 let t=0;
 // ---------- OBB no plano XZ (eixo local x = frente) ----------
 function setOBB(o,x,z,yaw){const c=Math.cos(yaw),s=Math.sin(yaw);o.x=x;o.z=z;o.ux=c;o.uz=-s;o.vx=s;o.vz=c;return o}
 function makeOBB(x,z,hw,hd,yaw=0,tag=''){return setOBB({x:0,z:0,hw,hd,ux:1,uz:0,vx:0,vz:1,r:Math.hypot(hw,hd),tag},x,z,yaw)}
 const projR=(O,nx,nz)=>O.hw*Math.abs(O.ux*nx+O.uz*nz)+O.hd*Math.abs(O.vx*nx+O.vz*nz);
 function obbHit(A,B,m=0){const dx=B.x-A.x,dz=B.z-A.z,lim=A.r+B.r+m;if(dx*dx+dz*dz>lim*lim)return false;
  for(let k=0;k<4;k++){const O=k<2?A:B,nx=k%2?O.vx:O.ux,nz=k%2?O.vz:O.uz;if(Math.abs(dx*nx+dz*nz)>projR(A,nx,nz)+projR(B,nx,nz)+m)return false}return true}
 function obbMTV(A,B){const dx=B.x-A.x,dz=B.z-A.z,lim=A.r+B.r;if(dx*dx+dz*dz>lim*lim)return null;let best=1e9,bx=0,bz=0;
  for(let k=0;k<4;k++){const O=k<2?A:B,nx=k%2?O.vx:O.ux,nz=k%2?O.vz:O.uz,p=dx*nx+dz*nz,o=projR(A,nx,nz)+projR(B,nx,nz)-Math.abs(p);if(o<=0)return null;if(o<best){best=o;const g=p<0?-1:1;bx=nx*g;bz=nz*g}}return {nx:bx,nz:bz,depth:best}}
 function circHit(O,x,z,r){const dx=x-O.x,dz=z-O.z,lx=dx*O.ux+dz*O.uz,lz=dx*O.vx+dz*O.vz,cx=Math.max(-O.hw,Math.min(O.hw,lx)),cz=Math.max(-O.hd,Math.min(O.hd,lz));return (lx-cx)**2+(lz-cz)**2<r*r}
 // Coarse colliders of the prototype, from the scene (world y → WebGL z = -y).
 const solids=scene.solids.map(s=>makeOBB(s.x,-s.y,s.length/2,s.width/2,s.yaw,s.tag));
 const LOOPS=Object.fromEntries(Object.entries(scene.lanes).map(([id,l])=>[id,{pts:l.points.map(([x,y])=>[x,-y]),color:l.colour}]));
 const robots=[],forklifts=[],workers=[],loose=[],flashes=[];
 const traffic={stops:0,yields:0,detours:0,hits:0,log:[]},hitMemo=new Map();
 let sensorsOn=true,looseSeq=0;
 const vehicles=()=>opts.vehicles?robots.concat(forklifts):robots;
 function track(points){let lengths=[],total=0;for(let j=0;j<points.length;j++){const a=points[j],b=points[(j+1)%points.length],d=Math.hypot(b[0]-a[0],b[1]-a[1]);lengths.push(d);total+=d}return {points,lengths,total}}
 function pathAt(tr,distance){let d=((distance%tr.total)+tr.total)%tr.total;for(let j=0;j<tr.points.length;j++){let len=tr.lengths[j];if(d<=len){let a=tr.points[j],b=tr.points[(j+1)%tr.points.length],u=Math.min(1,d/Math.max(len,.001));return {x:a[0]+(b[0]-a[0])*u,z:a[1]+(b[1]-a[1])*u,yaw:Math.atan2(-(b[1]-a[1]),b[0]-a[0])}}d-=len}return {x:tr.points[0][0],z:tr.points[0][1],yaw:0}}
 function offAt(r,D){const tt=r.detour;if(!tt)return 0;return tt.o*Math.min(smooth(tt.d0,tt.d0+tt.ramp,D),1-smooth(tt.d1,tt.d1+tt.ramp,D))}
 function vPoseAt(r,D){const k=r.S.k,a=pathAt(r.track,D-k),b=pathAt(r.track,D+k),y=Math.atan2(-(b.z-a.z),b.x-a.x),c=Math.cos(y),s=Math.sin(y),o=offAt(r,D);
  let yaw=y;if(r.detour){const dO=(offAt(r,D+.3)-offAt(r,D-.3))/.6;yaw=Math.atan2(s-c*dO,c+s*dO)}return {x:(a.x+b.x)/2+s*o,z:(a.z+b.z)/2+c*o,yaw}}
 function footprint(r,p,o){const c=Math.cos(p.yaw),s=Math.sin(p.yaw),S=r.S;return o?setOBB(o,p.x+c*S.cx,p.z-s*S.cx,p.yaw):makeOBB(p.x+c*S.cx,p.z-s*S.cx,S.L/2,S.W/2,p.yaw)}
 function makeFleet(){
  for(const L of Object.values(LOOPS))L.track=track(L.pts);
  const perLoop={};FLEET.forEach(f=>perLoop[f[0]]=(perLoop[f[0]]||0)+1);const seen={};let na=0,nl=0;
  FLEET.forEach(([loop,kind,sp],rank)=>{const L=LOOPS[loop],k=seen[loop]=(seen[loop]||0)+1,lift=kind==='lift';
   const r={id:lift?`EMP-${++nl}`:`AMR-${String(++na).padStart(2,'0')}`,kind,lift,S:VSPEC[kind],loop,track:L.track,speed:sp,v:0,d:L.track.total*(k-1)/perLoop[loop]+rank*3.1,color:lift?['#f4b64f','#e9a443','#e6c063'][nl-1]:L.color,rank,detour:null,samples:[],prot:.5,look:3,field:0,why:null,ref:null,yieldTo:null,waitT:0,emergT:0,boostT:0,touch:new Set(),odo:0,stopFlag:false,nextPlan:0,detourFail:false,lastLog:-99};
   r.pose=vPoseAt(r,r.d);r.obb=footprint(r,r.pose);
   for(let tries=0;tries<300&&[...robots,...forklifts].some(q=>obbHit(q.obb,r.obb,.9));tries++){r.d+=1.5;r.pose=vPoseAt(r,r.d);footprint(r,r.pose,r.obb)}
   (lift?forklifts:robots).push(r)})}
 function planSamples(r){const S=r.S,prot=r.v*r.v/(2*S.brake)+S.gap,look=Math.max(prot+2.4,3.4),out=[];r.prot=prot;r.look=look;
  for(let s=0;s<=look+1e-6;s+=.35){const p=vPoseAt(r,r.d+s);out.push({s,p,o:footprint(r,p)})}r.samples=out}
 function sweepHit(smp,list,kind,m,best,getO){for(const ref of list){const o=getO(ref);if(!o)continue;for(let i=0;i<smp.length&&(!best||i<best.i);i++)if(obbHit(smp[i].o,o,m)){best={i,kind,ref};break}}return best}
 function scanHazards(r,list){const smp=r.samples,px=r.pose.x,pz=r.pose.z,R=r.look+r.S.L+2,near=o=>Math.abs(o.x-px)<R+o.r&&Math.abs(o.z-pz)<R+o.r;let best=null;
  best=sweepHit(smp,solids.filter(near),'static',.08,best,o=>o);
  if(sensorsOn)best=sweepHit(smp,loose.filter(b=>near(b.obb)),'obj',.25,best,b=>b.obb);
  best=sweepHit(smp,list.filter(q=>q!==r&&near(q.obb)),'veh',.3,best,q=>q.obb);
  if(opts.people)for(const w of workers){if(w.y0>1||Math.abs(w.x-px)>R||Math.abs(w.z-pz)>R)continue;const ax=w.x+Math.cos(w.yaw)*w.v*.8,az=w.z-Math.sin(w.yaw)*w.v*.8;
   for(let i=0;i<smp.length&&(!best||i<best.i);i++)if(circHit(smp[i].o,w.x,w.z,.75)||(w.v>.1&&circHit(smp[i].o,ax,az,.6))){best={i,kind:'person',ref:w};break}}
  return best}
 function futureConflict(r,q){const A=r.samples,B=q.samples;for(let i=1;i<A.length;i++)for(let j=1;j<B.length;j++)if(obbHit(A[i].o,B[j].o,.15))return {i,j};return null}
 const prioOf=r=>(r.boostT>0?100:0)+(r.lift?2:1)+r.rank*.001;
 function logEvent(msg){traffic.log.unshift({time:clock().slice(0,5),msg});traffic.log.length=Math.min(traffic.log.length,5)}
 const withArt=label=>{const L=Object.values(LOOSE).find(q=>q.label===label);return L?`${L.art} ${label.toLowerCase()}`:'um obstáculo'};
 function rearClear(r,dist){for(let s=-.35;s>=-dist-1e-6;s-=.35){const f=footprint(r,vPoseAt(r,r.d+s));for(const so of solids)if(obbHit(f,so,.05))return false;for(const b of loose)if(obbHit(f,b.obb,.15))return false;for(const q of vehicles())if(q!==r&&obbHit(f,q.obb,.15))return false;for(const w of workers)if(w.y0<1&&circHit(f,w.x,w.z,.6))return false}return true}
 function backoffStep(r,dt){const b=r.backoff;r.why='backoff';r.ref=b.from;r.field=1;r.yieldTo=b.from;
  if(b.left>0&&rearClear(r,.7)){r.v=Math.max(r.v-1.5*dt,-.4);b.left+=r.v*dt}else{r.v=Math.min(0,r.v+2*dt);b.left=Math.min(b.left,0);b.hold-=dt}
  if(b.hold<=0)r.backoff=null}
 function decide(r,list,dt){const S=r.S;let lim=Infinity,why=null,ref=null;if(r.backoff){backoffStep(r,dt);return}
  if(r.emergT>0){r.emergT-=dt;lim=0;why='emerg'}
  const h=scanHazards(r,list);if(h){const s=r.samples[h.i].s;if(s<lim){lim=s;why=h.kind;ref=h.ref}}
  let yTo=null,ylim=Infinity;
  for(const q of list){if(q===r||!q.samples.length||q.yieldTo===r)continue;const dx=q.pose.x-r.pose.x,dz=q.pose.z-r.pose.z,R=r.look+q.look+6;if(dx*dx+dz*dz>R*R)continue;
   const fc=futureConflict(r,q);if(!fc)continue;const sA=r.samples[fc.i].s,sB=q.samples[fc.j].s,tA=sA/Math.max(r.v,.5),tB=(q.v<.05&&q.waitT>.6)?Infinity:sB/Math.max(q.v,.5);
   let y=tA>tB+.35||(Math.abs(tA-tB)<=.35&&prioOf(r)<prioOf(q));if(r.yieldTo===q&&tB<Infinity&&prioOf(r)<100)y=true;
   if(y){const l=Math.max(0,sA-.45);if(l<ylim){ylim=l;yTo=q}}}
  if(yTo&&ylim<lim){lim=ylim;why='yield';ref=yTo}
  if((why==='yield'||why==='veh')&&ref&&Math.cos(wrapA(ref.pose.yaw-r.pose.yaw))>.8&&ref.v>.05)why='follow';
  if(yTo&&r.yieldTo!==yTo&&r.v>.3){traffic.yields++;if(t-r.lastLog>10){r.lastLog=t;logEvent(`${r.id} cedeu passagem para ${yTo.id}`)}}
  r.yieldTo=yTo;
  let vt=r.speed;r.curve=false;const sm=r.samples;
  for(let i=1;i<sm.length;i++){const k=Math.abs(wrapA(sm[i].p.yaw-sm[i-1].p.yaw))/.35;if(k>.08){const vc=Math.sqrt(S.aLat/k),al=Math.sqrt(vc*vc+2*S.brake*sm[i-1].s);if(al<vt){vt=al;r.curve=true}}}
  if(lim<Infinity){const av=lim-(why==='yield'?0:S.gap);const vs=av>0?Math.sqrt(2*S.brake*av):0;if(vs<vt)vt=vs}
  r.pushing=false;r.pinned=false;if(r.touch.size){let m=0;for(const ob of loose)if(r.touch.has(ob.id)){m+=ob.L.mass;if(ob.pinned)r.pinned=true}r.pushing=true;vt=Math.min(vt,r.pinned?0:r.speed*S.mass/(S.mass+m))}
  const dv=vt-r.v;r.v+=dv>0?Math.min(dv,S.accel*dt):Math.max(dv,-S.emerg*(why==='emerg'?2:1)*dt);if(r.v<0)r.v=0;
  const stopped=r.v<.06;r.waitT=stopped&&lim<Infinity?r.waitT+dt:0;
  if(stopped&&!r.stopFlag&&(why==='person'||why==='obj'||why==='static'||why==='veh')){r.stopFlag=true;traffic.stops++;if(why==='person')logEvent(`${r.id} parou para ${ref.id} passar`);if(why==='obj')logEvent(`${r.id} detectou ${withArt(ref.L.label)} na rota`)}
  if(r.v>.4)r.stopFlag=false;
  if(r.waitT>12&&r.boostT<=0)r.boostT=5;r.boostT-=dt;
  if(why==='veh'&&ref&&r.waitT>1.5&&(ref.why==='veh'&&ref.ref===r&&prioOf(r)<prioOf(ref)||r.waitT>9&&(ref.why==='veh'||ref.why==='yield')&&ref.waitT>3)&&rearClear(r,1.2)){r.backoff={left:2.4,hold:2.6,from:ref};r.waitT=0;logEvent(`${r.id} recuou para liberar ${ref.id}`)}
  if(why==='veh'&&ref&&!r.backoff&&ref.why==='obj'&&ref.detourFail&&ref.waitT>2&&r.waitT>1.5&&rearClear(r,1.2)){r.backoff={left:r.lift?3:2.5,hold:4,from:ref};r.waitT=0;logEvent(`${r.id} recuou para abrir espaço a ${ref.id}`)}
  r.nextPlan-=dt;if(r.v>.4||r.waitT>20&&r.replans>=3)r.replans=0;if((why==='obj'||why==='static')&&r.detour&&r.waitT>1.5){r.detour=null;r.nextPlan=0}if((why==='obj'||why==='static')&&!r.detour&&r.nextPlan<=0&&(r.waitT>.8||lim>(r.lift?4.2:3)+.6)){r.nextPlan=1.2;r.detourFail=!planDetour(r,lim,ref.obb||ref);
   if(r.detourFail&&lim<2.2&&(r.replans||0)<3&&rearClear(r,2.2)){r.replans=(r.replans||0)+1;r.backoff={left:r.lift?2.6:2,hold:.4,from:null};r.nextPlan=0;logEvent(`${r.id} recuou para replanejar o desvio`)}}
  if(!r.detour&&why!=='obj')r.detourFail=false;
  r.why=why;r.ref=ref;
  r.field=(why==='emerg'||r.pinned||(lim<Infinity&&lim<=r.prot+.05&&why!=='follow'&&why!=='yield'))?2:(lim<Infinity||r.curve&&r.v<r.speed*.6)?1:0}
 function planDetour(r,hitS,ob){const S=r.S,base=r.d,ramp=clamp(hitS-.2,r.lift?2.6:1.8,r.lift?4.5:3),d1=base+hitS+2*ob.r+S.L+.6,keep=r.detour;
  for(const o of r.lift?[1.6,-1.6,2.05,-2.05,2.7,-2.7]:[1.2,-1.2,1.5,-1.5,2.15,-2.15]){r.detour={d0:base,d1,o,ramp};let ok=true;
   for(let s=0;s<=d1+ramp-base&&ok;s+=.18){const f=footprint(r,vPoseAt(r,base+s));
    for(const so of solids)if(obbHit(f,so,.15)){ok=false;break}
    if(ok)for(const b of loose)if(obbHit(f,b.obb,.32)){ok=false;break}
    if(ok&&s<10)for(const q of vehicles()){if(q===r)continue;if(obbHit(f,q.obb,.25)){ok=false;break}for(const qs of q.samples)if(obbHit(f,qs.o,.2)){ok=false;break}if(!ok)break}}
   if(ok){traffic.detours++;logEvent(`${r.id} contornou ${withArt(ob.tag)}`);return true}}
  r.detour=keep;return false}
 function spawnLoose(type,x,z,yaw=0,opt={}){const L=LOOSE[type],ob={id:++looseSeq,type,L,w:L.w,d:L.d,x,z,yaw,vx:0,vz:0,wv:0,tip:0,tipped:false,pinned:false,ttl:opt.ttl||0,user:!!opt.user,obb:makeOBB(x,z,L.w/2,L.d/2,yaw,L.label)};loose.push(ob);return ob}
 function pushLoose(r,ob,m){ob.x+=m.nx*(m.depth+.004);ob.z+=m.nz*(m.depth+.004);const c=Math.cos(r.pose.yaw),s=Math.sin(r.pose.yaw),vn=(c*r.v)*m.nx+(-s*r.v)*m.nz,on=ob.vx*m.nx+ob.vz*m.nz;
  if(vn>on){const add=vn*1.12-on;ob.vx+=m.nx*add;ob.vz+=m.nz*add;const rx=ob.x-r.obb.x,rz=ob.z-r.obb.z;ob.wv+=clamp((rx*m.nz-rz*m.nx)*add*(ob.L.mass>100?.15:.9),-3,3)}
  if(ob.type==='cone'&&!ob.tipped&&vn>.25){ob.tipped=true;ob.yaw=Math.atan2(-m.nz,m.nx);ob.obb.hw=.36;ob.obb.hd=.2;ob.obb.r=Math.hypot(.36,.2)}
  setOBB(ob.obb,ob.x,ob.z,ob.yaw)}
 function updateLoose(dt){for(let i=loose.length-1;i>=0;i--){const ob=loose[i];if(ob.ttl){ob.ttl-=dt;if(ob.ttl<=0){loose.splice(i,1);continue}}
   const sp=Math.hypot(ob.vx,ob.vz),dec=ob.L.mu*dt;if(sp<=dec){ob.vx=0;ob.vz=0}else{ob.vx*=(sp-dec)/sp;ob.vz*=(sp-dec)/sp}
   ob.wv-=Math.sign(ob.wv)*Math.min(Math.abs(ob.wv),3.5*dt);ob.x+=ob.vx*dt;ob.z+=ob.vz*dt;ob.yaw+=ob.wv*dt;if(ob.tipped)ob.tip=Math.min(Math.PI/2,ob.tip+dt*6);
   setOBB(ob.obb,ob.x,ob.z,ob.yaw);ob.pinned=false;
   for(const so of solids){if(Math.abs(so.x-ob.x)>so.r+ob.obb.r||Math.abs(so.z-ob.z)>so.r+ob.obb.r)continue;const m=obbMTV(so,ob.obb);if(!m)continue;
    ob.x+=m.nx*m.depth;ob.z+=m.nz*m.depth;const vn=ob.vx*m.nx+ob.vz*m.nz;if(vn<0){ob.vx-=1.3*vn*m.nx;ob.vz-=1.3*vn*m.nz}ob.wv*=.6;ob.pinned=true;setOBB(ob.obb,ob.x,ob.z,ob.yaw)}}
  for(let i=0;i<loose.length;i++)for(let j=i+1;j<loose.length;j++){const a=loose[i],b=loose[j],m=obbMTV(a.obb,b.obb);if(!m)continue;
   const ia=1/a.L.mass,ib=1/b.L.mass,k=m.depth/(ia+ib);a.x-=m.nx*k*ia;a.z-=m.nz*k*ia;b.x+=m.nx*k*ib;b.z+=m.nz*k*ib;
   const rv=(b.vx-a.vx)*m.nx+(b.vz-a.vz)*m.nz;if(rv<0){const J=-1.2*rv/(ia+ib);a.vx-=J*ia*m.nx;a.vz-=J*ia*m.nz;b.vx+=J*ib*m.nx;b.vz+=J*ib*m.nz}
   setOBB(a.obb,a.x,a.z,a.yaw);setOBB(b.obb,b.x,b.z,b.yaw)}}
 function onCollision(r,ob){traffic.hits++;flashes.push({x:(r.obb.x+ob.x)/2,z:(r.obb.z+ob.z)/2,t:0});logEvent(`Colisão: ${r.id} atingiu ${withArt(ob.L.label)}`);toast(`Colisão: ${r.id} atingiu ${withArt(ob.L.label)}`);if(sensorsOn)r.emergT=2}
 function resolveContacts(list){for(const r of list){const now=new Set();for(const ob of loose){const m=obbMTV(r.obb,ob.obb);if(!m)continue;now.add(ob.id);pushLoose(r,ob,m);if(!r.touch.has(ob.id)){const k=r.id+'>'+ob.id,last=hitMemo.get(k);hitMemo.set(k,t);if(last===undefined||t-last>4)onCollision(r,ob)}}r.touch=now}
  for(let i=0;i<list.length;i++)for(let j=i+1;j<list.length;j++){const a=list[i],b=list[j];if(obbHit(a.obb,b.obb,0)){const key=a.id+b.id;if(!a.vhit||a.vhit!==key){a.vhit=key;traffic.hits++;logEvent(`Contato entre ${a.id} e ${b.id}`);a.emergT=b.emergT=1.5}}else if(a.vhit===a.id+b.id)a.vhit=null}}
 function trafficStep(dt){if(dt<=0)return;const n=Math.ceil(dt/.034),h=dt/n,list=vehicles();
  for(let k=0;k<n;k++){for(const r of list)planSamples(r);for(const r of list)decide(r,list,h);
   for(const r of list){r.d+=r.v*h;r.odo+=Math.abs(r.v)*h;r.pose=vPoseAt(r,r.d);footprint(r,r.pose,r.obb);if(r.detour&&r.d>r.detour.d1+r.detour.ramp)r.detour=null}
   resolveContacts(list);updateLoose(h)}
  for(let i=flashes.length-1;i>=0;i--)if((flashes[i].t+=dt)>.9)flashes.splice(i,1)}
 // ---------- Pessoas: comportamento ----------
 function arriveEnd(w){const e=w.wp===0?0:1;let q=w.acts[e].filter(tp=>!(tp==='pick'&&w.carry)&&!(tp==='drop'&&!w.carry));if(!q.length)q=['idle'];
  const f=w.face[e];let face=w.yaw,goal=[w.x,w.z];
  if(f==='side'){face=Math.round((w.yaw+w.sideSign*Math.PI/2)/(Math.PI/2))*(Math.PI/2);w.sideSign*=-1;goal=[w.x+Math.cos(face)*.72,w.z-Math.sin(face)*.72]}else if(typeof f==='number')face=f;
  w.queue=q;w.actFace=face;w.actGoal=goal;startAct(w)}
 function startAct(w){const type=w.queue.shift();w.act={type,t:0,dur:ACTS[type].dur*rr(.85,1.2),ready:false,done:false}}
 function turnToward(w,target,dt,rate){const err=wrapA(target-w.yaw);w.yaw=wrapA(w.yaw+clamp(err*Math.min(1,dt*7),-rate*dt,rate*dt));return err}
 function updateWorker(w,dt,time){if(dt<=0)return;const prevYaw=w.yaw;let lookT=0;w.waiting=false;
  if(w.act){const act=w.act;
   if(!act.ready){const dx=w.actGoal[0]-w.x,dz=w.actGoal[1]-w.z,d=Math.hypot(dx,dz);if(d>.04){const sp=Math.min(.75,d*2.6);w.x+=dx/d*sp*dt;w.z+=dz/d*sp*dt;w.v=sp}else w.v=Math.max(0,w.v-3*dt);
    const err=turnToward(w,w.actFace,dt,3.4);if(d<.06&&Math.abs(err)<.1)act.ready=true}
   else{act.t+=dt;w.v=Math.max(0,w.v-3*dt);turnToward(w,w.actFace,dt,1.5);
    if(!act.done&&act.t>act.dur*.5){act.done=true;if(act.type==='pick')w.carry=true;if(act.type==='drop'){w.carry=false;spawnLoose('parcel',w.x+Math.cos(w.yaw)*.5,w.z-Math.sin(w.yaw)*.5,w.yaw,{ttl:28})}}
    if(act.t>=act.dur){if(w.queue.length)startAct(w);else{w.act=null;if(w.wp===w.path.length-1){w.dir=-1;w.wp--}else{w.dir=1;w.wp++}}}}
   lookT=0}
  else{const end=w.wp===0||w.wp===w.path.length-1,T=w.path[w.wp],Pv=w.path[w.wp-w.dir],sl=Math.hypot(T[0]-Pv[0],T[1]-Pv[1])||1,px=-(T[1]-Pv[1])/sl,pz=(T[0]-Pv[0])/sl;
   const tx=T[0]+px*w.side,tz=T[1]+pz*w.side,dx=tx-w.x,dz=tz-w.z,dist=Math.hypot(dx,dz),hd=Math.atan2(-dz,dx);
   const err=turnToward(w,hd,dt,3.4),fx=Math.cos(w.yaw),fz=-Math.sin(w.yaw);
   let vt=w.vmax*(w.carry?.85:1)*clamp(Math.cos(err)*1.25,.08,1);vt=end?Math.min(vt,.2+dist*.8):Math.min(vt,w.vmax*.62+dist*.35);
   let sideT=0,threat=null;
   if(w.ignore<=0&&w.y0<1)for(const vh of vehicles()){const p=vh.pose,rx=p.x-w.x,rz=p.z-w.z,d=Math.hypot(rx,rz);if(d>8)continue;const len=vh.lift?1.9:1.1,al=rx*fx+rz*fz,lat=Math.abs(-rx*fz+rz*fx);
    if(vh.v>.25){if(al>-.3&&al<4.2+len&&lat<1.3+len*.4){w.waiting=true;threat=p}
     const vfx=Math.cos(p.yaw),vfz=-Math.sin(p.yaw),toMe=(-rx*vfx-rz*vfz)/(d||1),latV=Math.abs(rx*vfz-rz*vfx);if(d<5+len&&toMe>.72&&latV<1.5){w.waiting=true;threat=p}}
    else if(al>0&&al<1.7+len&&lat<1.1+len*.3)sideT=1.5}
   for(const o of workers){if(o===w||Math.abs(o.y0-w.y0)>1)continue;const rx=o.x-w.x,rz=o.z-w.z;if(Math.abs(rx)>2.2||Math.abs(rz)>2.2)continue;const al=rx*fx+rz*fz,lat=Math.abs(-rx*fz+rz*fx);if(al>0&&al<1.6&&lat<.75){sideT=Math.max(sideT,.8);vt*=.7}}
   if(w.y0<1)for(const b of loose){const rx=b.x-w.x,rz=b.z-w.z;if(Math.abs(rx)>3||Math.abs(rz)>3)continue;const al=rx*fx+rz*fz,lat=Math.abs(-rx*fz+rz*fx);if(al>-.2&&al<1.8&&lat<b.obb.r+.45)sideT=Math.max(sideT,b.obb.r+.8)}
   if(w.waiting){vt=0;w.waitT+=dt;if(w.waitT>6){w.ignore=3;w.waitT=0}}else w.waitT=Math.max(0,w.waitT-dt*2);
   w.side+=(sideT-w.side)*(1-Math.exp(-dt*(sideT>w.side?2.6:.7)));
   w.v+=clamp(vt-w.v,-3.6*dt,1.5*dt);w.x+=fx*w.v*dt;w.z+=fz*w.v*dt;
   if(dist<(end?.25:.8)){if(end)arriveEnd(w);else w.wp+=w.dir}
   lookT=threat?clamp(wrapA(Math.atan2(-(threat.z-w.z),threat.x-w.x)-w.yaw),-1.3,1.3):clamp(err*.75,-.9,.9)+.4*Math.sin(time*.31+w.seed)*Math.sin(time*.17+w.seed*2)}
  w.ignore-=dt;
  animateGait(w,dt,wrapA(w.yaw-prevYaw)/dt);
  w.look+=(lookT-w.look)*(1-Math.exp(-dt*4))}
 function makeWorkers(){for(const c of scene.agents.workers){const R=ROLES[c.role],a=c.appearance;
   workers.push({id:c.id,role:c.role,R,path:c.path.map(([x,y])=>[x,-y]),acts:c.acts,face:c.face,y0:c.level_z,x:c.initial.x,z:-c.initial.y,yaw:c.initial.yaw,wp:c.initial.waypoint,dir:c.initial.direction,v:0,vmax:c.max_speed,phase:a.phase,gaitW:0,A:.3,look:0,side:0,waitT:0,ignore:0,waiting:false,act:null,queue:[],carry:c.carry,hold:c.hold,seed:a.seed,sideSign:a.side_sign,skin:a.skin,pants:a.pants,shirt:a.shirt,hair:a.hair,steps:0})}}
 makeFleet();makeWorkers();
 // ---------- ferramenta de objetos ----------
 const interior=scene.building.interior;
 function placementCheck(type,x,z){if(x<interior.x_min+.8||x>interior.x_max-.8||-z<interior.y_min+.8||-z>interior.y_max-.8)return {ok:false,msg:'Fora da área do galpão'};const L=LOOSE[type],o=makeOBB(x,z,L.w/2,L.d/2,0);
  for(const so of solids)if(obbHit(so,o,.04))return {ok:false,msg:'Ponto ocupado: '+so.tag.toLowerCase()};for(const b of loose)if(obbHit(b.obb,o,.04))return {ok:false,msg:'Já existe um objeto nesse ponto'};
  for(const r of vehicles())if(obbHit(r.obb,o,.25))return {ok:false,msg:'Há um veículo nesse ponto'};for(const w of workers)if(w.y0<1&&circHit(o,w.x,w.z,.45))return {ok:false,msg:'Há uma pessoa nesse ponto'};return {ok:true}}
 function toolClick(type,x,z){const hit=loose.find(b=>circHit(b.obb,x,z,.2));if(hit){loose.splice(loose.indexOf(hit),1);return `${hit.L.label} removid${hit.L.g}`}
  const c=placementCheck(type,x,z);if(!c.ok)return c.msg;spawnLoose(type,x,z,0,{user:true});logEvent(`${LOOSE[type].label} posicionad${LOOSE[type].g} no piso`);return `${LOOSE[type].label} posicionad${LOOSE[type].g}`}
 function clearUser(){let n=0;for(let i=loose.length-1;i>=0;i--)if(loose[i].user){loose.splice(i,1);n++}return n}
 function vehStatus(r){if(r.backoff)return r.backoff.from?'Recuando para liberar '+r.backoff.from.id:'Recuando para replanejar o desvio';if(r.emergT>0)return 'Parada de emergência após contato';if(r.pinned)return 'Bloqueado: objeto prensado';
  switch(r.why){case 'yield':return 'Cedendo passagem para '+r.ref.id;case 'follow':return 'Mantendo distância de '+r.ref.id;case 'veh':return 'Aguardando '+r.ref.id+' liberar a faixa';
   case 'person':return r.v<.06?'Parado: pessoa no campo de proteção':'Reduzindo: pessoa à frente';case 'obj':return r.v<.06?(r.detourFail?'Sem espaço para desviar, aguardando remoção':'Obstáculo na rota, replanejando'):'Reduzindo: objeto na rota';case 'static':return 'Aguardando caminho livre'}
  if(r.pushing)return 'Empurrando objeto (detecção desligada)';if(r.detour)return 'Contornando obstáculo';if(r.curve&&r.v<r.speed*.8)return 'Reduzindo para a curva';return 'Em rota'}
 function workerStatus(w){if(w.act)return w.act.ready?ACTS[w.act.type].label:'Posicionando-se para a tarefa';if(w.waiting)return 'Aguardando passagem de veículo';if(w.side>.45)return 'Desviando de obstáculo';if(w.v<.1)return 'Parado';return w.carry?'Transportando volume':'Caminhando'}
 return {robots,forklifts,workers,loose,traffic,opts,
  get time(){return t},
  step(dt){t+=dt;trafficStep(dt);for(const w of workers)updateWorker(w,dt,t)},
  // onboard: robot seen from its own camera; its field outline would sit on the lens and is not drawn.
  draw(a,{people=true,vehicles=true,fields=true,selected=null,eye=null,tool=null,hover=null,onboard=null}={}){
   for(const r of robots){drawRobot(a,r,t);if(fields&&r.id!==onboard)drawFields(a,r)}
   if(vehicles)for(const r of forklifts){drawForklift(a,r,t);if(fields)drawFields(a,r)}
   if(people)for(const w of workers){const far=eye&&Math.hypot(eye[0]-w.x,eye[1]-w.y0,eye[2]-w.z)>70;far&&w!==selected?drawHumanFar(a,w):drawHuman(a,w,t,w===selected)}
   for(const ob of loose)drawLooseObject(a,ob);for(const f of flashes)drawFlash(a,f);
   if(tool&&hover)drawPlacement(a,hover,LOOSE[tool].w,LOOSE[tool].d)},
  setSensors(on){sensorsOn=on;logEvent(on?'Detecção de objetos ativada':'Detecção de objetos desligada')},
  placementCheck,toolClick,clearUser,vehStatus,workerStatus};
}
