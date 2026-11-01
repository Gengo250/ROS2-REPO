// ATLAS drawing code (articulated workers, task poses, vehicles, safety fields, loose
// objects), taken from ATLAS_Galpao_Industrial_3D.html. Drawing only: nothing here moves
// an entity. Coordinates are WebGL (x east, y up, z south); B = {x, z, yaw}. Each draw call
// appends 13-float instances to a.cube / a.cylinder / a.sphere batches. Globals of the
// prototype (time, tool state) are passed as arguments.
'use strict';

export const TAU = Math.PI*2;
export const clamp=(v,a,b)=>v<a?a:v>b?b:v;
export const mix=(a,b,k)=>a+(b-a)*k;
export const smooth=(e0,e1,x)=>{const k=clamp((x-e0)/(e1-e0),0,1);return k*k*(3-2*k)};
export const wrapA=a=>{a=(a+Math.PI)%TAU;if(a<0)a+=TAU;return a-Math.PI};
export const LT=.45,LS=.44,ANKLE=.085,HIPW=.1;
export function rgb(hex){let a=parseInt(hex.replace('#',''),16);return [(a>>16&255)/255,(a>>8&255)/255,(a&255)/255]}
const rgbMemo=new Map();
export const rgbc=h=>{let v=rgbMemo.get(h);if(!v){v=rgb(h);rgbMemo.set(h,v)}return v};
// mesma ordem de rotação do vertex shader: roll (X), tilt (Z), twist (Y)
export function R3(v,tw,tl,rl){let x=v[0],y=v[1],z=v[2],c=Math.cos(rl),s=Math.sin(rl),y2=c*y-s*z;z=s*y+c*z;y=y2;c=Math.cos(tl);s=Math.sin(tl);const x2=c*x-s*y;y=s*x+c*y;x=x2;c=Math.cos(tw);s=Math.sin(tw);return [c*x+s*z,y,-s*x+c*z]}
export const v3add=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
export const v3mul=(a,k)=>[a[0]*k,a[1]*k,a[2]*k];
export function hpart(a,B,type,l,size,tw,tl,rl,col,em=0){const c=Math.cos(B.yaw),s=Math.sin(B.yaw),cl=rgbc(col);a[type].push(B.x+l[0]*c+l[2]*s,l[1],B.z-l[0]*s+l[2]*c,size[0],size[1],size[2],cl[0],cl[1],cl[2],em,B.yaw+tw,tl,rl)}
export function hlimb(a,B,j,len,r,tw,tl,rl,col){hpart(a,B,'cylinder',v3add(j,R3([0,-len/2,0],tw,tl,rl)),[r,len,r],tw,tl,rl,col);return v3add(j,R3([0,-len,0],tw,tl,rl))}
export function hball(a,B,p,r,col,em=0){hpart(a,B,'sphere',p,[r,r,r],0,0,0,col,em)}

// ---------- Pessoas: poses e desenho ----------
function basePose(w,time){const g=w.gaitW,A=w.A,s=Math.sin(w.phase),c=Math.cos(w.phase),idle=1-g,arm=A*1.15*g,brz=Math.sin(time*.9+w.seed);
 const P={lean:.04+.06*clamp(w.v/1.4,0,1),crouch:0,headPitch:.05,headYaw:w.look,twist:-.13*s*g,pelvisTwist:.16*s*g,sway:-.03*c*g+.022*idle*brz,breath:1+.014*Math.sin(time*1.7+w.seed),
  armR:{up:-arm*s+.04*idle*brz,bend:.2+.5*Math.max(0,-s)*g,roll:.1},armL:{up:arm*s-.04*idle*brz,bend:.2+.5*Math.max(0,s)*g,roll:.1}};
 const kneeling=w.act&&w.act.type==='kneel';
 if(w.carry){P.armR={up:.55,bend:1.05,roll:-.02};P.armL={up:.55,bend:1.05,roll:-.02};P.lean-=.04;P.twist*=.35}
 else if(w.hold==='tablet')P.armL={up:.22+.05*s*g,bend:1.3,roll:.2};
 if(w.hold==='toolbox'&&!kneeling&&!w.carry)P.armR={up:-.1*s*g,bend:.06,roll:.2};
 return P}
function actPose(w,act,time){const t=act.t,u=clamp(t/act.dur,0,1),bell=Math.sin(Math.PI*u);
 switch(act.type){
  case 'scan':act.beep=act.ready&&((t%1.4)>.75&&(t%1.4)<1.05);return {armR:{up:1.18,bend:.42,roll:.05},headPitch:-.04,headYaw:.1,twist:.12,lean:.02};
  case 'pick':{const k=Math.pow(bell,.8);return {armR:{up:.3+1.0*k,bend:.35-.15*k,roll:.06},armL:{up:.3+1.0*k,bend:.35-.15*k,roll:.06},headPitch:-.2*k,lean:.02,twist:0}}
  case 'drop':{const k=Math.pow(bell,.85);return {crouch:.86*k,lean:.08+.45*k,headPitch:.1+.35*k,armR:{up:.45+.4*k,bend:.9-.5*k,roll:0},armL:{up:.45+.4*k,bend:.9-.5*k,roll:0},twist:0}}
  case 'inspect':{const glance=smooth(.5,.6,u)*(1-smooth(.72,.82,u));return {armL:{up:.28,bend:1.3,roll:.22},armR:{up:.32,bend:1.12+.18*Math.max(0,Math.sin(t*5.5)),roll:-.12},headPitch:.42-.5*glance,headYaw:.5*glance,lean:.06}}
  case 'kneel':{const h=Math.sin(t*6.5);return {crouch:.93,lean:.34,headPitch:.36,armR:{up:.95+.16*h,bend:.55+.3*Math.sin(t*6.5+1.2),roll:.05},armL:{up:.6,bend:.5,roll:.1},twist:.08*h}}
  case 'signal':{const wv=Math.sin(t*6.2);return {armR:{up:2.7,bend:.35+.25*(.5+.5*wv),roll:.18+.32*wv},armL:{up:.06,bend:.25,roll:.12},headPitch:-.1,headYaw:0,lean:-.01}}
  case 'lookaround':{const pw=smooth(.42,.5,u)*(1-smooth(.7,.78,u));return {headYaw:1.0*Math.sin(u*TAU),twist:.26*Math.sin(u*TAU),headPitch:-.02,armR:{up:1.42,bend:.06,roll:.16},armRw:pw}}
  default:return {headYaw:.38*Math.sin(t*.8),sway:.03*Math.sin(t*1.1),armR:{up:-.06,bend:.18,roll:.12},armL:{up:-.06,bend:.18,roll:.12}}}}
function blendPose(P,Q,k){for(const key in Q){if(key==='armRw'||key==='armLw')continue;const q=Q[key];if(typeof q==='object'){const kk=k*(Q[key+'w']??1);for(const sub in q)P[key][sub]=mix(P[key][sub],q[sub],kk)}else P[key]=mix(P[key],q,k)}return P}
function workerPose(w,time){const P=basePose(w,time);if(w.act&&w.act.ready){const a=w.act,e=Math.min(1,a.t/.45,(a.dur-a.t)/.45);blendPose(P,actPose(w,a,time),smooth(0,1,e))}else if(w.act)w.act.beep=false;return P}

// Gait integration of the prototype (presentation only): phase, gait weight and amplitude
// follow the walked speed and turn rate; in LIVE they come from backend state.
export function animateGait(w,dt,yawRate){const vAnim=Math.max(w.v,Math.abs(yawRate)*.28);
 w.gaitW+=(smooth(.03,.45,vAnim)-w.gaitW)*(1-Math.exp(-dt*7));w.A=clamp(.14+.22*vAnim,.12,.46);
 const stride=Math.max(.45,4*(LT+LS)*Math.sin(w.A));w.phase=(w.phase+TAU*vAnim/stride*dt)%TAU;w.steps+=2*vAnim/stride*dt}

export function drawHuman(a,w,time,sel){const P=workerPose(w,time),B={x:w.x,z:w.z,yaw:w.yaw},y0=w.y0,g=w.gaitW,A=w.A,R=w.R;
 const boots=R.vest?'#2a2521':'#1d2128',gloves=R.vest?'#4a525c':w.skin;
 // pernas: a pelve é posicionada para que o pé de apoio toque o chão (o balanço vertical surge sozinho)
 const legs=[];for(const zs of [1,-1]){const ph=w.phase+(zs>0?0:Math.PI),s=Math.sin(ph),c=Math.cos(ph);const hip=g*A*s+P.crouch*1.2,knee=g*(.06+2.4*A*Math.max(0,c)**2)+P.crouch*2.05+.04,ft=g*.3*Math.max(0,c);legs.push({zs,hip,knee,ft,drop:LT*Math.cos(hip)+LS*Math.cos(hip-knee)})}
 const pelvisY=y0+ANKLE+Math.max(legs[0].drop,legs[1].drop),ptw=P.pelvisTwist;
 for(const L of legs){const hj=v3add([0,pelvisY,P.sway],R3([0,0,L.zs*HIPW],ptw,0,0));const kn=hlimb(a,B,hj,LT,.074,ptw,L.hip,0,w.pants);hball(a,B,kn,.066,w.pants);const an=hlimb(a,B,kn,LS,.06,ptw,L.hip-L.knee,0,w.pants);hpart(a,B,'cube',v3add(an,R3([.055,-.042,0],ptw,L.ft,0)),[.27,.09,.12],ptw,L.ft,0,boots)}
 const tw=P.twist,tl=-P.lean,root=[0,pelvisY+.06,P.sway],at=(v)=>v3add(root,R3(v,tw,tl,0));
 hpart(a,B,'cube',[0,pelvisY+.02,P.sway],[.22,.2,.31],ptw,0,0,w.pants);
 hpart(a,B,'cube',at([0,.02,0]),[.235,.06,.325],tw,tl,0,'#23272c');
 hpart(a,B,'cube',at([0,.27,0]),[.23,.5*P.breath,.36],tw,tl,0,w.shirt);
 if(R.vest){hpart(a,B,'cube',at([0,.21,0]),[.255,.36,.385],tw,tl,0,R.vest);for(const hy of [.1,.27])hpart(a,B,'cube',at([0,hy,0]),[.262,.042,.392],tw,tl,0,'#e6eee8',.55)}
 else hpart(a,B,'cube',at([.118,.3,.07]),[.012,.07,.05],tw,tl,0,'#7fe7d6',.5);
 // braços
 const hands={};for(const zs of [1,-1]){const arm=zs>0?P.armR:P.armL,rl=-zs*arm.roll,ut=tl*.5+arm.up,ft=ut+arm.bend,S=at([0,.44,zs*.205]);hball(a,B,S,.072,w.shirt);const el=hlimb(a,B,S,.28,.052,tw,ut,rl,w.shirt);hball(a,B,el,.049,w.shirt);const wr=hlimb(a,B,el,.25,.043,tw,ft,rl,w.shirt);const hc=v3add(wr,R3([0,-.055,0],tw,ft,rl));hpart(a,B,'cube',hc,[.075,.1,.055],tw,ft,rl,gloves);hands[zs]={wr,hc,ft,rl}}
 // cabeça
 const N=at([0,.5,0]),htw=tw+P.headYaw,htl=tl*.35-P.headPitch,H=v3add(N,R3([0,.17,0],htw,htl,0)),hh=v=>v3add(H,R3(v,htw,htl,0));
 hpart(a,B,'cylinder',v3add(N,R3([0,.04,0],tw,tl,0)),[.052,.1,.052],tw,tl,0,w.skin);
 hpart(a,B,'sphere',H,[.105,.125,.1],htw,htl,0,w.skin);
 if(R.helmet){hpart(a,B,'sphere',hh([-.005,.055,0]),[.13,.085,.122],htw,htl,0,R.helmet);hpart(a,B,'cube',hh([.045,.04,0]),[.29,.018,.255],htw,htl,0,R.helmet);hpart(a,B,'cube',hh([0,.13,0]),[.2,.03,.035],htw,htl,0,R.helmet)}
 else hpart(a,B,'sphere',hh([-.018,.045,0]),[.11,.1,.106],htw,htl,0,w.hair);
 for(const ez of [.038,-.038])hpart(a,B,'cube',hh([.095,.02,ez]),[.02,.024,.022],htw,htl,0,'#1b2129');
 // objetos nas mãos
 const act=w.act,kneeling=act&&act.type==='kneel'&&act.ready;
 if(w.carry){const m=v3mul(v3add(hands[1].hc,hands[-1].hc),.5),c=v3add(m,R3([.05,.04,0],tw,0,0));hpart(a,B,'cube',c,[.4,.3,.38],tw,0,0,'#c9a46e');hpart(a,B,'cube',v3add(c,[0,.152,0]),[.41,.01,.07],tw,0,0,'#e8d6a4')}
 if(w.hold==='tablet'&&!w.carry){const h=hands[-1],c=v3add(h.hc,R3([.04,.03,.045],tw,0,0));hpart(a,B,'cube',c,[.17,.014,.24],tw,.55,0,'#2b3a48');hpart(a,B,'cube',v3add(c,R3([0,.009,0],tw,.55,0)),[.15,.004,.21],tw,.55,0,'#5ad6e6',.75)}
 if(w.hold==='toolbox'){const c=kneeling?[.38,y0+.09,.42]:v3add(hands[1].hc,[0,-.14,0]);hpart(a,B,'cube',c,[.36,.17,.14],kneeling?.6:tw,0,0,'#c4473a');hpart(a,B,'cube',v3add(c,[0,.1,0]),[.2,.03,.03],kneeling?.6:tw,0,0,'#2b2f35')}
 if(w.role==='pick'){if(act&&act.type==='scan'&&act.ready){const h=hands[1],d=R3([0,-1,0],tw,h.ft,h.rl);hpart(a,B,'cube',v3add(h.wr,v3mul(d,.1)),[.055,.17,.065],tw,h.ft,h.rl,'#2c3138');const tip=v3add(h.wr,v3mul(d,.19));hpart(a,B,'cube',tip,[.045,.03,.055],tw,h.ft,h.rl,act.beep?'#ff5a4e':'#7a2b28',act.beep?1:.2);if(act.beep)hpart(a,B,'cube',v3add(tip,v3mul(d,.7)),[.01,1.4,.01],tw,h.ft,h.rl,'#ff4a4a',1)}else hpart(a,B,'cube',at([0,.04,-.19]),[.12,.07,.05],tw,tl,0,'#2c3138')}
 // sombra de contato e seleção
 hpart(a,B,'cylinder',[0,y0+.006,0],[.4,.005,.4],0,0,0,'#141f28');
 if(sel){hpart(a,B,'cylinder',[0,y0+.003,0],[.68,.004,.68],0,0,0,'#65eed7',.95);hpart(a,B,'sphere',[0,y0+2.25+.06*Math.sin(time*3),0],[.09,.09,.09],0,0,0,'#65eed7',1)}}

// Level of detail: a distant worker as body, head and helmet only (5 instances, same colours).
export function drawHumanFar(a,w){const B={x:w.x,z:w.z,yaw:w.yaw},y0=w.y0,R=w.R;
 hpart(a,B,'cube',[0,y0+.45,0],[.2,.9,.3],0,0,0,w.pants);hpart(a,B,'cube',[0,y0+1.2,0],[.24,.6,.38],0,0,0,R.vest||w.shirt);
 hpart(a,B,'sphere',[0,y0+1.62,0],[.11,.125,.1],0,0,0,w.skin);if(R.helmet)hpart(a,B,'sphere',[0,y0+1.68,0],[.13,.085,.122],0,0,0,R.helmet);
 hpart(a,B,'cylinder',[0,y0+.006,0],[.4,.005,.4],0,0,0,'#141f28')}

// ---------- Veículos, campos e objetos ----------
export const FIELD=['#4fe3c1','#ffbf45','#ff5145'];
export function boxDyn(arr,x,y,z,w,h,d,col,yaw=0,emit=0){const c=rgbc(col);arr.cube.push(x,y,z,w,h,d,c[0],c[1],c[2],emit,yaw,0,0)}
export function cylDyn(arr,x,y,z,r,h,col,yaw=0,emit=0){const c=rgbc(col);arr.cylinder.push(x,y,z,r,h,r,c[0],c[1],c[2],emit,yaw,0,0)}
export function sphereDyn(arr,x,y,z,r,col,emit=0){const c=rgbc(col);arr.sphere.push(x,y,z,r,r,r,c[0],c[1],c[2],emit,0,0,0)}
export function segDyn(a,x1,z1,x2,z2,w,y,col,em){const dx=x2-x1,dz=z2-z1,l=Math.hypot(dx,dz);if(l<1e-3)return;boxDyn(a,(x1+x2)/2,y,(z1+z2)/2,l,.012,w,col,-Math.atan2(dz,dx),em)}
function rotate(x,z,yaw){return [x*Math.cos(yaw)+z*Math.sin(yaw),-x*Math.sin(yaw)+z*Math.cos(yaw)]}
export function relbox(a,pos,dx,y,dz,w,h,d,c,em=0){let [x,z]=rotate(dx,dz,pos.yaw);boxDyn(a,pos.x+x,y,pos.z+z,w,h,d,c,pos.yaw,em)}
export function relcyl(a,pos,dx,y,dz,r,h,c,em=0){let [x,z]=rotate(dx,dz,pos.yaw);cylDyn(a,pos.x+x,y,pos.z+z,r,h,c,pos.yaw,em)}
export function beaconCol(r,t){if(r.emergT>0||r.field===2)return Math.sin(t*13)>0?'#ff4a3d':'#4a1512';if(r.field===1)return '#ffc14d';return r.lift?(Math.sin(t*7+r.rank)>.2?'#ffe08a':'#6b5420'):'#a5fff0'}
// Prototype AMR (1.64 × 1.30 m), used by the DEMO fleet.
export function drawRobot(a,r,t){const p=r.pose;relbox(a,p,0,.34,0,1.6,.46,1.22,'#263f54');relbox(a,p,0,.62,0,1.48,.13,1.1,r.color,.22);relbox(a,p,.4,.74,0,.58,.11,.84,'#d2ede8',.28);relbox(a,p,.62,.45,0,.2,.32,.98,r.field?FIELD[r.field]:'#6ecfcb',r.field?.8:.35);relcyl(a,p,0,.9,0,.2,.26,'#18354b',.3);relcyl(a,p,0,1.06,0,.1,.08,beaconCol(r,t),1);for(let dz of [-.55,.55])for(let dx of [-.53,.53])relbox(a,p,dx,.14,dz,.35,.21,.19,'#172631');relbox(a,p,-.77,.34,0,.08,.23,.48,'#f7c376',.2);relbox(a,p,.77,.34,0,.09,.23,.48,'#f7c376',.2);return p}
// The real AMR at true size (0.95 × 0.70 m chassis, front bumper, rear 360° scanner mast)
// in the ATLAS style: dark chassis, coloured deck, sensor bay lit by the safety-field state.
export function drawRobotTrue(a,r,t){const p=r.pose;relbox(a,p,0,.155,0,.95,.24,.70,'#263f54');relbox(a,p,0,.31,0,.88,.055,.64,r.color,.22);relbox(a,p,.15,.345,0,.34,.02,.5,'#d2ede8',.28);relbox(a,p,.44,.2,0,.06,.09,.5,r.field?FIELD[r.field]:'#6ecfcb',r.field?.8:.35);relbox(a,p,.4875,.07,0,.025,.07,.48,'#f7c376',.2);relcyl(a,p,-.38,.4,0,.018,.09,'#688199');relcyl(a,p,-.38,.48,0,.04,.05,'#18354b',.3);relcyl(a,p,-.38,.52,0,.025,.03,beaconCol(r,t),1);for(let dz of [-.29,.29])relbox(a,p,0,.12,dz,.24,.24,.055,'#172631');return p}
export function drawForklift(a,r,t){const p=r.pose;relbox(a,p,0,.72,0,2.9,1.08,1.82,r.color);relbox(a,p,-.42,1.68,0,1.5,1.1,1.74,'#4f6f7b');relbox(a,p,-.42,2.37,0,1.65,.18,1.86,'#edb34f');for(let dz of [-.86,.86])for(let dx of [-1.02,.96])relbox(a,p,dx,.28,dz,.42,.55,.26,'#182b34');relbox(a,p,1.25,1.46,0,.21,2.88,1.84,'#374f62');relbox(a,p,2.1,.31,-.48,1.9,.16,.18,'#84949b');relbox(a,p,2.1,.31,.48,1.9,.16,.18,'#84949b');relcyl(a,p,-.42,2.65,0,.22,.12,beaconCol(r,t),1);return p}
// Forklift blue spot and red side lines (prototype), independent of the sampled field.
export function drawForkliftMarks(a,r){const S=r.S,p=r.pose,c=Math.cos(p.yaw),s=Math.sin(p.yaw);const f=S.cx+S.L/2+2.3;a.cylinder.push(p.x+c*f,.2,p.z-s*f,.5,.012,.32,...rgbc('#3d9bff'),1.1,p.yaw,0,0);
 for(const sg of [1,-1]){const off=sg*(S.W/2+.7),x1=p.x+c*-1.3+s*off,z1=p.z+s*1.3+c*off,x2=p.x+c*2.2+s*off,z2=p.z-s*2.2+c*off;segDyn(a,x1,z1,x2,z2,.07,.205,'#ff3b3b',1)}}
// Field outline swept along sample poses ahead (DEMO) or a straight field (LIVE).
export function drawFieldSamples(a,r,samples){const S=r.S;if(!samples.length)return;const col=FIELD[r.field],hw=S.W/2+.22,front=S.cx+S.L/2,y=.21;let pl=null,pr=null,prot=false;
 for(const sm of samples){const q=sm.p,qc=Math.cos(q.yaw),qs=Math.sin(q.yaw),fx=q.x+qc*front,fz=q.z-qs*front,lx=fx+qs*hw,lz=fz+qc*hw,rx=fx-qs*hw,rz=fz-qc*hw;
  if(pl){segDyn(a,pl[0],pl[1],lx,lz,.05,y,col,.95);segDyn(a,pr[0],pr[1],rx,rz,.05,y,col,.95)}else segDyn(a,lx,lz,rx,rz,.04,y,col,.6);
  if(!prot&&sm.s>=r.prot){prot=true;segDyn(a,lx,lz,rx,rz,.035,y,col,.6)}pl=[lx,lz];pr=[rx,rz]}
 segDyn(a,pl[0],pl[1],pr[0],pr[1],.05,y,col,.95)}
export function drawFields(a,r){if(r.lift)drawForkliftMarks(a,r);drawFieldSamples(a,r,r.samples)}
export function drawLooseObject(a,ob){const B={x:ob.x,z:ob.z,yaw:ob.yaw};
 if(ob.type==='cone'){const th=-(ob.tip||0),sh=Math.sin(ob.tip||0),P=v=>{const q=R3(v,0,th,0);return [q[0]-.24*sh,q[1]+.03+.14*sh,q[2]]};
  hpart(a,B,'cube',P([0,.015,0]),[.42,.03,.42],0,th,0,'#2a2f35');hpart(a,B,'cylinder',P([0,.14,0]),[.16,.22,.16],0,th,0,'#ff7a2a');hpart(a,B,'cylinder',P([0,.33,0]),[.118,.16,.118],0,th,0,'#f2f4f0',.55);hpart(a,B,'cylinder',P([0,.5,0]),[.078,.18,.078],0,th,0,'#ff7a2a');hpart(a,B,'cylinder',P([0,.62,0]),[.04,.07,.04],0,th,0,'#ff7a2a')}
 else if(ob.type==='pallet'){hpart(a,B,'cube',[0,.09,0],[1.22,.14,1.02],0,0,0,'#b48a55');for(const z of [-.42,0,.42])hpart(a,B,'cube',[0,.035,z],[1.22,.07,.14],0,0,0,'#8f6a3e');hpart(a,B,'cube',[0,.62,0],[1.12,.9,.94],0,0,0,'#cfb28a');hpart(a,B,'cube',[0,1.08,0],[1.14,.04,.96],0,0,0,'#e3d3b2');hpart(a,B,'cube',[0,.62,0],[1.135,.08,.955],0,0,0,'#a9c8d6',.2)}
 else {const w=ob.w,d=ob.d,h=ob.type==='box'?.5:.3;hpart(a,B,'cube',[0,.03+h/2,0],[w,h,d],0,0,0,ob.type==='box'?'#c49a62':'#c9a46e');hpart(a,B,'cube',[0,.035+h,0],[w+.01,.01,.07],0,0,0,'#e8d6a4')}}
export function drawFlash(a,f){const k=f.t/.9,rad=.4+k*2.2;for(let i=0;i<14;i++){const a1=i/14*TAU,a2=(i+1)/14*TAU;segDyn(a,f.x+Math.cos(a1)*rad,f.z+Math.sin(a1)*rad,f.x+Math.cos(a2)*rad,f.z+Math.sin(a2)*rad,.08,.22,'#ff5145',1.3*(1-k))}}
// Placement preview of the object tool (outline + pin), valid in mint, invalid in red.
export function drawPlacement(a,hover,w,d){if(!hover)return;const col=hover.ok?'#65eed7':'#ff5145',c=Math.cos(hover.yaw||0),s=Math.sin(hover.yaw||0),ux=c,uz=-s,vx=s,vz=c,hw=w/2,hd=d/2;
 const cs=[[1,1],[1,-1],[-1,-1],[-1,1]].map(([i,j])=>[hover.x+ux*hw*i+vx*hd*j,hover.z+uz*hw*i+vz*hd*j]);for(let i=0;i<4;i++)segDyn(a,cs[i][0],cs[i][1],cs[(i+1)%4][0],cs[(i+1)%4][1],.06,.22,col,1);cylDyn(a,hover.x,.6,hover.z,.03,1.2,col,0,1)}
