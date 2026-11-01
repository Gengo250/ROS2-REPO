// ATLAS industrial campus builders, taken from ATLAS_Galpao_Industrial_3D.html with the
// geometry, colours, seed and call order unchanged. Only the DOM writes were removed.
// Output is in ATLAS/WebGL axes (x east, y up, z south); compile.mjs converts it to the
// ROS world frame. Instance layout per primitive (13 floats):
// x, y, z, scale x, scale y, scale z, r, g, b, emission, yaw, tilt, roll.
'use strict';

export const LAYERS = ['ground', 'architecture', 'roof', 'racks', 'cargo', 'equipment', 'markings', 'routes', 'deco'];

export function buildCampus() {
  const C = {floor:'#273b4c',floorAlt:'#354b5c',dark:'#101e2a',asphalt:'#253440',road:'#35434e',wall:'#9baec1',wallLight:'#c3d1dd',steel:'#415e74',steel2:'#688199',racks:'#64809a',beam:'#f6ab44',box:'#d5aa76',teal:'#42e0c7',blue:'#4c99e7',yellow:'#f4c36d',red:'#f08778',glass:'#57a2c0',white:'#dce8ed',green:'#5edbb9'};
  function rgb(hex){let a=parseInt(hex.replace('#',''),16);return [(a>>16&255)/255,(a>>8&255)/255,(a&255)/255]}
  let seed=42930;const rand=()=>((seed=(Math.imul(1664525,seed)+1013904223)>>>0)/4294967296);const rr=(a,b)=>a+(b-a)*rand();const choice=a=>a[Math.floor(rand()*a.length)];
  const groups={};for(let l of LAYERS)groups[l]={cube:[],cylinder:[],sphere:[]};
  function item(type,layer,pos,scale,col,yaw=0,emit=0){(groups[layer]??groups.deco)[type].push(...pos,...scale,...rgb(col),emit,yaw,0,0)}
  const box=(l,x,y,z,w,h,d,c,yaw=0,emit=0)=>item('cube',l,[x,y,z],[w,h,d],c,yaw,emit);
  const cyl=(l,x,y,z,r,h,c,yaw=0,emit=0)=>item('cylinder',l,[x,y,z],[r,h,r],c,yaw,emit);
  const orb=(l,x,y,z,r,c,emit=0)=>item('sphere',l,[x,y,z],[r,r,r],c,0,emit);
  function seg(l,x1,z1,x2,z2,width,h,c,y=0.1,emit=0){let dx=x2-x1,dz=z2-z1;box(l,(x1+x2)/2,y,(z1+z2)/2,Math.hypot(dx,dz),h,width,c,-Math.atan2(dz,dx),emit)}
  function stripeLine(x1,z1,x2,z2,dash=2,gap=1,width=.1,color='#dba94e'){let d=Math.hypot(x2-x1,z2-z1),ux=(x2-x1)/d,uz=(z2-z1)/d;for(let q=0;q<d;q+=dash+gap){let e=Math.min(q+dash,d);seg('markings',x1+ux*q,z1+uz*q,x1+ux*e,z1+uz*e,width,.018,color,.14,.08)}}
  // OBB colliders of the demo traffic (x, z, half sizes, yaw); kept as coarse footprints.
  const solids=[];
  function solid(x,z,w,d,tag,yaw=0){solids.push({x,z,w,d,yaw,tag})}

  function makeGround(){box('ground',0,-.2,0,146,.36,108,'#344a5c');box('ground',0,-.55,0,218,.30,190,'#1c303c');box('ground',0,-.75,0,280,.15,245,'#12202b');
   for(let x=-70;x<=70;x+=10)seg('markings',x,-53,x,53,.026,.012,'#5f7586',.012);
   for(let z=-50;z<=50;z+=10)seg('markings',-71,z,71,z,.026,.012,'#5f7586',.012);
   // operational spine and safety lanes
   box('ground',2,-.006,0,12,.008,102,'#33515d');for(let x of [-4,8]){seg('markings',x,-50,x,50,.14,.018,'#f4bc62',.14)}
   for(let z of [-23,21]){box('ground',0,.015,z,140,.025,7,'#3d5361');for(let r of [z-3.25,z+3.25])seg('markings',-69,r,69,r,.1,.02,'#edb760',.145)}
   // front dispatch staging
   for(let x=-65;x<65;x+=7.5){box('markings',x,.05,36,6,.03,12,'#315467');seg('markings',x-3,30,x-3,42,.1,.02,'#efc368',.15)}
   // External road and parking
   box('ground',0,-.37,76,188,.06,38,'#2a3d4a');for(let x=-85;x<=85;x+=11)stripeLine(x,67,x,89,1.5,1.2,.11,'#f0c371');
   stripeLine(-90,94,90,94,4,2,.17,'#d8bf7b');
   for(let i=0;i<7;i++){let x=-62+i*18;box('ground',x,-.34,104,11,.04,13,'#2e424c');seg('markings',x-5.5,98,x-5.5,111,.11,.02,'#bec8ca',-.28)}
   for(let z=-43;z<=42;z+=6){seg('markings',-9.2,z,-9.2,z+2,.12,.015,'#b9d7d0',.13);seg('markings',11,z,11,z+2,.12,.015,'#b9d7d0',.13)}
   // crosswalks near dock access
   for(let x=-68;x<68;x+=2.5)box('markings',x,.1,44,1.4,.025,2.7,'#d4e0da');
   // dock apron stripes
   for(let i=-3;i<=3;i++){let z=55+i*1.6;seg('markings',-68,z,68,z,.055,.02,'#668595',-.29)}
  }
  function makeArchitecture(){// side and rear walls, cutaway south walls
   for(let x of [-73,73]){for(let z=-54;z<54;z+=8){box('architecture',x,6.1,z,.65,12.5,.7,'#8fa3b4');if(z<-35||z>29)box('architecture',x,6.3,z+3.9,.35,11.6,7.3,'#5a7389')}}
   for(let x=-72;x<73;x+=8){box('architecture',x,6.5,-54,.7,13,.7,'#60798e');if(x>-60&&x<58){box('architecture',x+3.8,9,-54,.18,5,7.1,'#80a9be');box('architecture',x+3.8,3.6,-54,.18,4,7.1,'#98aebd')}}
   // south side: structural columns + loading doors
   for(let x=-72;x<=72;x+=9){box('architecture',x,5.7,54,.6,11.5,.7,'#8197ab');cyl('deco',x,10.4,53.9,.15,.35,'#83d7cf',0,.8)}
   for(let i=0;i<8;i++){let x=-56+16*i;box('architecture',x,7.7,54,13.2,1.0,.5,'#7b94a9');box('architecture',x,2.1,54,8,4.2,.25,'#3b586a');for(let y=.7;y<4.2;y+=.75)box('architecture',x,y,54.17,7.7,.09,.14,'#637e94');box('architecture',x-4.2,2.6,53.8,.24,5.2,.35,'#dcb264');box('architecture',x+4.2,2.6,53.8,.24,5.2,.35,'#dcb264');box('deco',x,5.0,53.85,3.1,.34,.18,'#326d78',0,.65);box('deco',x,5,54.01,1.6,.20,.05,'#c9fff0',0,.9);box('architecture',x,.33,54,10,.45,2,'#657b87')}
   // outer vertical columns
   for(let x of [-71,71])for(let z of [-51,51])cyl('architecture',x,5.5,z,.46,11,'#b3c7cc');
   // crane beams across interior ceiling
   for(let z=-47;z<47;z+=17){box('roof',0,12.2,z,145,.4,.7,'#7b93a7');for(let x=-70;x<=70;x+=12){box('roof',x,10.85,z,.2,2.6,.22,'#526e81');box('roof',x,12.5,z,.16,.38,5.5,'#587587')}}
   for(let x=-65;x<68;x+=13){box('roof',x,13.15,0,.23,.35,107,'#8ea6b5');for(let z=-46;z<49;z+=14){box('roof',x,12.4,z,7,.13,.65,'#b1fff1',0,.95);box('roof',x,12.65,z,8,.14,.9,'#a4dad7',0,.3)}}
   for(let z of [-53,53])box('roof',0,14.25,z,146,.42,.6,'#637e93');for(let x of [-72,72])box('roof',x,14.2,0,.5,.45,108,'#637e93');
   // translucent roof approximation: segmented steel panels with skylight strips
   for(let x=-72;x<72;x+=14){if((x+72)%28===0){box('roof',x+5,14.0,0,10,.15,107,'#647c8b')}else {for(let z=-45;z<47;z+=20)box('roof',x+5,14.0,z,10,.08,12,'#a7d8db',0,.25)}}
   // wall lamps
   for(let x=-60;x<65;x+=14){box('deco',x,10.7,-53,.8,.22,.22,'#b1fff6',0,.95);box('deco',x,9.7,-53,.15,1.8,.25,'#456777')}
  }
  function makeRack(x,za,zb,index){solid(x,(za+zb)/2,3.74,zb-za+.4,'Estante');const w=3.3,depth=zb-za,mod=3.4,levels=[1.45,3.25,5.05,6.85],col=choice(['#5a738a','#708da1','#668497']);
   for(let z=za;z<=zb+.1;z+=mod){for(let sx of [-1,1]){box('racks',x+sx*w/2,3.65,z,.15,7.3,.19,'#4c7189');box('racks',x+sx*w/2,3.65,z,.055,7.3,.24,'#7995a8')}for(let y of levels)box('racks',x,y,z,w,.15,.2,'#f6b752')}
   for(let y of levels){box('racks',x,y,za,w,.2,.12,'#eaa94a');box('racks',x,y,zb,w,.2,.12,'#eaa94a');for(let sx of [-1,1])box('racks',x+sx*w/2,y,(za+zb)/2,.13,.13,depth,'#ecac4f')}
   let inventory=0;
   for(let z=za+mod/2;z<zb-.4;z+=mod){for(let y of levels){for(let xoff of [-.78,.75]){if(rand()<.19)continue;inventory++;
   let wood=choice(['#cfa466','#aa8257','#bd9560']);box('cargo',x+xoff,y+.18,z,1.35,.19,2.5,wood);
   let h=rr(.55,1.0),c=choice(['#b99872','#cba983','#ae8c65','#bd9f77','#8aabc0','#d5c09e','#8ba3b4']);box('cargo',x+xoff,y+.26+h/2,z,rr(1.06,1.25),h,rr(1.76,2.1),c);
   if(rand()<.65){box('cargo',x+xoff,y+.29+h,z+.94,1.15,.028,.038,'#8e6f4a');box('cargo',x+xoff,y+.29+h,z-.94,1.15,.028,.038,'#8e6f4a')}
   if(rand()<.18){box('cargo',x+xoff,y+.28+h+.12,z,.89,.22,1.45,choice(['#c49c68','#99b6c2','#d0b07e']))}
   }}}
   // end protectors and rack id panels
   for(let zz of [za,zb]){for(let xx of [-1.7,1.7])box('racks',x+xx,.55,zz,.34,1.1,.36,'#e4ad3d');box('deco',x,6.85,zz+(zz===za?-.15:.15),1.2,.34,.08,'#e4b364',0,.25)}
   return inventory;
  }
  let stockCount=0;
  function makeStorage(){let count=0;const rowX=[-64,-58,-52,-46,-40,-34,-28,-22,18,24,30,36,42,48,54,60];for(let i=0;i<rowX.length;i++){let x=rowX[i];count+=makeRack(x,-45,-27,i);count+=makeRack(x,-18,15,i);if(i%3!==0)count+=makeRack(x,26,31,i)}
   // inventory floor storage lanes
   for(let x=-67;x<=65;x+=6){if(x>-16&&x<17)continue;for(let z of [32.5,38.5]){if(rand()<.5)continue;if(x<-58||x>60||(z<35&&x>-11&&x<40))continue;solid(x,z,2.7,2.3,'Palete no chão');box('cargo',x,.24,z,2.7,.35,2.3,'#c79e65');for(let j=0;j<3;j++)box('cargo',x+(j-1)*.75,.85,z,rr(.55,.7),.9,1.8,choice(['#bf9c72','#9eb2c0','#d4bc92']))}}
   stockCount=count;}
  function makeProduction(){// sorting conveyor blue long line front center
   const x0=-8,x1=37,z=34;solid((x0+x1)/2,z,x1-x0,3.3,'Esteira de triagem');box('equipment',(x0+x1)/2,1,z,x1-x0,.42,2.8,'#487c99');box('equipment',(x0+x1)/2,.68,z,x1-x0,.28,3.2,'#506777');for(let x=x0+.7;x<x1;x+=.92){cyl('equipment',x,1.26,z,.17,2.3,'#a6bdc9');box('equipment',x,1.42,z,.08,.07,2.5,'#d0dee0')}
   for(let x of [0,14,28]){box('equipment',x,2.65,z-1.4,.18,2.6,.28,'#5d97b5');box('equipment',x,2.65,z+1.4,.18,2.6,.28,'#5d97b5');box('equipment',x,4,z,.3,.18,3.2,'#64c8db');box('deco',x,4.2,z,.45,.14,1.8,'#8bfff0',0,.9)}
   for(let x=-6;x<32;x+=4){box('equipment',x,1.75,z,1.1,.7,1.1,'#cca17a');box('equipment',x,2.12,z,1.13,.06,1.1,'#e4c59f')}
   // small robotic arms / machining cells rear
   for(let x of [-14,-6,2,13.5])for(let z of [-37,-29]){solid(x+.1,z,4.25,3.2,'Célula de produção');box('equipment',x,.8,z,4,1.55,3.2,'#4a7894');box('equipment',x,1.7,z,3.6,.15,2.8,'#93b1be');box('equipment',x,2.18,z,.75,.85,.75,'#f3b652');box('equipment',x,3.2,z,.38,1.7,.38,'#f1aa55');orb('equipment',x,4.05,z,.41,'#eff0d9');box('equipment',x+1.8,1.5,z,.3,2.6,2.8,'#498fa8');box('equipment',x+2.05,1.5,z,.1,1.3,1.3,'#72c6c6',0,.6)}
   // round inspection tables and crates
   for(let x of [-8,2,12,24]){const tz=27.5;solid(x,tz,2.8,2.8,'Mesa de inspeção');cyl('equipment',x,.7,tz,1.35,.2,'#92b6c3');cyl('equipment',x,.33,tz,.12,.7,'#718fa4');for(let j=0;j<3;j++)box('cargo',x+Math.cos(j*2.1)*.55,.99,tz+Math.sin(j*2.1)*.55,.6,.45,.6,'#cba776',j)}
   // battery charging stations near east back
   for(let z of [-45,-39,-33,-27]){solid(66,z,1.8,3.4,'Estação de recarga');box('equipment',66,1.45,z,1.8,2.9,3.4,'#435c79');box('equipment',65.0,2.1,z,.1,.9,1.6,'#75eccc',0,.7);box('markings',62,.09,z,5,.03,3.5,'#efc770')}
  }
  function makeOffice(){let x=59,z=-39;box('architecture',x,4.8,z,17,.65,20,'#718c9e');for(let xx of [50.6,67.4])for(let zz of [-48,-30]){solid(xx,zz,.5,.5,'Pilar do mezanino');box('architecture',xx,2.35,zz,.5,4.7,.5,'#7c9cad')}solid(49.1,-40.8,2.5,16.2,'Escada do mezanino');
   // glass walls + frames
   for(let xx=51.5;xx<67;xx+=2.3){box('architecture',xx,6.5,-28.95,.16,3.3,.12,'#cad6de');box('architecture',xx,6.5,-29.08,2.0,2.7,.06,'#387f9a',0,.25);box('architecture',xx,6.5,-49.05,.15,3.3,.12,'#c6d2dd')}
   for(let zz=-49;zz<-29;zz+=2.6){box('architecture',50.5,6.6,zz,.15,3.3,.13,'#c6d2dd');box('architecture',67.5,6.6,zz,.15,3.3,.13,'#c6d2dd')}
   box('architecture',59,8.2,-39,17,.35,20,'#576c85');for(let j=0;j<10;j++){let z=-48+j*1.6;box('architecture',49.1,j*.40+.26,z,2.5,.18,1.5,'#bdcbd2')}
   // guardrails
   for(let z=-49;z<=-29;z+=2.2)box('architecture',49.5,6.25,z,.1,2.2,.1,'#f5b95d');seg('architecture',49.5,-49,49.5,-29,.08,.08,'#ffbf69',7.25);
   // desks upper floor
   for(let x=54;x<=64;x+=4)for(let z=-46;z<=-33;z+=6){box('deco',x,5.2,z,2.3,.17,1.3,'#b8b2a0');box('deco',x,5.75,z-.35,1.1,.8,.11,'#264a65');box('deco',x,5.25,z-.26,1.1,.04,.08,'#77e7dc',0,.8)}
  }
  function makeDocksAndTrucks(){for(let i=0;i<8;i++){let x=-56+16*i;for(let side of [-1,1]){box('deco',x+side*5.6,.78,55.3,.43,1.55,1.3,'#f1ba52');box('deco',x+side*5.6,1.61,55.3,.28,.16,.45,'#6f9aa9')}
   if(i%2===0){ // parked truck trailers, south exterior
   let z=66;box('equipment',x,2.55,z,7.1,4.65,15.4,choice(['#cbd5dd','#a4b7c5','#cbbda9','#869eac']));box('equipment',x,4.5,z-7.45,6.9,.22,.11,'#d3e0e5');box('equipment',x,.65,z+9,5.8,1.1,4.4,'#475c6c');box('equipment',x,2.0,z+9.25,5.4,3.15,4.25,choice(['#4d94b4','#c69e65','#bc7667']));box('equipment',x,2.9,z+11.45,4.7,1.5,.12,'#a6d4d8',0,.2);for(let dz of [-5.1,5.8,10])for(let sx of [-1,1])cyl('equipment',x+sx*3,.43,z+dz,.7,.65,'#1c2931')}}
   // overhead cranes and safety pillars
   for(let z of [-14,22]){box('architecture',-6,9,z,24,.45,.8,'#e0ad50');box('architecture',-7,7.6,z,.35,2.1,.5,'#f3c162')}
  }
  function makeDetails(){// utility pipes, signage and structural elements
   for(let x=-69;x<70;x+=8){box('deco',x,10.5,-53,.28,.3,.45,'#aecbd7');box('deco',x,9.9,-52.5,.13,.13,1,'#80dfc7',0,.4)}
   for(let x of [-69,69])for(let z=-43;z<43;z+=18){box('deco',x+(x<0?1:-1),1.1,z,.26,1.5,.45,'#d24e4c');box('deco',x+(x<0?1:-1),1.93,z,.3,.38,.49,'#efd89a')}
   // pipe grid
   for(let z of [-41,-5,19]){box('deco',0,9.4,z,136,.16,.16,'#6e8896');for(let x=-61;x<66;x+=14)box('deco',x,10.0,z,.14,1.2,.13,'#5a7786')}
   // safety fences around machines
   for(let z of [-44,-26.3]){seg('deco',-18,z,16,z,.07,.15,'#eebc5f',1.2);for(let x=-18;x<=16.01;x+=3.4)box('deco',x,.75,z,.11,1.5,.11,'#f6c260')}
   // shrub planter external
   for(let x=-91;x<=91;x+=8){box('deco',x,.02,115,4,.2,2.3,'#3a4b4e');for(let j=0;j<3;j++)orb('deco',x+rr(-1.3,1.3),.55,115+rr(-.7,.7),.45,choice(['#49726a','#5c8873','#3e6866']))}
  }
  function makeColliders(){solid(-73,0,.7,108,'Parede');solid(73,0,.7,108,'Parede');solid(0,-54,146,.7,'Parede');solid(0,53.75,146,.5,'Parede das docas');
   for(let x of [-71,71])for(let z of [-51,51])solid(x,z,.92,.92,'Pilar');
   for(let x of [-68,68])for(let z=-43;z<43;z+=18)solid(x,z,.3,.5,'Extintor');
   for(let z of [-44,-26.3])solid(-1,z,34,.16,'Grade de proteção')}

  // One-way vehicle lanes. In LIVE they are painted lanes for the forklifts; AMRs plan on the graph.
  const LOOPS={
   west:{pts:[[-70,-21],[-1,-21],[-1,20.5],[-70,20.5]],color:'#64e5cd'},
   east:{pts:[[70,-24],[5,-24],[5,23.5],[70,23.5]],color:'#6dbbff'},
   north:{pts:[[-63.5,-21],[63.5,-21],[63.5,-24],[-63.5,-24]],color:'#ffd07e'},
   front:{pts:[[-12,20.5],[-12,41],[64.5,41],[64.5,20.5]],color:'#eea6dc'},
   dock:{pts:[[-66,41],[66,41],[66,44],[-66,44]],color:'#98e6cd'}};
  function drawLanes(){let li=0;
   for(const L of Object.values(LOOPS)){const y=.16+li++*.004;
    for(let j=0;j<L.pts.length;j++){const a=L.pts[j],b=L.pts[(j+1)%L.pts.length];seg('routes',a[0],a[1],b[0],b[1],.18,.045,L.color,y,.45);
     const d=Math.hypot(b[0]-a[0],b[1]-a[1]),ux=(b[0]-a[0])/d,uz=(b[1]-a[1])/d;for(let q=4;q<d-2;q+=9){const px=a[0]+ux*q,pz=a[1]+uz*q;for(const sg of [1,-1])seg('routes',px,pz,px-ux*.55+uz*sg*.4,pz-uz*.55-ux*sg*.4,.1,.03,L.color,y+.01,.7)}}}}

  // Original fleet table (loop, kind, cruise speed m/s); only forklifts are ported to LIVE.
  const FLEET=[['west','amr',1.85],['west','amr',1.7],['east','amr',1.9],['east','amr',1.75],['north','amr',2.0],['front','amr',1.6],['dock','amr',1.55],['dock','amr',1.65],['north','lift',1.45],['front','lift',1.3],['dock','lift',1.35]];
  const VSPEC={amr:{L:1.64,W:1.3,cx:0,accel:.9,brake:1.05,emerg:2.8,aLat:.75,gap:.45,k:.7,mass:260,name:'Robô móvel autônomo'},
   lift:{L:4.55,W:1.98,cx:.8,accel:.55,brake:.85,emerg:2.3,aLat:.5,gap:.7,k:1.1,mass:3800,name:'Empilhadeira'}};
  function track(points){let lengths=[],total=0;for(let j=0;j<points.length;j++){const a=points[j],b=points[(j+1)%points.length],d=Math.hypot(b[0]-a[0],b[1]-a[1]);lengths.push(d);total+=d}return {points,lengths,total}}
  function forklifts(){const perLoop={};FLEET.forEach(f=>perLoop[f[0]]=(perLoop[f[0]]||0)+1);const seen={};let nl=0;const out=[];
   FLEET.forEach(([loop,kind,sp],rank)=>{const k=seen[loop]=(seen[loop]||0)+1;if(kind!=='lift')return;
    const t=track(LOOPS[loop].pts);out.push({id:`EMP-${++nl}`,loop,speed:sp,start_distance:t.total*(k-1)/perLoop[loop]+rank*3.1,color:['#f4b64f','#e9a443','#e6c063'][nl-1]})});return out}

  // Workers: same crew table and the same seeded draws as makeAgents, in the same order.
  const TAU=Math.PI*2,H=Math.PI/2;
  const ROLES={
   pick:{name:'Separação de pedidos',vest:'#ff9a3c',helmet:'#f2c230',vmax:[1.15,1.4]},
   check:{name:'Conferência',vest:'#cdef4f',helmet:'#f1f3ee',hold:'tablet',vmax:[1.0,1.2]},
   tech:{name:'Manutenção',vest:'#e9ef52',helmet:'#3d86da',shirt:'#2f5e93',pants:'#2f5e93',hold:'toolbox',vmax:[1.05,1.25]},
   dock:{name:'Operação de docas',vest:'#ff8a33',helmet:'#f2b21b',vmax:[1.15,1.35]},
   lead:{name:'Supervisão',vest:'#cdef4f',helmet:'#f4f6f2',hold:'tablet',vmax:[1.05,1.25]},
   office:{name:'Análise e controle',vest:null,helmet:null,vmax:[.95,1.15]}};
  const ACTS={scan:{dur:2.8,label:'Lendo etiqueta do palete'},pick:{dur:2.4,label:'Retirando volume da estante'},drop:{dur:2.3,label:'Depositando volume'},inspect:{dur:4.2,label:'Conferindo no tablet'},kneel:{dur:5.6,label:'Manutenção do equipamento'},signal:{dur:3.6,label:'Sinalizando para a doca'},lookaround:{dur:4.2,label:'Observando a operação'},idle:{dur:3.2,label:'Em pausa'}};
  function crew(){
   const list=[
    {role:'pick',pts:[[-55,-26],[-55,-40]],acts:[['drop'],['scan','pick']],face:[null,'side']},
    {role:'pick',pts:[[-37,-26],[-37,-41]],acts:[['drop'],['scan','pick']],face:[null,'side']},
    {role:'pick',pts:[[-61,18],[-61,-12]],acts:[['drop'],['scan','pick']],face:[null,'side']},
    {role:'pick',pts:[[-25,18],[-25,-8]],acts:[['drop'],['scan','pick']],face:[null,'side']},
    {role:'pick',pts:[[27,18],[27,-12]],acts:[['drop'],['scan','pick']],face:[null,'side']},
    {role:'pick',pts:[[45,18],[45,4],[45,-14]],acts:[['drop'],['scan','pick']],face:[null,'side']},
    {role:'lead',pts:[[39,-26],[39,-42]],acts:[['lookaround'],['inspect']],face:[null,'side']},
    {role:'lead',pts:[[15,28.5],[15,-16]],acts:[['lookaround'],['lookaround']],face:[null,null]},
    {role:'check',pts:[[-46,31.5],[-46,47]],acts:[['inspect'],['inspect']],face:[H,-H]},
    {role:'check',pts:[[-6,31.8],[28,31.8]],acts:[['inspect'],['inspect']],face:[-H,-H]},
    {role:'dock',pts:[[-50,48],[-10,48]],acts:[['signal'],['signal']],face:[-H,-H]},
    {role:'dock',pts:[[20,48],[58,48]],acts:[['signal'],['signal']],face:[-H,-H]},
    {role:'tech',pts:[[-1.95,-29],[-1.95,-37]],acts:[['kneel'],['kneel']],face:[0,0]},
    {role:'tech',pts:[[-10,-29],[-10,-37]],acts:[['kneel'],['kneel']],face:[Math.PI,Math.PI]},
    {role:'office',pts:[[52,-31.5],[52,-47]],y:5.13,acts:[['idle'],['lookaround']],face:[null,null]},
    {role:'office',pts:[[56,-37],[64.5,-37]],y:5.13,acts:[['lookaround'],['idle']],face:[null,null]}];
   const SKIN=['#c68a63','#e3b58f','#8e5a3c','#f0c9a5','#a86f4c','#d39f7a'],PANTS=['#2e3d51','#383d45','#44505d','#2b3440','#4a4f3f'],SHIRT=['#4b6e98','#dfe5ea','#58707f','#6c5a86','#3f6b63','#8a4f4f'],HAIR=['#2a1d16','#4a3222','#151515','#7a5a3a'];
   const workers=[];
   list.forEach((c,i)=>{const R=ROLES[c.role],path=c.pts,k=Math.floor(rand()*(path.length-1)),u=rand(),a=path[k],b=path[k+1],x=a[0]+(b[0]-a[0])*u,z=a[1]+(b[1]-a[1])*u,fwd=rand()<.5,wp=fwd?k+1:k,T=path[wp];
    workers.push({id:`COL-${String(i+1).padStart(2,'0')}`,role:c.role,path,acts:c.acts,face:c.face,y0:c.y??.03,x,z,yaw:Math.atan2(-(T[1]-z),T[0]-x),wp,dir:fwd?1:-1,vmax:rr(R.vmax[0],R.vmax[1]),phase:rand()*TAU,carry:c.role==='pick'&&rand()<.5,hold:R.hold||null,seed:rand()*10,sideSign:rand()<.5?1:-1,skin:choice(SKIN),pants:R.pants||choice(PANTS),shirt:R.shirt||choice(SHIRT),hair:choice(HAIR)})});
   return workers;
  }

  // Same order as the prototype's start-up: colliders, ground, architecture, storage,
  // production, office, docks, details, then agents (lanes, fleet, crew).
  makeColliders();makeGround();makeArchitecture();makeStorage();makeProduction();makeOffice();makeDocksAndTrucks();makeDetails();
  drawLanes();
  const workers=crew();
  return {groups, solids, stockCount, LOOPS, VSPEC, FLEET, forklifts: forklifts(), ROLES, ACTS, workers};
}
