struct Params { n:u32, op:u32, a:u32, b:u32, alpha:f32, beta:f32, omega:f32, pad:f32 }
@group(0) @binding(0) var<uniform> cfg:Params;
@group(0) @binding(1) var<storage,read> rows:array<u32>;
@group(0) @binding(2) var<storage,read> columns:array<u32>;
@group(0) @binding(3) var<storage,read> values:array<f32>;
@group(0) @binding(4) var<storage,read_write> state:array<f32>;
@group(0) @binding(5) var<storage,read_write> partial:array<f32>;
var<workgroup> sums:array<f32,256>;
fn at(slot:u32,i:u32)->u32 {return slot*cfg.n+i;}
@compute @workgroup_size(256)
fn main(@builtin(global_invocation_id) gid:vec3<u32>,@builtin(local_invocation_id) lid:vec3<u32>,@builtin(workgroup_id) group:vec3<u32>) {
 let i=gid.x;
 if cfg.op==7u {
  var value=0.0;
  if i<cfg.n {value=state[at(cfg.a,i)]*state[at(cfg.b,i)];}
  sums[lid.x]=value;workgroupBarrier();
  var stride=128u;
  loop {if lid.x<stride {sums[lid.x]+=sums[lid.x+stride];}workgroupBarrier();if stride==1u {break;}stride/=2u;}
  if lid.x==0u {partial[group.x]=sums[0];}return;
 }
 if i>=cfg.n {return;}
 switch cfg.op {
 case 0u: {
  state[at(0u,i)]=0.0;state[at(2u,i)]=state[at(1u,i)];state[at(3u,i)]=0.0;state[at(4u,i)]=0.0;
  var diagonal=0.0;for(var k=rows[i];k<rows[i+1u];k++){if columns[k]==i {diagonal=values[k];}}
  state[at(9u,i)]=1.0/diagonal;
 }
 case 1u: {
  let p=state[at(1u,i)]+cfg.beta*(state[at(3u,i)]-cfg.omega*state[at(4u,i)]);
  state[at(3u,i)]=p;state[at(7u,i)]=p*state[at(9u,i)];
 }
 case 2u,4u: {
  var sum=0.0;let source=select(7u,8u,cfg.op==4u);let destination=select(4u,6u,cfg.op==4u);
  for(var k=rows[i];k<rows[i+1u];k++){sum+=values[k]*state[at(source,columns[k])];}
  state[at(destination,i)]=sum;
 }
 case 3u: {let s=state[at(1u,i)]-cfg.alpha*state[at(4u,i)];state[at(5u,i)]=s;state[at(8u,i)]=s*state[at(9u,i)];}
 case 5u: {state[at(0u,i)]+=cfg.alpha*state[at(7u,i)]+cfg.omega*state[at(8u,i)];state[at(1u,i)]=state[at(5u,i)]-cfg.omega*state[at(6u,i)];}
 case 8u: {
  var ax=0.0;for(var k=rows[i];k<rows[i+1u];k++){ax+=values[k]*state[at(0u,columns[k])];}
  state[at(7u,i)]=state[at(0u,i)]+0.6666667*(state[at(2u,i)]-ax)*state[at(9u,i)];
 }
 case 9u: {state[at(0u,i)]=state[at(7u,i)];}
 case 6u: {state[at(0u,i)]+=cfg.alpha*state[at(7u,i)];state[at(1u,i)]=state[at(5u,i)];}
 default: {}
 }
}
