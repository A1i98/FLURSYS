struct Params { n:u32, coarse:u32, op:u32, pad:u32, omega:f32, pad1:f32, pad2:f32, pad3:f32 }
@group(0) @binding(0) var<uniform> cfg:Params;
@group(0) @binding(1) var<storage,read> rows:array<u32>;
@group(0) @binding(2) var<storage,read> columns:array<u32>;
@group(0) @binding(3) var<storage,read> values:array<f32>;
@group(0) @binding(4) var<storage,read_write> vectors:array<f32>;
@group(0) @binding(5) var<storage,read> links:array<u32>;
@group(0) @binding(6) var<storage,read_write> child:array<f32>;
@group(0) @binding(7) var<storage,read> inverse:array<f32>;
@compute @workgroup_size(128)
fn main(@builtin(global_invocation_id) gid:vec3<u32>) {
 let i=gid.x;
 if cfg.op==4u {
  if i>=cfg.coarse {return;}
  var sum=0.0;let offset=cfg.n+cfg.coarse+1u;
  for(var k=links[cfg.n+i];k<links[cfg.n+i+1u];k++){sum+=vectors[3u*cfg.n+links[offset+k]];}
  child[cfg.coarse+i]=sum;child[i]=0.0;return;
 }
 if i>=cfg.n {return;}
 switch cfg.op {
 case 0u: {vectors[i]=0.0;}
 case 1u: {
  var ax=0.0;var diagonal=0.0;
  for(var k=rows[i];k<rows[i+1u];k++){ax+=values[k]*vectors[columns[k]];if columns[k]==i {diagonal=values[k];}}
  vectors[2u*cfg.n+i]=vectors[i]+cfg.omega*(vectors[cfg.n+i]-ax)/diagonal;
 }
 case 2u: {vectors[i]=vectors[2u*cfg.n+i];}
 case 3u: {
  var ax=0.0;for(var k=rows[i];k<rows[i+1u];k++){ax+=values[k]*vectors[columns[k]];}
  vectors[3u*cfg.n+i]=vectors[cfg.n+i]-ax;
 }
 case 5u: {vectors[i]+=child[links[i]];}
 case 6u: {var sum=0.0;for(var j=0u;j<cfg.n;j++){sum+=inverse[i*cfg.n+j]*vectors[cfg.n+j];}vectors[i]=sum;}
 default: {}
 }
}
