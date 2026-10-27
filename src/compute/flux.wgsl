struct Params { count:u32, scheme:u32, gamma:f32, pad:u32 }
struct State { rho:f32, u:vec3<f32>, p:f32 }
struct Flux { a:vec4<f32>, e:f32 }
@group(0) @binding(0) var<storage,read> inputs:array<f32>;
@group(0) @binding(1) var<storage,read_write> outputs:array<f32>;
@group(0) @binding(2) var<uniform> params:Params;
fn add(a:Flux,b:Flux)->Flux{return Flux(a.a+b.a,a.e+b.e);}
fn sub(a:Flux,b:Flux)->Flux{return Flux(a.a-b.a,a.e-b.e);}
fn scale(a:Flux,s:f32)->Flux{return Flux(a.a*s,a.e*s);}
fn conservative(p:State)->Flux{return Flux(vec4<f32>(p.rho,p.rho*p.u),p.p/(params.gamma-1.0)+0.5*p.rho*dot(p.u,p.u));}
fn physical(p:State,n:vec3<f32>)->Flux{
 let q=conservative(p);let un=dot(p.u,n);
 return Flux(vec4<f32>(q.a.x*un,q.a.yzw*un+p.p*n),(q.e+p.p)*un);
}
fn sound(p:State)->f32{return sqrt(params.gamma*p.p/p.rho);}
fn rusanov(l:State,r:State,n:vec3<f32>)->Flux{
 let speed=max(abs(dot(l.u,n))+sound(l),abs(dot(r.u,n))+sound(r));
 return sub(scale(add(physical(l,n),physical(r,n)),0.5),scale(sub(conservative(r),conservative(l)),0.5*speed));
}
fn finite(v:f32)->bool{return v==v && abs(v)<=3.4028234e38;}
fn valid(f:Flux)->bool{
 if !all(f.a==f.a)||!finite(f.e)||f.a.x<=0.0{return false;}
 return f.e-0.5*dot(f.a.yzw,f.a.yzw)/f.a.x>0.0;
}
@compute @workgroup_size(128)
fn main(@builtin(global_invocation_id) id:vec3<u32>){
 let i=id.x;if i>=params.count{return;}
 let k=i*16u;
 let l=State(inputs[k],vec3<f32>(inputs[k+1u],inputs[k+2u],inputs[k+3u]),inputs[k+4u]);
 let r=State(inputs[k+5u],vec3<f32>(inputs[k+6u],inputs[k+7u],inputs[k+8u]),inputs[k+9u]);
 let n=vec3<f32>(inputs[k+10u],inputs[k+11u],inputs[k+12u]);
 var f=rusanov(l,r,n);var fallback=0.0;
 if params.scheme!=0u{
  let ul=dot(l.u,n);let ur=dot(r.u,n);
  let sl=min(ul-sound(l),ur-sound(r));let sr=max(ul+sound(l),ur+sound(r));
  if sl>=0.0{f=physical(l,n);}else if sr<=0.0{f=physical(r,n);}else if params.scheme==2u{
   f=scale(add(sub(scale(physical(l,n),sr),scale(physical(r,n),sl)),scale(sub(conservative(r),conservative(l)),sl*sr)),1.0/(sr-sl));
  }else{
   let den=l.rho*(sl-ul)-r.rho*(sr-ur);
   let sm=(r.p-l.p+l.rho*ul*(sl-ul)-r.rho*ur*(sr-ur))/den;
   var p=l;var wave=sl;var un=ul;
   if sm<0.0{p=r;wave=sr;un=ur;}
   let rho=p.rho*(wave-un)/(wave-sm);
   let velocity=p.u+n*(sm-un);
   let energy=rho*(conservative(p).e/p.rho+(sm-un)*(sm+p.p/(p.rho*(wave-un))));
   let star=Flux(vec4<f32>(rho,rho*velocity),energy);
   if finite(sm)&&valid(star){
    let candidate=add(physical(p,n),scale(sub(star,conservative(p)),wave));
    if all(abs(candidate.a)<=vec4<f32>(3.4028234e38))&&finite(candidate.e){f=candidate;}else{fallback=1.0;}
   }else{fallback=1.0;}
  }
 }
 let j=i*8u;
 outputs[j]=f.a.x;outputs[j+1u]=f.a.y;outputs[j+2u]=f.a.z;outputs[j+3u]=f.a.w;outputs[j+4u]=f.e;outputs[j+5u]=fallback;outputs[j+6u]=0.0;outputs[j+7u]=0.0;
}
