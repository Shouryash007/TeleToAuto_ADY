// Browser-side dual SO-101 hand teleoperation.
// Port of dual_hand_ik_teleop.py: MediaPipe hand features -> DLS/Jacobian IK.
// Output is compatible with the existing robot-connection.js joint_targets packet.

const JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];
const DEG = 180 / Math.PI;

const DEFAULT_CHAIN = [
  ["shoulder_pan", [0.0388353, -8.97657e-9, 0.0624], [Math.PI, 0, -Math.PI], [-1.91986, 1.91986]],
  ["shoulder_lift", [-0.0303992, -0.0182778, -0.0542], [-Math.PI/2, -Math.PI/2, 0], [-1.74533, 1.74533]],
  ["elbow_flex", [-0.11257, -0.028, 0], [0, 0, Math.PI/2], [-1.69, 1.69]],
  ["wrist_flex", [-0.1349, 0.0052, 0], [0, 0, -Math.PI/2], [-1.65806, 1.65806]],
  ["wrist_roll", [0, -0.0611, 0.0181], [Math.PI/2, 0.0486795, Math.PI], [-2.74385, 2.84121]],
  ["gripper_frame_joint", [-0.0079, -0.000218121, -0.0981274], [0, Math.PI, 0], null, "fixed"]
];

const CFG = {
  home: [0.24, 0, 0.22], homePitch: 0.35,
  range: [0.06, 0.12, 0.06], handSpanX: 0.22, handSpanY: 0.20,
  depthSpan: 0.35, depthSign: 1, lateralSign: 1, rollGain: 1,
  pitchWeight: 0.3, damping: 0.05, iterations: 25, maxJointSpeed: 2.5,
  smoothing: 0.35, lostHoldMs: 500, pinchClosed: 0.25, pinchOpen: 0.95,
  gripperOpen: 1.2, gripperClosed: 0, staleMs: 500,
  swapHands: false
};

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = v => Math.hypot(...v);
const sub = (a,b) => a.map((v,i)=>v-b[i]);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
const wrap = a => (a + Math.PI) % (2*Math.PI) - Math.PI;

function matMul(A,B) {
  const C=Array.from({length:4},()=>[0,0,0,0]);
  for(let i=0;i<4;i++) for(let j=0;j<4;j++) for(let k=0;k<4;k++) C[i][j]+=A[i][k]*B[k][j];
  return C;
}
function rpyMatrix(r,p,y) {
  const cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
          [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];
}
function transform(xyz,rpy) {
  const R=rpyMatrix(...rpy);
  return [[R[0][0],R[0][1],R[0][2],xyz[0]],
          [R[1][0],R[1][1],R[1][2],xyz[1]],
          [R[2][0],R[2][1],R[2][2],xyz[2]],[0,0,0,1]];
}
function rotZ(q) {
  const c=Math.cos(q),s=Math.sin(q);
  return [[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]];
}
function solveLinear(A,b) { // Gaussian elimination with partial pivoting
  const n=b.length, M=A.map((row,i)=>[...row,b[i]]);
  for(let c=0;c<n;c++){
    let pivot=c;
    for(let r=c+1;r<n;r++) if(Math.abs(M[r][c])>Math.abs(M[pivot][c])) pivot=r;
    if(Math.abs(M[pivot][c])<1e-12) return Array(n).fill(0);
    [M[c],M[pivot]]=[M[pivot],M[c]];
    const d=M[c][c]; for(let j=c;j<=n;j++) M[c][j]/=d;
    for(let r=0;r<n;r++) if(r!==c){const f=M[r][c];for(let j=c;j<=n;j++)M[r][j]-=f*M[c][j];}
  }
  return M.map(row=>row[n]);
}
class SO101Kinematics {
  constructor(){
    this.lower=DEFAULT_CHAIN.slice(0,5).map(c=>c[3][0]);
    this.upper=DEFAULT_CHAIN.slice(0,5).map(c=>c[3][1]);
    this.fixed=DEFAULT_CHAIN.map(c=>transform(c[1],c[2]));
    this.pitchSigns=[0,1,1,1,0];
  }
  fkFull(q){
    let T=[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]], axes=[],origins=[],k=0;
    for(let i=0;i<DEFAULT_CHAIN.length;i++){
      T=matMul(T,this.fixed[i]);
      if(i<5){
        axes.push([T[0][2],T[1][2],T[2][2]]);
        origins.push([T[0][3],T[1][3],T[2][3]]);
        T=matMul(T,rotZ(q[k++]));
      }
    }
    return {p:[T[0][3],T[1][3],T[2][3]],axes,origins};
  }
  jacobian(q){
    const {p,axes,origins}=this.fkFull(q);
    return axes.map((a,i)=>cross(a,sub(p,origins[i]))); // columns represented as vectors
  }
  pitch(q){return dot(this.pitchSigns,q);}
  solve(q0,target,roll,neutral){
    const q=q0.slice(), idx=[0,1,2,3];
    for(let iter=0;iter<CFG.iterations;iter++){
      const {p}=this.fkFull(q), e=sub(target,p), err=norm(e);
      const Jcols=this.jacobian(q);
      const pitchErr=(this.targetPitch ?? CFG.homePitch)-this.pitch(q);
      const rows=3+(CFG.pitchWeight>0?1:0);
      const J=Array.from({length:rows},(_,r)=>idx.map(i=>r<3?Jcols[i][r]:CFG.pitchWeight*this.pitchSigns[i]));
      const ev=[...e,...(rows===4?[CFG.pitchWeight*pitchErr]:[])];
      const A=Array.from({length:rows},(_,r)=>Array.from({length:rows},(_,c)=>{
        let s=0;for(let k=0;k<idx.length;k++)s+=J[r][k]*J[c][k];
        return s+(r===c?CFG.damping**2:0);
      }));
      const y=solveLinear(A,ev);
      const dq=idx.map((_,k)=>{let s=0;for(let r=0;r<rows;r++)s+=J[r][k]*y[r];return s;});
      const m=Math.max(...dq.map(Math.abs));
      const scale=m>0.25?0.25/m:1;
      idx.forEach((joint,k)=>q[joint]=clamp(q[joint]+dq[k]*scale,this.lower[joint],this.upper[joint]));
      q[4]=clamp(roll,this.lower[4],this.upper[4]);
      if(err<0.0005 && Math.abs(pitchErr)<0.02) break;
    }
    return q;
  }
}

function features(hand){
  if(!hand || hand.length<21) return null;
  const d=(a,b)=>Math.hypot(hand[a].x-hand[b].x,hand[a].y-hand[b].y);
  const palmIds=[0,5,9,13,17];
  const cx=palmIds.reduce((s,i)=>s+hand[i].x,0)/5;
  const cy=palmIds.reduce((s,i)=>s+hand[i].y,0)/5;
  const size=d(0,9);
  if(size<1e-5) return null;
  const vx=hand[5].x-hand[17].x, vy=hand[5].y-hand[17].y;
  return {cx,cy,size,roll:Math.atan2(vy,vx),pinch:d(4,8)/size};
}
function gaussianHome(kin){
  let q=[0,-1,0.6,0.8,0], target=CFG.home.slice();
  // Find a practical starting configuration by the same DLS iteration.
  for(let n=0;n<400;n++){
    const {p}=kin.fkFull(q), e=sub(target,p), Jcols=kin.jacobian(q);
    const pitchErr=CFG.homePitch-kin.pitch(q);
    const rows=4, J=Array.from({length:4},(_,r)=>[0,1,2,3].map(i=>r<3?Jcols[i][r]:kin.pitchSigns[i]));
    const ev=[...e,pitchErr], A=Array.from({length:4},(_,r)=>Array.from({length:4},(_,c)=>J[r].reduce((s,v,k)=>s+v*J[c][k],0)+(r===c?0.05**2:0)));
    const y=solveLinear(A,ev), dq=[0,1,2,3].map(k=>J.reduce((s,row,r)=>s+row[k]*y[r],0));
    const m=Math.max(...dq.map(Math.abs)), scale=m>0.25?0.25/m:1;
    [0,1,2,3].forEach(k=>q[k]=clamp(q[k]+dq[k]*scale,kin.lower[k],kin.upper[k]));
  }
  return q;
}

export function createDualHandIKController(options={}) {
  Object.assign(CFG,options);
  const kin=new SO101Kinematics();
  const qHome=gaussianHome(kin);
  kin.targetPitch=kin.pitch(qHome);
  const states={
    left:{q:qHome.slice(),refs:null,smoothed:null,lastSeen:0,target:CFG.home.slice(),roll:0,grip:CFG.gripperOpen},
    right:{q:qHome.slice(),refs:null,smoothed:null,lastSeen:0,target:CFG.home.slice(),roll:0,grip:CFG.gripperOpen}
  };
  let lastUpdate=0, lastTime=performance.now();
  const output=()=>Object.fromEntries(["left","right"].map(side=>{
    const s=states[side], a={};
    JOINTS.forEach((n,i)=>a[n]=s.q[i]*DEG);
    // Existing wire protocol represents gripper as a closed boolean.
    a.gripper=clamp((CFG.gripperOpen-s.grip)/(CFG.gripperOpen-CFG.gripperClosed)*100,0,100);
    return [side,a];
  }));
  function updateFromHands(hands,handedness){
    const now=performance.now(), dt=clamp((now-lastTime)/1000,0.001,0.1); lastTime=now;
    let changed=false;
    for(let i=0;i<(hands||[]).length;i++){
      let label=String(handedness?.[i]?.label||"").toLowerCase();
      if(label!=="left"&&label!=="right") continue;
      if(CFG.swapHands) label=label==="left"?"right":"left";
      const side=label, s=states[side], f=features(hands[i]);
      if(!f) continue;
      s.lastSeen=now; changed=true;
      if(!s.refs){s.refs={...f};s.smoothed={...f};}
      const a=CFG.smoothing;
      for(const key of ["cx","cy","size","pinch"]) s.smoothed[key]=(1-a)*s.smoothed[key]+a*f[key];
      s.smoothed.roll=(1-a)*s.smoothed.roll+a*(s.smoothed.roll+wrap(f.roll-s.smoothed.roll));
      const cx=s.smoothed.cx,cy=s.smoothed.cy,size=s.smoothed.size;
      const ux=clamp((cx-s.refs.cx)/CFG.handSpanX,-1,1)*CFG.lateralSign;
      const uz=clamp(-(cy-s.refs.cy)/CFG.handSpanY,-1,1);
      const ud=clamp((size/s.refs.size-1)/CFG.depthSpan,-1,1)*CFG.depthSign;
      s.target=[CFG.home[0]+ud*CFG.range[0],CFG.home[1]-ux*CFG.range[1],CFG.home[2]+uz*CFG.range[2]];
      s.roll=clamp(CFG.rollGain*wrap(s.smoothed.roll-s.refs.roll),kin.lower[4],kin.upper[4]);
      const t=clamp((s.smoothed.pinch-CFG.pinchClosed)/(CFG.pinchOpen-CFG.pinchClosed),0,1);
      s.grip=CFG.gripperClosed+t*(CFG.gripperOpen-CFG.gripperClosed);
    }
    for(const side of ["left","right"]){
      const s=states[side];
      if(s.lastSeen && now-s.lastSeen<=CFG.lostHoldMs){
        const qIk=kin.solve(s.q,s.target,s.roll,qHome);
        const maxStep=CFG.maxJointSpeed*dt;
        s.q=s.q.map((v,i)=>clamp(v+clamp(qIk[i]-v,-maxStep,maxStep),kin.lower[i],kin.upper[i]));
      }
    }
    if(changed) lastUpdate=Date.now();
    return changed;
  }
  return {
    updateFromHands,
    getTargets:output,
    getGripperClosed:()=>({left:states.left.grip<(CFG.gripperOpen+CFG.gripperClosed)/2,right:states.right.grip<(CFG.gripperOpen+CFG.gripperClosed)/2}),
    isFresh:(now=Date.now())=>lastUpdate>0&&now-lastUpdate<=CFG.staleMs,
    markStale:()=>{lastUpdate=0;},
    getJointRadians:()=>Object.fromEntries(["left","right"].map(side=>[side,states[side].q.slice()])),
    getHomeRadians:()=>qHome.slice()
  };
}
