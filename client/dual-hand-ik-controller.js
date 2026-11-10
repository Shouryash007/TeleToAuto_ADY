// Browser-side dual SO-101 hand teleoperation.
// MediaPipe hand features -> DLS/Jacobian IK.
//
// The IK now uses the SAME kinematic chains as static/urdf/humanoid_gazebo.urdf
// (Revolute_23..27 right arm, Revolute_32..36 left arm): the real joint origins
// and joint axes of the simulated, upside-down-mounted arms. Joint angles that
// come out of the solver are therefore exactly the angles the Gazebo position
// controllers need, so no sign/offset hacks are required.
//
// All frames in the URDF arm chains have rpy = 0, so the arm base frame is
// axis-aligned with the robot frame: +X forward, +Y left, +Z up.
// Output is compatible with the existing robot-connection.js joint_targets packet.

const JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];
const DEG = 180 / Math.PI;

// [origin xyz in parent frame, axis in parent frame, [lower, upper]] copied from the URDF.
const ARM_CHAINS = {
  right: {
    joints: [
      [[0.044438, 0.017268, 0.0089],   [0, 0, -1],                         [-1.91986, 1.91986]],   // Revolute_23
      [[0.032911, -0.019478, -0.055376], [0.040768, -0.999169, 0],         [-1.74533, 1.74533]],   // Revolute_24
      [[0.089893, 0.004777, -0.090864],  [0.040768, -0.999169, 0],         [-1.69, 1.69]],         // Revolute_25
      [[0.102006, 0.003837, -0.103859],  [0.040768, -0.999169, 0],         [-1.65806, 1.65806]],   // Revolute_26
      [[0.050125, 0.01784, -0.036048],   [0.824037, 0.033658, -0.565536],  [-2.74385, 2.84121]]    // Revolute_27
    ],
    tip: [0.008115, -0.01324, -0.033824]                                     // Revolute_28 (gripper) origin
  },
  left: {
    joints: [
      [[0.044438, -0.017631, 0.0089],  [0, 0, -1],                         [-1.91986, 1.91986]],   // Revolute_32
      [[0.03335, -0.018609, -0.055376],  [0.064042, -0.997947, 0],         [-1.74533, 1.74533]],   // Revolute_33
      [[0.092599, 0.007053, -0.087951],  [0.064042, -0.997947, 0],         [-1.69, 1.69]],         // Revolute_34
      [[0.105215, 0.006427, -0.100476],  [0.064042, -0.997947, 0],         [-1.65806, 1.65806]],   // Revolute_35
      [[0.052062, 0.019155, -0.032446],  [0.860105, 0.055233, -0.507119],  [-2.74385, 2.84121]]    // Revolute_36
    ],
    tip: [0.008399, 0.010827, -0.034604]                                     // Revolute_37 (gripper) origin
  }
};

const CFG = {
  // Neutral gripper position, measured from the shoulder-pan joint origin (robot frame:
  // +X forward, +Y left, +Z up), and the neutral tool pitch (sum of lift+elbow+wrist_flex, rad).
  // Home / range were chosen by measuring this arm's reachable workspace: with the tool
  // pitch left free (pitchWeight 0) the arm can reach about +-0.11 m forward/back,
  // +-0.25 m sideways and +-0.11 m up/down around this home pose.
  home: [0.28, 0.012, -0.24], homePitch: 0.4,
  range: [0.09, 0.20, 0.09], handSpanX: 0.22, handSpanY: 0.20,
  depthSpan: 0.35, depthSign: 1, lateralSign: 1, rollGain: 1, rollSign: 1,
  pitchWeight: 0, nullGain: 0.05, damping: 0.05, iterations: 25, maxJointSpeed: 3.0,
  // Input filtering. Lower smoothing = smoother but laggier.
  smoothing: 0.25, deadband: 0.004, lostHoldMs: 500,
  pinchClosed: 0.25, pinchOpen: 0.95,
  gripperOpen: 1.2, gripperClosed: 0, staleMs: 500,
  // MediaPipe assumes a mirrored (selfie) image. The tracker feeds the raw, un-mirrored
  // webcam frame, so MediaPipe's "Left" label is the operator's RIGHT hand. Swapping makes the
  // operator's right hand drive the robot's right arm. Set false if your arms appear crossed.
  swapHands: true
};

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = v => Math.hypot(...v);
const sub = (a,b) => a.map((v,i)=>v-b[i]);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const dot = (a,b) => a.reduce((s,v,i)=>s+v*b[i],0);
// Proper wrap to [-PI, PI). (JS % keeps the sign of the dividend, so the old
// "(a+PI)%(2PI)-PI" version did not wrap values below -PI; that let the wrist
// roll reference drift by 2*PI and made the joint spin.)
const TWO_PI = 2*Math.PI;
const wrap = a => ((a + Math.PI) % TWO_PI + TWO_PI) % TWO_PI - Math.PI;

function matVec(R,v){return [dot(R[0],v),dot(R[1],v),dot(R[2],v)];}
function matMul3(A,B){
  return A.map((row,i)=>[0,1,2].map(j=>row[0]*B[0][j]+row[1]*B[1][j]+row[2]*B[2][j]));
}
// Rodrigues rotation about a unit axis.
function axisAngle(a,q){
  const c=Math.cos(q), s=Math.sin(q), t=1-c, [x,y,z]=a;
  return [[t*x*x+c,   t*x*y-s*z, t*x*z+s*y],
          [t*x*y+s*z, t*y*y+c,   t*y*z-s*x],
          [t*x*z-s*y, t*y*z+s*x, t*z*z+c]];
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

class ArmKinematics {
  constructor(chain){
    this.origins=chain.joints.map(j=>j[0]);
    this.axes=chain.joints.map(j=>{const n=norm(j[1]);return j[1].map(v=>v/n);});
    this.lower=chain.joints.map(j=>j[2][0]);
    this.upper=chain.joints.map(j=>j[2][1]);
    this.tip=chain.tip;
    // lift, elbow and wrist_flex share one axis, so tool pitch is the sum of those angles.
    this.pitchSigns=[0,1,1,1,0];
    this.targetPitch=CFG.homePitch;
  }
  fkFull(q){
    let R=[[1,0,0],[0,1,0],[0,0,1]], p=[0,0,0];
    const axes=[], origins=[];
    for(let i=0;i<5;i++){
      const o=matVec(R,this.origins[i]);
      p=[p[0]+o[0],p[1]+o[1],p[2]+o[2]];
      axes.push(matVec(R,this.axes[i]));
      origins.push(p.slice());
      R=matMul3(R,axisAngle(this.axes[i],q[i]));
    }
    const t=matVec(R,this.tip);
    return {p:[p[0]+t[0],p[1]+t[1],p[2]+t[2]],axes,origins};
  }
  jacobian(q){
    const {p,axes,origins}=this.fkFull(q);
    return axes.map((a,i)=>cross(a,sub(p,origins[i]))); // columns represented as vectors
  }
  pitch(q){return dot(this.pitchSigns,q);}
  solve(q0,target,roll,qNeutral=null){
    const q=q0.slice(), idx=[0,1,2,3];
    for(let iter=0;iter<CFG.iterations;iter++){
      const {p}=this.fkFull(q), e=sub(target,p), err=norm(e);
      const Jcols=this.jacobian(q);
      const pitchErr=this.targetPitch-this.pitch(q);
      const rows=3+(CFG.pitchWeight>0?1:0);
      const J=Array.from({length:rows},(_,r)=>idx.map(i=>r<3?Jcols[i][r]:CFG.pitchWeight*this.pitchSigns[i]));
      const ev=[...e,...(rows===4?[CFG.pitchWeight*pitchErr]:[])];
      const A=Array.from({length:rows},(_,r)=>Array.from({length:rows},(_,c)=>{
        let s=0;for(let k=0;k<idx.length;k++)s+=J[r][k]*J[c][k];
        return s+(r===c?CFG.damping**2:0);
      }));
      const y=solveLinear(A,ev);
      let dq=idx.map((_,k)=>{let s=0;for(let r=0;r<rows;r++)s+=J[r][k]*y[r];return s;});
      if(qNeutral && CFG.nullGain>0 && rows<idx.length){
        // Null-space pull toward the neutral posture: keeps the redundant joint
        // (wrist_flex) from drifting while the gripper position is tracked.
        const v=idx.map(k=>CFG.nullGain*(qNeutral[k]-q[k]));
        const Jv=J.map(row=>row.reduce((s,x,k)=>s+x*v[k],0));
        const z=solveLinear(A,Jv);
        dq=dq.map((d,k)=>d+v[k]-J.reduce((s,row,r)=>s+row[k]*z[r],0));
      }
      const m=Math.max(...dq.map(Math.abs));
      const scale=m>0.25?0.25/m:1;
      idx.forEach((joint,k)=>q[joint]=clamp(q[joint]+dq[k]*scale,this.lower[joint],this.upper[joint]));
      q[4]=clamp(roll,this.lower[4],this.upper[4]);
      if(err<0.0005 && Math.abs(pitchErr)<0.02) break;
    }
    return q;
  }
  // Find the neutral joint configuration for the configured home pose.
  homeConfiguration(){
    const q=[0,0.3,0.3,0,0], target=CFG.home.slice();
    this.targetPitch=CFG.homePitch;
    for(let n=0;n<400;n++){
      const {p}=this.fkFull(q), e=sub(target,p), Jcols=this.jacobian(q);
      const pitchErr=CFG.homePitch-this.pitch(q);
      const J=Array.from({length:4},(_,r)=>[0,1,2,3].map(i=>r<3?Jcols[i][r]:this.pitchSigns[i]));
      const ev=[...e,pitchErr];
      const A=Array.from({length:4},(_,r)=>Array.from({length:4},(_,c)=>J[r].reduce((s,v,k)=>s+v*J[c][k],0)+(r===c?0.05**2:0)));
      const y=solveLinear(A,ev), dq=[0,1,2,3].map(k=>J.reduce((s,row,r)=>s+row[k]*y[r],0));
      const m=Math.max(...dq.map(Math.abs)), scale=m>0.25?0.25/m:1;
      [0,1,2,3].forEach(k=>q[k]=clamp(q[k]+dq[k]*scale,this.lower[k],this.upper[k]));
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

export function createDualHandIKController(options={}) {
  Object.assign(CFG,options);
  const kins={left:new ArmKinematics(ARM_CHAINS.left),right:new ArmKinematics(ARM_CHAINS.right)};
  const qHomes={left:kins.left.homeConfiguration(),right:kins.right.homeConfiguration()};
  for(const side of ["left","right"]) kins[side].targetPitch=kins[side].pitch(qHomes[side]);
  const mkState=side=>({q:qHomes[side].slice(),refs:null,smoothed:null,lastSeen:0,
    target:CFG.home.slice(),roll:0,grip:CFG.gripperOpen,closed:false});
  const states={left:mkState("left"),right:mkState("right")};
  let lastUpdate=0, lastTime=performance.now();
  const gripPercent=s=>clamp((CFG.gripperOpen-s.grip)/(CFG.gripperOpen-CFG.gripperClosed)*100,0,100);
  const output=()=>Object.fromEntries(["left","right"].map(side=>{
    const s=states[side], a={};
    JOINTS.forEach((n,i)=>a[n]=s.q[i]*DEG);
    a.gripper=gripPercent(s);
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
      // Angular smoothing on the shortest path, kept wrapped so it can never drift by 2*PI.
      s.smoothed.roll=wrap(s.smoothed.roll+a*wrap(f.roll-s.smoothed.roll));
      const cx=s.smoothed.cx,cy=s.smoothed.cy,size=s.smoothed.size;
      // Small dead-band so camera/landmark jitter around the reference does not move the arm.
      const db=v=>Math.abs(v)<CFG.deadband?0:v;
      const ux=clamp(db(cx-s.refs.cx)/CFG.handSpanX,-1,1)*CFG.lateralSign;
      const uz=clamp(-db(cy-s.refs.cy)/CFG.handSpanY,-1,1);
      const ud=clamp((size/s.refs.size-1)/CFG.depthSpan,-1,1)*CFG.depthSign;
      // Robot frame: +X forward, +Y left, +Z up. The raw (un-mirrored) camera image has the
      // operator's right hand at smaller cx, so +ux (larger cx) maps to +Y (left) on the robot.
      // The reachable volume is ellipsoidal, not a box: limit the combined hand offset to the
      // unit ball so diagonal hand moves (e.g. far + up + sideways) stay reachable.
      let off=[ud,ux,uz]; const len=Math.hypot(...off); if(len>1) off=off.map(v=>v/len);
      s.target=[CFG.home[0]+off[0]*CFG.range[0],CFG.home[1]+off[1]*CFG.range[1],CFG.home[2]+off[2]*CFG.range[2]];
      const kin=kins[side];
      s.roll=clamp(CFG.rollSign*CFG.rollGain*wrap(s.smoothed.roll-s.refs.roll),kin.lower[4],kin.upper[4]);
      const t=clamp((s.smoothed.pinch-CFG.pinchClosed)/(CFG.pinchOpen-CFG.pinchClosed),0,1);
      s.grip=CFG.gripperClosed+t*(CFG.gripperOpen-CFG.gripperClosed);
      // Hysteresis so the gripper does not chatter around the threshold.
      if(!s.closed && t<0.35) s.closed=true; else if(s.closed && t>0.65) s.closed=false;
    }
    for(const side of ["left","right"]){
      const s=states[side], kin=kins[side];
      if(s.lastSeen && now-s.lastSeen<=CFG.lostHoldMs){
        const qIk=kin.solve(s.q,s.target,s.roll,qHomes[side]);
        const maxStep=CFG.maxJointSpeed*dt;
        s.q=s.q.map((v,k)=>clamp(v+clamp(qIk[k]-v,-maxStep,maxStep),kin.lower[k],kin.upper[k]));
      }
    }
    if(changed) lastUpdate=Date.now();
    return changed;
  }
  return {
    updateFromHands,
    getTargets:output,
    getGripperClosed:()=>({left:states.left.closed,right:states.right.closed}),
    isFresh:(now=Date.now())=>lastUpdate>0&&now-lastUpdate<=CFG.staleMs,
    markStale:()=>{lastUpdate=0;},
    getJointRadians:()=>Object.fromEntries(["left","right"].map(side=>[side,states[side].q.slice()])),
    getHomeRadians:()=>qHomes.right.slice(),
    getHomeRadiansBySide:()=>({left:qHomes.left.slice(),right:qHomes.right.slice()})
  };
}

// Exposed for offline tuning/tests.
export { ArmKinematics, ARM_CHAINS, CFG };
