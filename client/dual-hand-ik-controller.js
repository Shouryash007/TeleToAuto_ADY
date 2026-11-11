// Browser-side dual SO-101 hand teleoperation.
// MediaPipe hand features -> DLS/Jacobian IK.
//
// The IK uses the SAME kinematic chains as static/urdf/humanoid_gazebo.urdf
// (Revolute_23..27 right arm, Revolute_32..36 left arm): the real joint origins
// and joint axes of the simulated, upside-down-mounted arms. Joint angles that
// come out of the solver are therefore exactly the angles the Gazebo position
// controllers need, so no sign/offset hacks are required.
//
// All frames in the URDF arm chains have rpy = 0, so the arm base frame is
// axis-aligned with the robot frame: +X forward, +Y left, +Z up.
// Output is compatible with the existing robot-connection.js joint_targets packet.
//
// NEW in this version
//  * Quality-aware input: hands arrive from hand-tracker.js with quality / status /
//    pinchQuality / reacquired. Low-quality frames are ignored, smoothing weakens with
//    quality, the gripper HOLDS while fingertips are unreliable (occluded / coasting).
//  * Re-acquire ramp: after an occlusion the arm target glides (rate-limited) to the
//    hand's new position instead of snapping. A global target speed limit is applied too.
//  * User <-> robot calibration (see startCalibration / sync):
//      NEUTRAL   - operator holds a relaxed hand still; that pose becomes the robot's home pose.
//      WORKSPACE - operator sweeps the hand to its comfortable extremes; the per-axis
//                  (asymmetric) spans are fitted so the full robot range maps onto the
//                  operator's own reach.
//      USER-SYNC - one-shot re-centre ("clutch"): re-references the hand without moving the
//                  robot (anchor "current") or glides it to home (anchor "home").
//    Calibration is persisted in localStorage and auto-loaded.

const JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"];
const SIDES = ["left", "right"];
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
  range: [0.09, 0.20, 0.09], handSpanX: 0.22, handSpanY: 0.20,   // spans = UNCALIBRATED defaults
  depthSpan: 0.35, depthSign: 1, lateralSign: 1, rollGain: 1, rollSign: 1,
  pitchWeight: 0, nullGain: 0.05, damping: 0.05, iterations: 25, maxJointSpeed: 3.0,
  // Input filtering. Lower smoothing = smoother but laggier.
  smoothing: 0.25, deadband: 0.004, lostHoldMs: 500,
  pinchClosed: 0.25, pinchOpen: 0.95,
  gripperOpen: 1.2, gripperClosed: 0, staleMs: 500,
  // MediaPipe assumes a mirrored (selfie) image. The tracker feeds the raw, un-mirrored
  // webcam frame, so MediaPipe's "Left" label is the operator's RIGHT hand. Swapping makes the
  // operator's right hand drive the robot's right arm. Set false if your arms appear crossed.
  swapHands: true,

  // --- quality / occlusion handling (values come from hand-tracker.js) ---
  minQuality: 0.25,          // frames below this are ignored (arm holds its last pose)
  minPinchQuality: 0.5,      // below this the gripper holds instead of following the pinch
  maxTargetSpeed: 1.5,       // m/s limit on the IK target (safety / anti-teleport)
  reacquireTargetSpeed: 0.2, // m/s right after a hand re-appears ...
  reacquireRampMs: 800,      // ... blending up to maxTargetSpeed over this long
  reacquireGapMs: 400,       // a gap in observations longer than this counts as re-acquire
  motionScale: 1,            // 0..1: scale the whole hand->robot motion (precision mode)

  // --- calibration ---
  autoLoadCalibration: true,
  calib: {
    storageKey: "dualHandIK.calibration.v1",
    minQuality: 0.6,         // only high-quality frames are used for calibration
    neutralHoldMs: 2000,     // how long the hand must stay still in the NEUTRAL step
    stillTol: 0.02,          // max drift (image units) allowed during the hold
    workspaceMs: 9000,       // length of the WORKSPACE sweep
    minSamples: 60,
    percentile: 0.03,        // robust extremes: ignore the most extreme 3% each side
    spanShrink: 0.9,         // reach 100% of the robot range at 90% of the operator's extreme
    minSpan: 0.03, maxSpan: 0.45,           // image units
    minDepthSpan: 0.06, maxDepthSpan: 0.8,  // palm-size ratio
    timeoutMs: 20000         // give up on the NEUTRAL step if no steady hand shows up
  }
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

// Asymmetric normalisation: d>=0 uses the positive span, d<0 the negative span -> [-1, 1].
const axisNorm = (d, neg, pos) => d >= 0 ? clamp(d/pos, 0, 1) : clamp(d/neg, -1, 0);
const percentile = (arr, p) => {
  const a = [...arr].sort((x,y)=>x-y);
  return a[clamp(Math.floor(p*(a.length-1)), 0, a.length-1)];
};
// Move `a` toward `b` by at most `maxDist`.
function stepToward(a, b, maxDist){
  const d = sub(b, a), n = norm(d);
  if(n<=maxDist || n<1e-9) return b.slice();
  return a.map((v,i)=>v+d[i]*maxDist/n);
}

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
  // `onCalibration` is a callback, not a tunable: keep it out of CFG.
  const {onCalibration, calib:calibOpts, ...cfgOptions}=options;
  Object.assign(CFG,cfgOptions);
  if(calibOpts) CFG.calib={...CFG.calib,...calibOpts};

  const kins={left:new ArmKinematics(ARM_CHAINS.left),right:new ArmKinematics(ARM_CHAINS.right)};
  const qHomes={left:kins.left.homeConfiguration(),right:kins.right.homeConfiguration()};
  for(const side of SIDES) kins[side].targetPitch=kins[side].pitch(qHomes[side]);

  const defaultSpans=()=>({
    xNeg:CFG.handSpanX, xPos:CFG.handSpanX,     // image x: + = operator moves hand to THEIR LEFT
    yUp:CFG.handSpanY,  yDown:CFG.handSpanY,
    dNear:CFG.depthSpan, dFar:CFG.depthSpan     // palm-size ratio: + = closer to the camera
  });
  const mkState=side=>({q:qHomes[side].slice(),refs:null,smoothed:null,lastSeen:0,
    target:CFG.home.slice(),desired:CFG.home.slice(),origin:CFG.home.slice(),
    roll:0,grip:CFG.gripperOpen,closed:false,
    spans:defaultSpans(),calibrated:false,quality:0,ramp:0,frozen:false});
  const states={left:mkState("left"),right:mkState("right")};
  let lastUpdate=0, lastTime=performance.now();

  const gripPercent=s=>clamp((CFG.gripperOpen-s.grip)/(CFG.gripperOpen-CFG.gripperClosed)*100,0,100);
  const output=()=>Object.fromEntries(SIDES.map(side=>{
    const s=states[side], a={};
    JOINTS.forEach((n,i)=>a[n]=s.q[i]*DEG);
    a.gripper=gripPercent(s);
    return [side,a];
  }));

  // ------------------------------------------------------------------
  // Calibration
  // ------------------------------------------------------------------
  const calib={active:false,phase:"idle",message:"",t0:0,phaseT0:0,lastEmit:0,sides:[],
    buf:{left:[],right:[]},ws:{left:[],right:[]},ext:{left:null,right:null},
    done:{left:false,right:false},progress:{left:0,right:0}};

  function emitCal(force=false){
    const now=performance.now();
    if(!force && now-calib.lastEmit<100) return;
    calib.lastEmit=now;
    const sideInfo=Object.fromEntries(SIDES.map(sd=>{
      const e=calib.ext[sd], m=CFG.calib.minSpan;
      return [sd,{
        done:calib.done[sd], progress:calib.progress[sd], calibrated:states[sd].calibrated,
        // Workspace coverage in operator terms: which directions have been reached so far.
        coverage:(calib.phase==="workspace"&&e)?{
          left:e.xmax>m, right:-e.xmin>m, up:e.ymax>m, down:-e.ymin>m,
          closer:e.smax>CFG.calib.minDepthSpan, farther:-e.smin>CFG.calib.minDepthSpan}:null
      }];
    }));
    const p=calib.sides.length?Math.min(...calib.sides.map(sd=>calib.progress[sd])):0;
    onCalibration?.({active:calib.active,phase:calib.phase,progress:p,message:calib.message,sides:sideInfo});
  }

  function startCalibration(sides=SIDES){
    const now=performance.now();
    calib.active=true; calib.phase="neutral"; calib.t0=calib.phaseT0=now;
    calib.sides=sides.filter(sd=>SIDES.includes(sd));
    for(const sd of SIDES){
      calib.buf[sd]=[]; calib.ws[sd]=[]; calib.ext[sd]=null;
      calib.done[sd]=false; calib.progress[sd]=0;
    }
    calib.message="NEUTRAL: hold a relaxed, comfortable open hand still in front of the camera";
    emitCal(true);
  }

  function cancelCalibration(message="CALIBRATION CANCELLED"){
    calib.active=false; calib.phase="idle"; calib.message=message;
    emitCal(true);
  }

  function feedCalibration(side,f,q,now){
    if(!calib.active || !calib.sides.includes(side) || q<CFG.calib.minQuality) return;
    const s=states[side];
    if(calib.phase==="neutral"){
      if(calib.done[side]) return;
      const buf=calib.buf[side];
      if(buf.length && now-buf[buf.length-1].t>300) buf.length=0;     // hand was lost: start over
      if(buf.length){
        const mx=buf.reduce((a,b)=>a+b.cx,0)/buf.length, my=buf.reduce((a,b)=>a+b.cy,0)/buf.length;
        if(Math.hypot(f.cx-mx,f.cy-my)>CFG.calib.stillTol) buf.length=0; // moved: start over
      }
      buf.push({t:now,cx:f.cx,cy:f.cy,size:f.size,sin:Math.sin(f.roll),cos:Math.cos(f.roll)});
      calib.progress[side]=clamp((now-buf[0].t)/CFG.calib.neutralHoldMs,0,1);
      if(calib.progress[side]>=1){
        const n=buf.length, avg=k=>buf.reduce((a,b)=>a+b[k],0)/n;
        s.refs={cx:avg("cx"),cy:avg("cy"),size:avg("size"),roll:Math.atan2(avg("sin"),avg("cos"))};
        s.spans=defaultSpans(); s.calibrated=false;
        s.origin=CFG.home.slice();            // neutral hand pose == robot home pose
        s.ramp=CFG.reacquireRampMs/1000;      // glide to home once the freeze ends
        calib.done[side]=true;
      }
    } else if(calib.phase==="workspace" && s.refs){
      const dx=f.cx-s.refs.cx, dy=-(f.cy-s.refs.cy), ds=f.size/s.refs.size-1;
      if(calib.ws[side].length<5000) calib.ws[side].push({dx,dy,ds});
      const e=calib.ext[side]||(calib.ext[side]={xmin:0,xmax:0,ymin:0,ymax:0,smin:0,smax:0});
      e.xmin=Math.min(e.xmin,dx); e.xmax=Math.max(e.xmax,dx);
      e.ymin=Math.min(e.ymin,dy); e.ymax=Math.max(e.ymax,dy);
      e.smin=Math.min(e.smin,ds); e.smax=Math.max(e.smax,ds);
    }
  }

  function finalizeWorkspace(){
    const C=CFG.calib, warn=[];
    for(const side of calib.sides){
      const w=calib.ws[side], s=states[side];
      if(w.length<C.minSamples){ warn.push(side); continue; }
      const col=k=>w.map(x=>x[k]);
      const dx=col("dx"), dy=col("dy"), ds=col("ds");
      const hi=a=>percentile(a,1-C.percentile), lo=a=>percentile(a,C.percentile);
      // raw extreme -> span; if the operator never really moved on an axis keep the default.
      const fin=(raw,def,min,max)=>raw>=min?clamp(raw*C.spanShrink,min,max):def;
      s.spans={
        xPos:fin(hi(dx),CFG.handSpanX,C.minSpan,C.maxSpan),  xNeg:fin(-lo(dx),CFG.handSpanX,C.minSpan,C.maxSpan),
        yUp:fin(hi(dy),CFG.handSpanY,C.minSpan,C.maxSpan),   yDown:fin(-lo(dy),CFG.handSpanY,C.minSpan,C.maxSpan),
        dNear:fin(hi(ds),CFG.depthSpan,C.minDepthSpan,C.maxDepthSpan),
        dFar:fin(-lo(ds),CFG.depthSpan,C.minDepthSpan,C.maxDepthSpan)
      };
      s.calibrated=true;
    }
    calib.active=false; calib.phase="done";
    calib.message=warn.length
      ? `CALIBRATION PARTIAL: not enough hand data for ${warn.join(" + ")}`
      : "CALIBRATION COMPLETE";
    for(const sd of calib.sides) calib.progress[sd]=1;
    saveCalibration();
    emitCal(true);
  }

  function tickCalibration(now){
    if(!calib.active) return;
    if(calib.phase==="neutral"){
      if(calib.sides.every(sd=>calib.done[sd])){
        calib.phase="workspace"; calib.phaseT0=now;
        for(const sd of SIDES) calib.progress[sd]=0;
        calib.message="WORKSPACE: sweep your hand left, right, up, down, closer and farther";
        emitCal(true);
      } else if(now-calib.phaseT0>CFG.calib.timeoutMs){
        const missing=calib.sides.filter(sd=>!calib.done[sd]).join(" + ");
        cancelCalibration(`CALIBRATION TIMED OUT: no steady hand for ${missing}`);
      } else emitCal();
    } else if(calib.phase==="workspace"){
      const p=clamp((now-calib.phaseT0)/CFG.calib.workspaceMs,0,1);
      for(const sd of calib.sides) calib.progress[sd]=p;
      if(p>=1) finalizeWorkspace(); else emitCal();
    }
  }

  // USER-SYNC: re-reference the hand to the robot. Needs a currently visible hand.
  //  anchor "current": robot stays where it is, the hand's present pose becomes the new zero
  //                    (use like lifting a mouse to re-centre; no motion at all).
  //  anchor "home":    hand's present pose is mapped to the robot home pose; the arm glides there.
  function sync({sides=SIDES,anchor="current",delayMs=0}={}){
    const run=()=>{
      const now=performance.now(), synced=[];
      for(const side of sides){
        const s=states[side];
        if(!SIDES.includes(side) || !s.smoothed || !s.lastSeen || now-s.lastSeen>CFG.lostHoldMs) continue;
        s.refs={cx:s.smoothed.cx,cy:s.smoothed.cy,size:s.smoothed.size,roll:s.smoothed.roll};
        if(anchor==="home"){
          s.origin=CFG.home.slice();
          s.desired=s.origin.slice();
          s.ramp=CFG.reacquireRampMs/1000;
        } else {
          const p=kins[side].fkFull(s.q).p;
          s.origin=p.map((v,k)=>clamp(v,CFG.home[k]-CFG.range[k],CFG.home[k]+CFG.range[k]));
          s.desired=s.origin.slice();
          s.target=s.origin.slice();
        }
        synced.push(side);
      }
      calib.message=synced.length?`SYNCED ${synced.join(" + ").toUpperCase()}`:"SYNC FAILED: show your hand to the camera";
      emitCal(true);
      return synced;
    };
    if(delayMs>0){
      calib.message=`SYNC in ${Math.ceil(delayMs/1000)} s: hold the pose that should match the robot`;
      emitCal(true);
      return new Promise(res=>setTimeout(()=>res(run()),delayMs));
    }
    return Promise.resolve(run());
  }

  // ---- persistence ----
  function exportCalibration(){
    return {version:1,savedAt:Date.now(),sides:Object.fromEntries(
      SIDES.filter(sd=>states[sd].refs&&states[sd].calibrated)
           .map(sd=>[sd,{refs:{...states[sd].refs},spans:{...states[sd].spans}}]))};
  }
  function importCalibration(data){
    if(!data||data.version!==1||!data.sides) return false;
    const finite=o=>o&&Object.values(o).every(v=>Number.isFinite(v));
    let ok=false;
    for(const sd of SIDES){
      const e=data.sides[sd];
      if(!e||!finite(e.refs)||!finite(e.spans)) continue;
      if(!Object.values(e.spans).every(v=>v>0)) continue;
      states[sd].refs={cx:e.refs.cx,cy:e.refs.cy,size:e.refs.size,roll:e.refs.roll};
      states[sd].spans={...defaultSpans(),...e.spans};
      states[sd].calibrated=true; ok=true;
    }
    return ok;
  }
  function saveCalibration(){
    try{localStorage.setItem(CFG.calib.storageKey,JSON.stringify(exportCalibration()));return true;}
    catch{return false;}
  }
  function loadCalibration(){
    try{const raw=localStorage.getItem(CFG.calib.storageKey);return raw?importCalibration(JSON.parse(raw)):false;}
    catch{return false;}
  }
  function resetCalibration(){
    for(const sd of SIDES){
      const s=states[sd];
      s.refs=null; s.spans=defaultSpans(); s.calibrated=false; s.origin=CFG.home.slice();
    }
    try{localStorage.removeItem(CFG.calib.storageKey);}catch{}
    calib.message="CALIBRATION RESET"; emitCal(true);
  }
  if(CFG.autoLoadCalibration) loadCalibration();

  // ------------------------------------------------------------------
  // Main update: hands + handedness come from hand-tracker.js
  // ------------------------------------------------------------------
  function updateFromHands(hands,handedness){
    const now=performance.now(), dt=clamp((now-lastTime)/1000,0.001,0.1); lastTime=now;
    let changed=false;
    for(let i=0;i<(hands||[]).length;i++){
      const meta=handedness?.[i]||{};
      let label=String(meta.label||"").toLowerCase();
      if(label!=="left"&&label!=="right") continue;
      if(CFG.swapHands) label=label==="left"?"right":"left";
      const side=label, s=states[side], f=features(hands[i]);
      if(!f) continue;

      // Confidence gate: a frame the tracker does not trust never moves the arm.
      const q=meta.quality??1;
      if(q<CFG.minQuality) continue;

      const gap=s.lastSeen?now-s.lastSeen:0;
      const reacquire=!!meta.reacquired||(s.lastSeen>0&&gap>CFG.reacquireGapMs);
      if(!s.refs) s.refs={...f};
      if(!s.smoothed||reacquire){
        s.smoothed={...f};                                  // do not glide from stale values
        if(s.lastSeen) s.ramp=CFG.reacquireRampMs/1000;     // ... let the target glide instead
      }
      s.lastSeen=now; s.quality=q; changed=true;

      // Lower quality -> weaker influence on the smoothed signal.
      const a=clamp(CFG.smoothing*(0.4+0.6*q),0.05,1);
      for(const key of ["cx","cy","size"]) s.smoothed[key]=(1-a)*s.smoothed[key]+a*f[key];
      // Angular smoothing on the shortest path, kept wrapped so it can never drift by 2*PI.
      s.smoothed.roll=wrap(s.smoothed.roll+a*wrap(f.roll-s.smoothed.roll));
      // Pinch only follows the fingers when the tracker trusts the thumb/index tips.
      const pinchOk=(meta.pinchQuality??1)>=CFG.minPinchQuality;
      if(pinchOk) s.smoothed.pinch=(1-a)*s.smoothed.pinch+a*f.pinch;

      feedCalibration(side,s.smoothed,q,now);

      const cx=s.smoothed.cx,cy=s.smoothed.cy,size=s.smoothed.size;
      // Small dead-band so camera/landmark jitter around the reference does not move the arm.
      const db=v=>Math.abs(v)<CFG.deadband?0:v;
      const sp=s.spans;
      const ux=axisNorm(db(cx-s.refs.cx),sp.xNeg,sp.xPos)*CFG.lateralSign;
      const uz=axisNorm(db(-(cy-s.refs.cy)),sp.yDown,sp.yUp);
      const ud=axisNorm(size/s.refs.size-1,sp.dFar,sp.dNear)*CFG.depthSign;
      // Robot frame: +X forward, +Y left, +Z up. The raw (un-mirrored) camera image has the
      // operator's right hand at smaller cx, so +ux (larger cx) maps to +Y (left) on the robot.
      // The reachable volume is ellipsoidal, not a box: limit the combined hand offset to the
      // unit ball so diagonal hand moves (e.g. far + up + sideways) stay reachable.
      let off=[ud,ux,uz]; const len=Math.hypot(...off); if(len>1) off=off.map(v=>v/len);
      off=off.map(v=>v*CFG.motionScale);
      s.desired=[s.origin[0]+off[0]*CFG.range[0],s.origin[1]+off[1]*CFG.range[1],s.origin[2]+off[2]*CFG.range[2]];
      const kin=kins[side];
      s.roll=clamp(CFG.rollSign*CFG.rollGain*wrap(s.smoothed.roll-s.refs.roll),kin.lower[4],kin.upper[4]);
      if(pinchOk){
        const t=clamp((s.smoothed.pinch-CFG.pinchClosed)/(CFG.pinchOpen-CFG.pinchClosed),0,1);
        s.grip=CFG.gripperClosed+t*(CFG.gripperOpen-CFG.gripperClosed);
        // Hysteresis so the gripper does not chatter around the threshold.
        if(!s.closed && t<0.35) s.closed=true; else if(s.closed && t>0.65) s.closed=false;
      } // else: fingers occluded / predicted -> gripper holds its last state
    }

    tickCalibration(now);

    for(const side of SIDES){
      const s=states[side], kin=kins[side];
      s.frozen=calib.active&&calib.phase==="neutral"&&calib.sides.includes(side);
      // Rate-limit the IK target: slow glide right after a (re-)acquire, normal limit otherwise.
      if(s.ramp>0) s.ramp=Math.max(0,s.ramp-dt);
      const rampFrac=CFG.reacquireRampMs>0?s.ramp/(CFG.reacquireRampMs/1000):0;
      const speed=CFG.maxTargetSpeed+(CFG.reacquireTargetSpeed-CFG.maxTargetSpeed)*rampFrac;
      s.target=stepToward(s.target,s.desired,speed*dt);
      if(!s.frozen && s.lastSeen && now-s.lastSeen<=CFG.lostHoldMs){
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
    getJointRadians:()=>Object.fromEntries(SIDES.map(side=>[side,states[side].q.slice()])),
    getHomeRadians:()=>qHomes.right.slice(),
    getHomeRadiansBySide:()=>({left:qHomes.left.slice(),right:qHomes.right.slice()}),
    // --- tracking status (for UI) ---
    getStatus:()=>Object.fromEntries(SIDES.map(side=>{
      const s=states[side], now=performance.now();
      return [side,{tracking:s.lastSeen>0&&now-s.lastSeen<=CFG.lostHoldMs,quality:s.quality,
                    calibrated:s.calibrated,ramping:s.ramp>0}];
    })),
    setMotionScale:v=>{CFG.motionScale=clamp(v,0,1);},
    // --- calibration ---
    startCalibration,cancelCalibration,sync,
    getCalibration:exportCalibration,setCalibration:importCalibration,
    saveCalibration,loadCalibration,resetCalibration,
    isCalibrating:()=>calib.active
  };
}

// Exposed for offline tuning/tests.
export { ArmKinematics, ARM_CHAINS, CFG };
