// web/app.js — Lyra 控制台前端
//  Three.js 实时 3D（DH 正解与后端一致）+ WebSocket（相对 /ws，同源）+ 拖拽逆解跟随
//  + 关节滑杆 + 示教回放 + 抓取编排 + 急停。
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---------- DH 参数（与 lib/arm/kinematics.hpp 的 ArmModel::urStyle 保持一致）----------
const D2R = Math.PI / 180;
const LINKS = [
  { a: 0.000, alpha: 90, d: 0.1625, off: 0 },
  { a: -0.425, alpha: 0, d: 0.0000, off: -90 * D2R },
  { a: -0.392, alpha: 0, d: 0.0000, off: 0 },
  { a: 0.000, alpha: 90, d: 0.1333, off: 0 },
  { a: 0.000, alpha: -90, d: 0.0997, off: 0 },
  { a: 0.000, alpha: 0, d: 0.0996, off: 0 },
];
const TOOL = [0, 0, 0.06]; // T6_tool 平移
const QLIM = 2.967;

// ---------- 行优先 4x4 ----------
function matIdentity() { return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]; }
function matMul(A, B) {
  const C = new Array(16).fill(0);
  for (let r = 0; r < 4; r++)
    for (let c = 0; c < 4; c++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[r * 4 + k] * B[k * 4 + c];
      C[r * 4 + c] = s;
    }
  return C;
}
function dhTransform(l, q) {
  const th = q + l.off, ca = Math.cos(l.alpha * D2R), sa = Math.sin(l.alpha * D2R);
  const c = Math.cos(th), s = Math.sin(th);
  return [
    c, -s * ca, s * sa, l.a * c,
    s, c * ca, -c * sa, l.a * s,
    0, sa, ca, l.d,
    0, 0, 0, 1,
  ];
}
function fk(q) {
  const frames = [matIdentity()];
  for (let i = 0; i < 6; i++) frames.push(matMul(frames[i], dhTransform(LINKS[i], q[i])));
  return frames; // frames[0..6]
}
function toolPose(T6) {
  return matMul(T6, [1, 0, 0, TOOL[0], 0, 1, 0, TOOL[1], 0, 0, 1, TOOL[2], 0, 0, 0, 1]);
}

// ---------- WebSocket（同源相对路径，预览代理安全）----------
let ws = null, wsAlive = false;
let lastState = null;
let reconnectTimer = null;

function wsConnect() {
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
  ws = new WebSocket(url);
  ws.onopen = () => {
    wsAlive = true;
    setBadge(true);
  };
  ws.onclose = () => {
    wsAlive = false;
    setBadge(false);
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(wsConnect, 1500);
  };
  ws.onerror = () => ws && ws.close();
  ws.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch { return; }
    if (m.type === 'state') { lastState = m; onState(m); }
    else if (m.type === 'demo') {
      document.getElementById('demo-out').value = JSON.stringify(m, null, 1);
    }
  };
}
function send(obj) {
  if (wsAlive && ws.readyState === 1) ws.send(JSON.stringify(obj));
}
function setBadge(on) {
  const b = document.getElementById('conn');
  b.textContent = on ? '在线' : '离线';
  b.className = 'badge ' + (on ? 'on' : 'off');
}

// ---------- 3D 场景 ----------
const viewport = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
viewport.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);
const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 50);
camera.position.set(0.85, -0.85, 0.65);
camera.up.set(0, 0, 1);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.target.set(0, 0, 0.3);
orbit.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x202020, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.2);
sun.position.set(1.5, -2, 3);
scene.add(sun);
// 地面网格（XY 平面，Z 向上 —— 与机械臂基座 z 轴一致）
const grid = new THREE.GridHelper(1.6, 32, 0x2a3342, 0x1d2430);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// 基座
const base = new THREE.Mesh(
  new THREE.CylinderGeometry(0.05, 0.07, 0.04, 32),
  new THREE.MeshStandardMaterial({ color: 0x39445a, metalness: 0.4, roughness: 0.6 })
);
base.rotation.x = Math.PI / 2;
base.position.set(0, 0, 0.02);
scene.add(base);

// 连杆与关节可视化：每帧由 FK 驱动
const linkMat = new THREE.MeshStandardMaterial({ color: 0x5ab0ff, metalness: 0.3, roughness: 0.5 });
const jointMat = new THREE.MeshStandardMaterial({ color: 0xffb454, metalness: 0.5, roughness: 0.4 });
const linkMeshes = [];
const jointMeshes = [];
for (let i = 0; i < 6; i++) {
  const link = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.035, 1), linkMat);
  link.scale.z = 1;
  scene.add(link);
  linkMeshes.push(link);
  const jn = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.06, 20), jointMat);
  jn.rotation.x = Math.PI / 2;
  scene.add(jn);
  jointMeshes.push(jn);
}
// 末端 TCP 坐标轴
const tcpAxes = new THREE.AxesHelper(0.12);
scene.add(tcpAxes);
// 拖拽手柄
const handle = new THREE.Mesh(
  new THREE.SphereGeometry(0.025, 20, 20),
  new THREE.MeshStandardMaterial({ color: 0x3ddc84, emissive: 0x0a3a22 })
);
scene.add(handle);
// 目标位姿虚影
const ghost = new THREE.Mesh(
  new THREE.SphereGeometry(0.02, 16, 16),
  new THREE.MeshBasicMaterial({ color: 0xff5252, transparent: true, opacity: 0.55 })
);
ghost.visible = false;
scene.add(ghost);

// 把行优先 4x4 写进 THREE 物体（列优先自动换算）
function applyPose(obj, M) {
  const e = obj.matrixWorld.elements; // THREE 列优先
  for (let r = 0; r < 4; r++)
    for (let c = 0; c < 4; c++) e[c * 4 + r] = M[r * 4 + c];
  obj.matrixAutoUpdate = false;
  obj.matrixWorldNeedsUpdate = true;
}
// 细长连杆：从 A 到 B
function stretchBetween(mesh, A, B) {
  const a = new THREE.Vector3(A[3], A[7], A[11]);
  const b = new THREE.Vector3(B[3], B[7], B[11]);
  const mid = a.clone().add(b).multiplyScalar(0.5);
  const dir = b.clone().sub(a);
  const len = Math.max(dir.length(), 1e-4);
  const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), dir.clone().normalize());
  mesh.position.copy(mid);
  mesh.quaternion.copy(q);
  mesh.scale.set(1, 1, len);
  mesh.matrixAutoUpdate = true;
}

function renderRobot(q) {
  const frames = fk(q);
  for (let i = 0; i < 6; i++) {
    stretchBetween(linkMeshes[i], frames[i], frames[i + 1]);
    const O = [frames[i][3], frames[i][7], frames[i][11]];
    jointMeshes[i].position.set(O[0], O[1], O[2]);
  }
  const T6 = frames[6];
  const Ttcp = toolPose(T6);
  // 关节 6 壳体到 TCP
  stretchBetween(linkMeshes[5], T6, Ttcp);
  applyPose(tcpAxes, Ttcp);
  tcpAxes.matrixAutoUpdate = true;
  handle.position.set(Ttcp[3], Ttcp[7], Ttcp[11]);
}

// ---------- UI 构建 ----------
const jointEls = [];
{
  const box = document.getElementById('joints');
  for (let i = 0; i < 6; i++) {
    const row = document.createElement('div');
    row.className = 'jrow';
    row.innerHTML = `<span>J${i + 1}</span><input type="range" min="${-QLIM}" max="${QLIM}" step="0.01" value="0"/>
                     <output>0.0°</output>`;
    box.appendChild(row);
    const inp = row.querySelector('input'), out = row.querySelector('output');
    jointEls.push({ inp, out });
  }
}
const $ = (id) => document.getElementById(id);
const speedEl = $('speed'), followEl = $('follow'), dragEl = $('drag-mode');
let qCmd = [0, -0.5, 0.5, 0, 0.5, 0];
let lastJointSend = 0;

function speed() { return parseFloat(speedEl.value); }
$('speed').addEventListener('input', () => ($('speed-v').textContent = speed().toFixed(2)));

jointEls.forEach(({ inp, out }, i) => {
  inp.addEventListener('input', () => {
    qCmd[i] = parseFloat(inp.value);
    out.textContent = ((qCmd[i] / D2R).toFixed(1)) + '°';
    if (!followEl.checked) return;
    const now = performance.now();
    if (now - lastJointSend > 60) {
      lastJointSend = now;
      send({ type: 'joint_target', q: qCmd, speed: speed() });
    }
  });
  inp.addEventListener('change', () => send({ type: 'joint_target', q: qCmd, speed: speed() }));
});

$('go-ee').onclick = () => send({
  type: 'ee_target',
  pos: [+$('px').value, +$('py').value, +$('pz').value],
  rpy: [+$('pr').value, +$('pp').value, +$('pw').value],
  speed: speed(),
});
$('grip').addEventListener('input', () => send({ type: 'grip', g: +$('grip').value }));
$('teach-add').onclick = () => send({ type: 'teach_add' });
$('teach-clear').onclick = () => { send({ type: 'teach_clear' }); $('demo-out').value = ''; };
$('teach-play').onclick = () => send({ type: 'teach_play', speed: speed() });
$('teach-export').onclick = () => send({ type: 'teach_export' });
$('grasp-here').onclick = () => {
  if (!lastState) return;
  const [x, y] = lastState.ee_pos;
  send({ type: 'grasp', pos: [x, y, 0.0], height: 0.05, approach: 0.1, grip_z: 0.03 });
};
$('estop').onclick = () => send({ type: 'estop', on: true });
$('estop-clear').onclick = () => send({ type: 'estop', on: false });
$('reset').onclick = () => send({ type: 'reset' });

// ---------- 状态显示 ----------
function onState(m) {
  $('st-mode').textContent = m.mode;
  $('st-t').textContent = (+m.t).toFixed(2);
  $('st-ee').textContent = m.ee_pos.map((v) => v.toFixed(3)).join(', ');
  $('st-rpy').textContent = m.ee_rpy.map((v) => v.toFixed(2)).join(', ');
  $('teach-n').textContent = m.teach_count;
  // 奇异度条（无量纲 η = σ_min/σ_max，本臂正常域约 0.02~0.25）
  const s0 = m.sig_idx ?? 0;
  const bar = $('sig-bar');
  bar.style.width = (Math.min(1, s0 / 0.2) * 100).toFixed(0) + '%';
  bar.style.background = s0 < 0.02 ? '#ff5252' : s0 < 0.06 ? '#ffb454' : '#3ddc84';
  $('sig-val').textContent = s0.toFixed(4);
  // 滑杆同步（只在非拖动时）
  for (let i = 0; i < 6; i++) {
    const { inp, out } = jointEls[i];
    if (document.activeElement !== inp) {
      inp.value = m.q[i];
      out.textContent = ((m.q[i] / D2R).toFixed(1)) + '°';
      qCmd[i] = m.q[i];
    }
  }
  // 位姿输入框同步
  if (document.activeElement?.type !== 'number') {
    $('px').value = m.ee_pos[0].toFixed(3);
    $('py').value = m.ee_pos[1].toFixed(3);
    $('pz').value = m.ee_pos[2].toFixed(3);
    $('pr').value = m.ee_rpy[0].toFixed(3);
    $('pp').value = m.ee_rpy[1].toFixed(3);
    $('pw').value = m.ee_rpy[2].toFixed(3);
  }
  renderRobot(m.q);
}

// ---------- 3D 拖拽末端（平移，姿态跟随）----------
const ray = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let dragging = false;
const dragPlane = new THREE.Plane();
const hitPt = new THREE.Vector3();

renderer.domElement.addEventListener('pointerdown', (ev) => {
  if (!dragEl.checked || !lastState) return;
  mouse.x = (ev.offsetX / renderer.domElement.clientWidth) * 2 - 1;
  mouse.y = -(ev.offsetY / renderer.domElement.clientHeight) * 2 + 1;
  ray.setFromCamera(mouse, camera);
  const p = handle.position.clone();
  const camDir = camera.getWorldDirection(new THREE.Vector3());
  dragPlane.setFromNormalAndCoplanarPoint(camDir, p);
  if (ray.ray.intersectPlane(dragPlane, hitPt)) {
    dragging = true;
    orbit.enabled = false;
    ghost.visible = true;
  }
});
renderer.domElement.addEventListener('pointermove', (ev) => {
  if (!dragging) return;
  mouse.x = (ev.offsetX / renderer.domElement.clientWidth) * 2 - 1;
  mouse.y = -(ev.offsetY / renderer.domElement.clientHeight) * 2 + 1;
  ray.setFromCamera(mouse, camera);
  if (ray.ray.intersectPlane(dragPlane, hitPt)) {
    ghost.position.copy(hitPt);
    // 姿态跟随当前
    const rpy = lastState.ee_rpy;
    send({ type: 'ee_drag', pos: [hitPt.x, hitPt.y, hitPt.z], rpy });
  }
});
window.addEventListener('pointerup', () => {
  if (dragging) {
    dragging = false;
    orbit.enabled = true;
    ghost.visible = false;
    send({ type: 'joint_target', q: qCmd, speed: speed() }); // 落到最近构型
  }
});

// ---------- 主渲染循环 ----------
function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();
(function loop() {
  requestAnimationFrame(loop);
  orbit.update();
  renderer.render(scene, camera);
})();

wsConnect();
// 状态请求：后端 50Hz 推送，无需轮询
