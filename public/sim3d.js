/*
 * SMARS 3D simulyatoru
 *  - Robot modeli SMARS CAD fayllarından (chassis_SL, master/slave wheel, mechanical_track,
 *    TCRT5000 kaseti, kürək, orijinal ultrasəs korpusu) — /models/smars.bin + smars.json
 *  - Virtual "firmware": real robotla eyni yollar (/status /mode /tank /servo /settings /set /reset)
 *  - Fizika: tank sürüşü, toqquşma, itələnən kublar; sensorlar: HC-SR04 (konus), 3 × TCRT5000 (yerin rəngi)
 *
 * Koordinatlar: dünya = three.js (Y yuxarı, mm). Robotun "şassi çərçivəsi" = CAD: +X sağ, +Y irəli, +Z yuxarı.
 */
import * as THREE from './vendor/three/three.module.js';
import { OrbitControls } from './vendor/three/OrbitControls.js';

const WORLD = 3000;              // mm, kvadrat meydança
const FLOOR_PX = 2048;
const TAN15 = Math.tan((15 * Math.PI) / 180);

export const SETTINGS_DEF = [
  { k: 'tankSpd', l: 'Standart sürət', t: 'i', d: 180, mn: 60, mx: 255, st: 1, g: 'drive' },
  { k: 'turnK', l: 'Dönmə əmsalı', t: 'f', d: 0.7, mn: 0.2, mx: 1, st: 0.05, g: 'drive' },
  { k: 'obsDist', l: 'Maneə məsafəsi (sm)', t: 'i', d: 22, mn: 5, mx: 80, st: 1, g: 'obs' },
  { k: 'obsBack', l: 'Geri çəkilmə (ms)', t: 'i', d: 450, mn: 100, mx: 1500, st: 50, g: 'obs' },
  { k: 'lineSpd', l: 'Xətt sürəti', t: 'i', d: 130, mn: 60, mx: 255, st: 1, g: 'line' },
  { k: 'irThr', l: 'IR qara həddi (%)', t: 'i', d: 50, mn: 5, mx: 95, st: 1, g: 'line' },
  { k: 'lineInv', l: 'Ağ xətt (invert)', t: 'b', d: 0, mn: 0, mx: 1, st: 1, g: 'line' },
  { k: 'arenaSpd', l: 'Arena hücum sürəti', t: 'i', d: 220, mn: 60, mx: 255, st: 1, g: 'arena' },
  { k: 'arenaSee', l: 'Hədəf məsafəsi (sm)', t: 'i', d: 100, mn: 10, mx: 150, st: 1, g: 'arena' },
  { k: 'grpLo', l: 'Gripper min', t: 'i', d: 20, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'grpHi', l: 'Gripper max', t: 'i', d: 160, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'elbLo', l: 'Elbow min', t: 'i', d: 10, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'elbHi', l: 'Elbow max', t: 'i', d: 170, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'maxSpd', l: 'Motor maks. sürəti (mm/s)', t: 'i', d: 300, mn: 100, mx: 700, st: 10, g: 'sim' },
  { k: 'noise', l: 'Sensor küyü', t: 'b', d: 1, mn: 0, mx: 1, st: 1, g: 'sim' },
  { k: 'autoHome', l: 'Batareya azalanda özü stansiyaya qayıtsın', t: 'b', d: 1, mn: 0, mx: 1, st: 1, g: 'bat' },
  { k: 'lowPct', l: 'Avtomatik qayıdış həddi (%)', t: 'i', d: 25, mn: 5, mx: 60, st: 1, g: 'bat' },
  { k: 'dockRev', l: 'Stansiyaya girmə sürəti', t: 'i', d: 110, mn: 60, mx: 200, st: 1, g: 'bat' },
  { k: 'drainX', l: 'Boşalmanı sürətləndir × (stansiya səhnəsində)', t: 'i', d: 10, mn: 1, mx: 200, st: 1, g: 'bat' },
  { k: 'chgX', l: 'Şarjı sürətləndir × (stansiya səhnəsində)', t: 'i', d: 40, mn: 1, mx: 200, st: 1, g: 'bat' },
];
const GROUPS = [
  { i: 'drive', t: 'Sürüş' }, { i: 'obs', t: 'Maneədən qaçma' }, { i: 'line', t: 'Line following' },
  { i: 'arena', t: 'Arena' }, { i: 'servo', t: 'Servo limitləri' }, { i: 'bat', t: 'Batareya / şarj' }, { i: 'sim', t: 'Simulyasiya' },
];
export const SCENES = { line: 'Xətt trası', obstacle: 'Maneələr', arena: 'Arena (kub çıxar)', station: 'Şarj stansiyası' };

// 2S Li-ion: gərginlik ↔ faiz (firmware/smars_dock.h ilə eyni əyri)
const BAT_CURVE = [[6.4, 0], [6.8, 8], [7.2, 25], [7.4, 45], [7.6, 60], [7.8, 72], [8.0, 84], [8.2, 93], [8.4, 100]];
function voltsFromPct(p) {
  for (let i = 1; i < BAT_CURVE.length; i++) {
    const [v0, p0] = BAT_CURVE[i - 1], [v1, p1] = BAT_CURVE[i];
    if (p <= p1) return v0 + ((p - p0) / (p1 - p0)) * (v1 - v0);
  }
  return 8.4;
}
const DOCK_NAMES = {
  idle: 'IDLE', seek: 'XƏTT AXTARIR', follow: 'XƏTT İZLƏYİR', turn: 'XƏTTİN SONU — DÖNÜR', stop: 'T — DAYANIR',
  spin: '180° FIRLANIR', center: 'TUŞLANIR', reverse: 'STANSİYAYA GİRİR', retry: 'YENİDƏN DÜZLƏNİR', backoff: 'GERİ ÇƏKİLİR',
  docked: 'ŞARJ OLUNUR', charged: 'DOLDU', undock: 'STANSİYADAN ÇIXIR', failed: 'DOK ALINMADI',
};

// ---------------------------------------------------------------- assets
let assetP = null;
function loadAssets() {
  if (!assetP) {
    assetP = Promise.all([
      fetch('/models/smars.json').then((r) => r.json()),
      fetch('/models/smars.bin').then((r) => r.arrayBuffer()),
    ]).then(([layout, buf]) => {
      const geo = {};
      for (const [name, p] of Object.entries(layout.parts)) {
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(buf, p.v[0], p.v[1] * 3), 3));
        g.setIndex(new THREE.BufferAttribute(new Uint32Array(buf, p.i[0], p.i[1]), 1));
        const flat = g.toNonIndexed();          // CAD görünüşü üçün düz kölgələmə
        flat.computeVertexNormals();
        geo[name] = flat;
      }
      return { layout, geo };
    });
  }
  return assetP;
}

const mat = (color, o = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.62, metalness: 0.04, ...o });
const rnd = (a, b) => a + Math.random() * (b - a);

// ---------------------------------------------------------------- 2D geometry helpers (x, z dünya müstəvisi)
function rectPoly(cx, cz, h, x0, x1, y0, y1) {
  // robot yerli düzbucaqlısı (x sağ, y irəli) → dünya poliqonu
  const c = Math.cos(h), s = Math.sin(h);
  return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([lx, ly]) => [cx + lx * c - ly * s, cz - lx * s - ly * c]);
}
function satOverlap(A, B) {
  for (const P of [A, B]) {
    for (let i = 0; i < P.length; i++) {
      const [x1, z1] = P[i], [x2, z2] = P[(i + 1) % P.length];
      const nx = z2 - z1, nz = x1 - x2;
      let aMin = Infinity, aMax = -Infinity, bMin = Infinity, bMax = -Infinity;
      for (const [x, z] of A) { const d = x * nx + z * nz; aMin = Math.min(aMin, d); aMax = Math.max(aMax, d); }
      for (const [x, z] of B) { const d = x * nx + z * nz; bMin = Math.min(bMin, d); bMax = Math.max(bMax, d); }
      if (aMax <= bMin || bMax <= aMin) return false;
    }
  }
  return true;
}
function boxPoly(o) {
  const hw = o.w / 2, hd = o.d / 2;
  return [[o.x - hw, o.z - hd], [o.x + hw, o.z - hd], [o.x + hw, o.z + hd], [o.x - hw, o.z + hd]];
}
function polyCircle(P, cx, cz, r) {
  let pos = 0, neg = 0;                      // istiqamətdən asılı olmayan "içəridə" yoxlaması
  let best = Infinity;
  for (let i = 0; i < P.length; i++) {
    const [x1, z1] = P[i], [x2, z2] = P[(i + 1) % P.length];
    const ex = x2 - x1, ez = z2 - z1;
    const t = Math.max(0, Math.min(1, ((cx - x1) * ex + (cz - z1) * ez) / (ex * ex + ez * ez)));
    const px = x1 + t * ex - cx, pz = z1 + t * ez - cz;
    best = Math.min(best, px * px + pz * pz);
    if ((ex * (cz - z1) - ez * (cx - x1)) > 0) pos++; else neg++;
  }
  return pos === 0 || neg === 0 || best < r * r;
}
function rayBox(ox, oz, dx, dz, o) {
  const x0 = o.x - o.w / 2, x1 = o.x + o.w / 2, z0 = o.z - o.d / 2, z1 = o.z + o.d / 2;
  let tmin = 0, tmax = Infinity;
  for (const [p, d, a, b] of [[ox, dx, x0, x1], [oz, dz, z0, z1]]) {
    if (Math.abs(d) < 1e-9) { if (p < a || p > b) return Infinity; continue; }
    let t1 = (a - p) / d, t2 = (b - p) / d;
    if (t1 > t2) [t1, t2] = [t2, t1];
    tmin = Math.max(tmin, t1); tmax = Math.min(tmax, t2);
    if (tmin > tmax) return Infinity;
  }
  return tmin;
}
function rayPoly(ox, oz, dx, dz, P) {
  let best = Infinity;
  for (let i = 0; i < P.length; i++) {
    const [x1, z1] = P[i], [x2, z2] = P[(i + 1) % P.length];
    const ex = x2 - x1, ez = z2 - z1;
    const den = dx * ez - dz * ex;
    if (Math.abs(den) < 1e-12) continue;
    const t = ((x1 - ox) * ez - (z1 - oz) * ex) / den;
    const u = ((x1 - ox) * dz - (z1 - oz) * dx) / den;
    if (t >= 0 && u >= 0 && u <= 1) best = Math.min(best, t);
  }
  return best;
}
function rayCircle(ox, oz, dx, dz, cx, cz, r) {
  const fx = ox - cx, fz = oz - cz;
  const b = fx * dx + fz * dz, c = fx * fx + fz * fz - r * r;
  const disc = b * b - c;
  if (disc < 0) return Infinity;
  const t = -b - Math.sqrt(disc);
  return t >= 0 ? t : (c < 0 ? 0 : Infinity);
}

// ---------------------------------------------------------------- simulator
export async function createSim(container, opts = {}) {
  const { layout, geo } = await loadAssets();
  const L = layout;
  const GZ = -L.ground_z;                          // şassi mərkəzinin yerdən hündürlüyü (28.75)
  const TRACK_W = L.track.x * 2;                   // tırtıllar arası məsafə
  const R = L.track.link_radius, AY = L.wheels.axle_y, AZ = L.wheels.axle_z;
  const TRACK_LEN = 4 * AY + 2 * Math.PI * R;
  const NLINK = L.track.links_per_side;

  // ---------- renderer / scene  (headless: test üçün — WebGL və kamera yoxdur, zamanı step() irəli çəkir)
  const headless = !!opts.headless;
  let renderer = null;
  if (!headless) {
    renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: !!opts.preserveDrawingBuffer });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    container.appendChild(renderer.domElement);
  }
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#0a1118');
  scene.fog = new THREE.Fog('#0a1118', 2600, 6000);
  const camera = new THREE.PerspectiveCamera(50, 1, 5, 20000);
  camera.position.set(-380, 330, 420);
  const controls = headless
    ? { target: new THREE.Vector3(), update() {}, dispose() {}, enabled: true }
    : new OrbitControls(camera, renderer.domElement);
  if (!headless) {
    controls.enableDamping = true;
    controls.maxPolarAngle = Math.PI * 0.49;
    controls.minDistance = 120;
    controls.maxDistance = 4500;
  }

  scene.add(new THREE.HemisphereLight('#dfefff', '#20262c', 1.1));
  const sun = new THREE.DirectionalLight('#ffffff', 1.6);
  sun.position.set(600, 1400, 500);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  Object.assign(sun.shadow.camera, { left: -500, right: 500, top: 500, bottom: -500, near: 100, far: 4000 });
  scene.add(sun, sun.target);

  // ---------- floor (rəng xəritəsi həm görüntü, həm də IR sensor üçün)
  const floorCanvas = opts.createCanvas ? opts.createCanvas(FLOOR_PX, FLOOR_PX) : document.createElement('canvas');
  floorCanvas.width = floorCanvas.height = FLOOR_PX;
  const fctx = floorCanvas.getContext('2d', { willReadFrequently: true });
  const floorTex = new THREE.CanvasTexture(floorCanvas);
  floorTex.colorSpace = THREE.SRGBColorSpace;
  floorTex.anisotropy = 8;
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(WORLD, WORLD), new THREE.MeshStandardMaterial({ map: floorTex, roughness: 0.9 }));
  floor.rotation.x = -Math.PI / 2;
  floor.receiveShadow = true;
  scene.add(floor);
  let floorData = null;
  const toPx = (v) => ((v + WORLD / 2) / WORLD) * FLOOR_PX;
  function darkness(x, z) {
    const px = Math.floor(toPx(x)), py = Math.floor(toPx(z));
    if (!floorData || px < 0 || py < 0 || px >= FLOOR_PX || py >= FLOOR_PX) return 0.85;
    let sum = 0, n = 0;                            // ~5 mm-lik ləkə (TCRT5000 görmə sahəsi)
    for (let dy = -2; dy <= 2; dy++) for (let dx = -2; dx <= 2; dx++) {
      const qx = Math.min(FLOOR_PX - 1, Math.max(0, px + dx)), qy = Math.min(FLOOR_PX - 1, Math.max(0, py + dy));
      const i = (qy * FLOOR_PX + qx) * 4;
      sum += 0.2126 * floorData[i] + 0.7152 * floorData[i + 1] + 0.0722 * floorData[i + 2]; n++;
    }
    return 1 - sum / n / 255;
  }

  // ---------- robot model
  const robot = new THREE.Group();
  scene.add(robot);
  const frame = new THREE.Group();                 // şassi çərçivəsi (CAD koordinatları)
  frame.rotation.x = -Math.PI / 2;
  frame.position.y = GZ;
  robot.add(frame);
  const partMesh = (name, colorOverride) => {
    const m = new THREE.Mesh(geo[name], mat(colorOverride || L.parts[name].color));
    m.castShadow = true; m.receiveShadow = true;
    return m;
  };
  frame.add(partMesh('chassis'));
  const wheels = [];
  for (const [kind, list] of [['wheel_master', L.wheels.master], ['wheel_slave', L.wheels.slave]]) {
    for (const [x, y] of list) {
      const w = partMesh(kind);
      w.position.set(x, y, AZ);
      frame.add(w);
      wheels.push({ m: w, side: x < 0 ? 0 : 1 });
    }
  }
  const links = new THREE.InstancedMesh(geo.link, mat(L.parts.link.color), NLINK * 2);
  links.castShadow = true;
  frame.add(links);
  // Arduino + motor shield (sxematik)
  const board = new THREE.Group();
  const ard = new THREE.Mesh(new THREE.BoxGeometry(53.3, 68.6, 1.6), mat('#0b5aa6')); ard.position.set(0, 0, 19);
  const shield = new THREE.Mesh(new THREE.BoxGeometry(53.3, 68.6, 1.6), mat('#1f7ad1')); shield.position.set(0, 0, 31);
  const chips = new THREE.Mesh(new THREE.BoxGeometry(20, 30, 4), mat('#15181b')); chips.position.set(-6, 4, 34);
  const term = new THREE.Mesh(new THREE.BoxGeometry(40, 8, 9), mat('#2f8fe0')); term.position.set(0, -26, 36.5);
  board.add(ard, shield, chips, term);
  board.traverse((o) => { o.castShadow = true; });
  frame.add(board);
  // servo qolu (sxematik — CAD-də yoxdur)
  const arm = new THREE.Group(); arm.position.set(0, -40, 24); frame.add(arm);
  const armBase = new THREE.Mesh(new THREE.BoxGeometry(24, 12, 14), mat('#2a2f35')); arm.add(armBase);
  const elbow = new THREE.Group(); elbow.position.set(0, 0, 7); arm.add(elbow);
  const upper = new THREE.Mesh(new THREE.BoxGeometry(8, 6, 46), mat('#ff9f00')); upper.position.set(0, 0, 23); elbow.add(upper);
  const hand = new THREE.Group(); hand.position.set(0, 0, 46); elbow.add(hand);
  const palm = new THREE.Mesh(new THREE.BoxGeometry(22, 6, 6), mat('#b44fff')); hand.add(palm);
  const fingers = [-1, 1].map((sx) => {
    const f = new THREE.Mesh(new THREE.BoxGeometry(3, 5, 16), mat('#b44fff')); f.position.set(sx * 8, 0, 10); hand.add(f); return f;
  });
  arm.traverse((o) => { o.castShadow = true; });
  // konfiqurasiya hissələri
  const cfgGroups = {};
  for (const [k, c] of Object.entries(L.configs)) {
    const g = new THREE.Group();
    c.parts.forEach((p) => g.add(partMesh(p)));
    g.visible = false;
    frame.add(g);
    cfgGroups[k] = g;
  }
  // arxa şarj quyruğu (bütün konfiqurasiyalarda arxa extension yuvasında)
  const DK = L.dock || null;
  if (DK) DK.robot_parts.forEach((p) => geo[p] && frame.add(partMesh(p)));
  // şarj stansiyası (yalnız "station" səhnəsində görünür) — dok çərçivəsində qurulur
  const stationGroup = new THREE.Group(); stationGroup.visible = false; scene.add(stationGroup);
  const stationFrame = new THREE.Group(); stationFrame.rotation.x = -Math.PI / 2; stationFrame.position.y = GZ;
  stationGroup.add(stationFrame);
  const ledMats = {};
  if (DK) {
    DK.station_parts.forEach((p) => {
      if (!geo[p]) return;
      let m;
      if (p === 'S_led_red' || p === 'S_led_green') {
        const col = L.parts[p].color;
        ledMats[p] = mat(col, { emissive: new THREE.Color(col), emissiveIntensity: p === 'S_led_red' ? 0.9 : 0.0 });
        m = new THREE.Mesh(geo[p], ledMats[p]);
      } else m = partMesh(p);
      stationFrame.add(m);
    });
  }
  // sensor köməkçiləri (dünya səhnəsində)
  const helpers = new THREE.Group(); scene.add(helpers);
  const coneMat = new THREE.MeshBasicMaterial({ color: '#00e5ff', transparent: true, opacity: 0.08, depthWrite: false, side: THREE.DoubleSide });
  const cone = new THREE.Mesh(new THREE.ConeGeometry(1, 1, 32, 1, true), coneMat);
  helpers.add(cone);
  const hitDot = new THREE.Mesh(new THREE.SphereGeometry(6, 16, 12), new THREE.MeshBasicMaterial({ color: '#ff2244' }));
  helpers.add(hitDot);
  const irDots = [0, 1, 2].map(() => {
    const d = new THREE.Mesh(new THREE.CircleGeometry(6, 20), new THREE.MeshBasicMaterial({ color: '#39ff14', transparent: true, opacity: 0.85 }));
    d.rotation.x = -Math.PI / 2; helpers.add(d); return d;
  });

  // ---------- world objects
  const world = new THREE.Group(); scene.add(world);
  let statics = [];          // {kind:'box'|'cyl', x,z,w,d,r,h, poly}
  let cubes = [];            // {x,z,a,s,h,out,mesh}
  let arenaR = 0, arenaBorder = 0;
  const cubeGeo = new THREE.BoxGeometry(50, 50, 50);

  const bbOf = (P) => [Math.min(...P.map((p) => p[0])), Math.min(...P.map((p) => p[1])), Math.max(...P.map((p) => p[0])), Math.max(...P.map((p) => p[1]))];
  function addBox(x, z, w, d, h, color) {
    const o = { kind: 'box', x, z, w, d, h };
    o.poly = boxPoly(o); o.bb = bbOf(o.poly);
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat(color));
    m.position.set(x, h / 2, z); m.castShadow = m.receiveShadow = true;
    world.add(m); statics.push(o);
  }
  function addCyl(x, z, r, h, color) {
    const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 32), mat(color));
    m.position.set(x, h / 2, z); m.castShadow = m.receiveShadow = true;
    world.add(m); statics.push({ kind: 'cyl', x, z, r, h, bb: [x - r, z - r, x + r, z + r] });
  }
  function addPoly(poly, h, guide) {           // qabarıq poliqon maneə (stansiya hissələri) — görüntüsü ayrıca
    statics.push({ kind: 'poly', poly, h, guide: !!guide, bb: bbOf(poly) });
  }
  // dok pozası: robot burada olanda stansiyaya tam oturub (dok çərçivəsi → dünya)
  let dockPose = null;
  const dockToWorld = (lx, ly) => {
    const c = Math.cos(dockPose.h), s = Math.sin(dockPose.h);
    return [dockPose.x + lx * c - ly * s, dockPose.z - lx * s - ly * c];
  };
  function addCube(x, z) {
    const m = new THREE.Mesh(cubeGeo, mat('#ff5a1f'));
    m.castShadow = m.receiveShadow = true; world.add(m);
    cubes.push({ x, z, a: rnd(0, Math.PI), s: 50, h: 50, out: false, mesh: m });
  }

  let startPose = { x: 0, z: 0, h: 0 };
  let linePath = null;

  let sceneName = 'line';
  function buildScene(name) {
    sceneName = name;
    world.clear(); statics = []; cubes = []; arenaR = 0; linePath = null; dockPose = null;
    stationGroup.visible = false;
    const g = fctx;
    const S = FLOOR_PX / WORLD;
    g.setTransform(1, 0, 0, 1, 0, 0);
    const W = WORLD / 2;
    // sərhəd divarları (hər səhnədə)
    const wallC = '#3b4a57';
    addBox(0, -W + 20, WORLD, 40, 160, wallC); addBox(0, W - 20, WORLD, 40, 160, wallC);
    addBox(-W + 20, 0, 40, WORLD - 80, 160, wallC); addBox(W - 20, 0, 40, WORLD - 80, 160, wallC);

    if (name === 'line') {
      g.fillStyle = '#f1f1ec'; g.fillRect(0, 0, FLOOR_PX, FLOOR_PX);
      g.strokeStyle = 'rgba(0,0,0,0.05)'; g.lineWidth = 1;
      for (let i = 0; i <= FLOOR_PX; i += 100 * S) { g.beginPath(); g.moveTo(i, 0); g.lineTo(i, FLOOR_PX); g.stroke(); g.beginPath(); g.moveTo(0, i); g.lineTo(FLOOR_PX, i); g.stroke(); }
      const pts = [];
      for (let i = 0; i <= 720; i++) {
        const t = (i / 720) * Math.PI * 2;
        const r = 1 + 0.16 * Math.sin(3 * t) + 0.06 * Math.cos(5 * t);
        pts.push([1050 * r * Math.cos(t), 760 * r * Math.sin(t)]);
      }
      linePath = pts;
      g.strokeStyle = '#111'; g.lineWidth = 19 * S; g.lineJoin = g.lineCap = 'round';
      g.beginPath(); pts.forEach(([x, z], i) => (i ? g.lineTo(toPx(x), toPx(z)) : g.moveTo(toPx(x), toPx(z)))); g.closePath(); g.stroke();
      // start işarəsi
      g.fillStyle = '#ff2244'; g.fillRect(toPx(pts[0][0]) - 40 * S, toPx(pts[0][1]) - 4 * S, 80 * S, 8 * S);
      const [x0, z0] = pts[0], [x1, z1] = pts[4];
      const hdir = Math.atan2(-(x1 - x0), -(z1 - z0));     // irəli = (-sin h, -cos h)
      // robotu sensorları xəttin üstünə düşəcək şəkildə yerləşdir
      const sy = (current.ir[0] || { y: 68 }).y;
      startPose = { x: x0 + Math.sin(hdir) * sy, z: z0 + Math.cos(hdir) * sy, h: hdir };
      addCyl(0, 0, 120, 120, '#4d8dff');
    } else if (name === 'obstacle') {
      g.fillStyle = '#c9cfd4'; g.fillRect(0, 0, FLOOR_PX, FLOOR_PX);
      g.strokeStyle = 'rgba(0,0,0,0.08)'; g.lineWidth = 2;
      for (let i = 0; i <= FLOOR_PX; i += 250 * S) { g.beginPath(); g.moveTo(i, 0); g.lineTo(i, FLOOR_PX); g.stroke(); g.beginPath(); g.moveTo(0, i); g.lineTo(FLOOR_PX, i); g.stroke(); }
      const cols = ['#4d8dff', '#b44fff', '#39b36b', '#e0a400', '#e05a5a'];
      const spots = [[-700, -600], [600, -750], [-950, 350], [250, 450], [900, 250], [-300, 950], [800, 900], [-1050, -1050], [1150, -150], [-450, -100]];
      spots.forEach(([x, z], i) => {
        if (i % 3 === 0) addCyl(x, z, rnd(60, 110), rnd(90, 200), cols[i % cols.length]);
        else addBox(x, z, rnd(120, 320), rnd(120, 320), rnd(90, 220), cols[i % cols.length]);
      });
      addBox(-200, -1050, 900, 40, 120, '#3b4a57');
      addBox(450, 1100, 40, 700, 120, '#3b4a57');
      startPose = { x: 0, z: -300, h: Math.PI / 4 };
    } else if (name === 'station' && DK) {
      // ağ döşəmə, stansiya şimal divarına yaxın; qara xətt stansiyanın mərkəzindən çıxır, T zolağı (CAD ölçüləri)
      dockPose = { x: 0, z: 950, h: 0 };
      g.fillStyle = '#f1f1ec'; g.fillRect(0, 0, FLOOR_PX, FLOOR_PX);
      g.strokeStyle = 'rgba(0,0,0,0.05)'; g.lineWidth = 1;
      for (let i = 0; i <= FLOOR_PX; i += 100 * S) { g.beginPath(); g.moveTo(i, 0); g.lineTo(i, FLOOR_PX); g.stroke(); g.beginPath(); g.moveTo(0, i); g.lineTo(FLOOR_PX, i); g.stroke(); }
      const quad = (pts) => {
        g.beginPath();
        pts.forEach(([lx, ly], i) => { const [wx, wz] = dockToWorld(lx, ly); i ? g.lineTo(toPx(wx), toPx(wz)) : g.moveTo(toPx(wx), toPx(wz)); });
        g.closePath(); g.fill();
      };
      g.fillStyle = '#111';
      // əsas xətt stansiyadan qarşı divara qədər (~2.2 m): avtomatik qayıdışda robot onu tez tapır
      quad([[-9.5, DK.line_start], [9.5, DK.line_start], [9.5, 2150], [-9.5, 2150]]);
      quad([[-50, DK.t_y[0] + 0.25], [50, DK.t_y[0] + 0.25], [50, DK.t_y[1] - 0.25], [-50, DK.t_y[1] - 0.25]]);   // T
      DK.station_polys.forEach((P) => addPoly(P.map(([lx, ly]) => dockToWorld(lx, ly)), DK.station_h, true));
      stationGroup.position.set(dockPose.x, 0, dockPose.z);
      stationGroup.rotation.y = dockPose.h;
      stationGroup.visible = true;
      startPose = { x: 0, z: 150, h: Math.PI };       // xətt üzərində, stansiyaya tərəf baxır (~80 sm)
    } else {
      arenaR = 720; arenaBorder = 45;
      g.fillStyle = '#26292d'; g.fillRect(0, 0, FLOOR_PX, FLOOR_PX);
      g.fillStyle = '#0c0c0c'; g.beginPath(); g.arc(toPx(0), toPx(0), (arenaR + arenaBorder) * S, 0, Math.PI * 2); g.fill();
      g.fillStyle = '#f4f4f0'; g.beginPath(); g.arc(toPx(0), toPx(0), arenaR * S, 0, Math.PI * 2); g.fill();
      g.strokeStyle = 'rgba(0,0,0,0.12)'; g.lineWidth = 3;
      for (const r of [150, 350]) { g.beginPath(); g.arc(toPx(0), toPx(0), r * S, 0, Math.PI * 2); g.stroke(); }
      for (let i = 0; i < 6; i++) {
        const a = (i / 6) * Math.PI * 2 + 0.4, r = rnd(330, 560);
        addCube(Math.cos(a) * r, Math.sin(a) * r);
      }
      startPose = { x: 0, z: 0, h: rnd(0, Math.PI * 2) };
    }
    floorData = g.getImageData(0, 0, FLOOR_PX, FLOOR_PX).data;
    floorTex.needsUpdate = true;
    resetRobot();
  }

  // ---------- robot state ("firmware")
  const set = Object.fromEntries(SETTINGS_DEF.map((x) => [x.k, x.d]));
  try { Object.assign(set, JSON.parse(localStorage.getItem('smarsSimSettings') || '{}')); } catch (_) { /* */ }
  const saveSet = () => { try { localStorage.setItem('smarsSimSettings', JSON.stringify(set)); } catch (_) { /* */ } };
  let current = null;                // aktiv konfiqurasiya (layout.configs[k])
  let cfgKey = 'A';
  const st = {
    x: 0, z: 0, h: 0, vl: 0, vr: 0, sL: 0, sR: 0,
    mode: 0, dir: 'S', lastDrive: 0, status: '', grip: 90, elbow: 90,
    dist: 999, distHit: null, ir: [0, 0, 0], blk: [0, 0, 0],
    stalled: 0, ai: { s: 'go', t: 0, turn: 1, last: 0, search: 0 }, t: 0,
    soc: 0.85, vbat: 8.0, det: false, detSince: 0, dk: { s: 'idle', t0: 0 },
  };
  function resetRobot() {
    Object.assign(st, { x: startPose.x, z: startPose.z, h: startPose.h, vl: 0, vr: 0, dir: 'S', stalled: 0, det: false, detSince: 0 });
    st.ai = { s: 'go', t: 0, turn: 1, last: 0, search: 0 };
    st.dk = { s: 'idle', t0: st.t };
    st.soc = sceneName === 'station' ? 0.3 : 0.85;
    snapCamera();
  }
  function setConfig(k) {
    if (!L.configs[k]) return;
    cfgKey = k; current = L.configs[k];
    Object.entries(cfgGroups).forEach(([n, g2]) => { g2.visible = n === k; });
    irDots.forEach((d, i) => { d.visible = !!current.ir[i] && showHelpers; });
    if (!current.ir.length && st.mode === 2) st.mode = 0;
  }

  // ---------- sensors
  function usOrigin() {
    const c = Math.cos(st.h), s = Math.sin(st.h), y = current.us.y;
    return [st.x - y * s, st.z - y * c];
  }
  function measureUS() {
    const [ox, oz] = usOrigin();
    const hz = current.us.z + GZ;
    let best = Infinity, bestDir = null;
    for (let i = -4; i <= 4; i++) {
      const a = st.h + (i / 4) * (15 * Math.PI / 180);
      const dx = -Math.sin(a), dz = -Math.cos(a);
      for (const o of statics) {
        const t = o.kind === 'box' ? rayBox(ox, oz, dx, dz, o)
          : o.kind === 'poly' ? rayPoly(ox, oz, dx, dz, o.poly) : rayCircle(ox, oz, dx, dz, o.x, o.z, o.r);
        if (t < best && o.h >= hz - t * TAN15) { best = t; bestDir = [dx, dz]; }
      }
      for (const c of cubes) {
        const t = rayCircle(ox, oz, dx, dz, c.x, c.z, c.s * 0.55);
        if (t < best && c.h >= hz - t * TAN15) { best = t; bestDir = [dx, dz]; }
      }
    }
    let cm = best / 10;
    if (!(cm >= 2 && cm <= 400)) { st.dist = 999; st.distHit = null; return; }
    if (set.noise) cm += rnd(-0.4, 0.4);
    st.dist = Math.round(cm);
    st.distHit = [ox + bestDir[0] * best, oz + bestDir[1] * best];
  }
  function readIR() {
    const c = Math.cos(st.h), s = Math.sin(st.h);
    current.ir.forEach((p, i) => {
      const wx = st.x + p.x * c - p.y * s, wz = st.z - p.x * s - p.y * c;
      let v = darkness(wx, wz) * 100;
      if (set.noise) v += rnd(-1.5, 1.5);
      v = Math.max(0, Math.min(100, v));
      st.ir[i] = Math.round(v * 10) / 10;
      st.blk[i] = v > set.irThr ? 1 : 0;
    });
  }

  // ---------- control logic (rejimlər)
  const spd = (v) => (v / 255) * set.maxSpd;
  function drive(l, r) { st.vl = l; st.vr = r; }

  // ---------- rejim 4: şarj stansiyasına qayıt — firmware/smars_dock.h ilə eyni vəziyyət maşını
  const DP = { spin: 120, seek: 140, seekLeg: 20000, seekStuck: 2500, seekBack: 300, tArm: 700, tConfirm: 40, lost: 700, brake: 180, spinMin: 450, spinTimeout: 6000,
               revTimeout: 3000, retryFwd: 800, backoff: 1200, maxRetries: 3, safety: 3, tNear: 10, detConfirm: 60, detLoss: 600, kP: 1.6, inner: -20 };
  const pwm = (l, r) => drive(spd(l), spd(r));
  function dgo(s) {
    const d = st.dk; d.s = s; d.t0 = st.t; d.tSince = 0;
    if (s === 'docked') d.lastDet = st.t;
    if (s === 'seek') {
      d.legStart = d.distRefT = st.t; d.distRef = -100; d.backUntil = d.turnUntil = 0; d.turnDir = 1;
      d.rng = ((d.rng || 0x2545f491) ^ Math.round(st.t)) >>> 0;
    }
    if (s === 'spin' || s === 'turn') pwm(DP.spin, -DP.spin);            // yeni vəziyyətin əmri DƏRHAL
    else if (s === 'center') pwm(-DP.spin, DP.spin);
    else if (s === 'reverse' || s === 'backoff') pwm(-set.dockRev, -set.dockRev);
    else if (['stop', 'docked', 'charged', 'idle', 'failed'].includes(s)) pwm(0, 0);
  }
  function dockStart() {
    st.dk = { s: 'idle', t0: st.t, retries: 0, lastDir: 1, lastSeen: st.t };
    if (st.det) return dgo('docked');
    return dgo(st.blk.some(Boolean) ? 'follow' : 'seek');
  }
  function followDock(c, l, r) {
    const v = set.lineSpd, inn = (v * DP.inner) / 100;
    if (c || l || r) {                                         // analoq: sol − sağ qaralıq fərqi
      const e = Math.max(-1, Math.min(1, (st.ir[1] - st.ir[2]) / 100));
      const k = DP.kP * e;
      pwm(Math.min(255, Math.max(inn, v * (1 - k))), Math.min(255, Math.max(inn, v * (1 + k))));
      st.dk.lastDir = e > 0 ? -1 : 1;
      return;
    }
    const d = st.dk.lastDir || 1;
    pwm(d * DP.spin * 0.6, -d * DP.spin * 0.6);
  }
  function dockUpdate() {
    const d = st.dk, now = st.t, t = now - d.t0;
    const [c, l, r] = st.blk;
    if (st.det) { if (!st.detSince) st.detSince = now; } else st.detSince = 0;
    const detOk = st.detSince && now - st.detSince >= DP.detConfirm;
    switch (d.s) {
      case 'seek': {      // düz gedir; maneədə / ilişəndə: bir az geri + təsadüfi tərəfə ~60°…170° dönür
        if (c || l || r) { d.lastSeen = now; dgo('follow'); break; }
        if (d.backUntil > now) { pwm(-DP.seek, -DP.seek); break; }
        if (d.turnUntil > now) { pwm(d.turnDir * DP.seek, -d.turnDir * DP.seek); break; }
        // ilişmə enkodersiz: irəli gedərkən HC-SR04 məsafəsi 2.5 san dəyişmir (məs. yanı divara dirənib)
        if (Math.abs(st.dist - d.distRef) > 2) { d.distRef = st.dist; d.distRefT = now; }
        const stuck = st.dist < 400 && now - d.distRefT > DP.seekStuck;
        if (st.dist < 15 || stuck || now - d.legStart >= DP.seekLeg) {
          d.rng = (Math.imul(d.rng, 1103515245) + 12345) >>> 0;
          d.backUntil = now + DP.seekBack;
          d.turnUntil = d.backUntil + 350 + ((d.rng >>> 16) % 600);
          d.turnDir = (d.rng >>> 27) & 1 ? 1 : -1;
          d.legStart = d.distRefT = d.turnUntil;
          pwm(-DP.seek, -DP.seek);
        } else pwm(DP.seek, DP.seek);
        break;
      }
      case 'follow': case 'retry': {
        const all = c && l && r;
        // T: tArm-dan əvvəl yalnız stansiya artıq qarşıdadırsa (HC-SR04 ≤ 10 sm) qəbul olunur
        if (d.s === 'follow' && all && (t >= DP.tArm || st.dist <= DP.tNear)) {
          if (!d.tSince) d.tSince = now;
          if (now - d.tSince >= DP.tConfirm) { dgo('stop'); break; }
        } else d.tSince = 0;
        if (d.s === 'follow' && st.dist <= DP.safety) { dgo('stop'); break; }
        if (d.s === 'retry' && t >= d.retryMs) { dgo('reverse'); break; }
        if (c || l || r) d.lastSeen = now;
        if (!(c || l || r) && now - d.lastSeen > DP.lost && d.s === 'follow') { dgo('turn'); break; }
        followDock(c, l, r);
        break;
      }
      case 'turn':
        pwm(DP.spin, -DP.spin);
        if (t > DP.spinMin && c) { d.lastSeen = now; dgo('follow'); } else if (t > DP.spinTimeout) dgo('seek');
        break;
      case 'stop':
        pwm(0, 0);
        if (t >= DP.brake) { d.phase = 0; dgo('spin'); }
        break;
      case 'spin':
        pwm(DP.spin, -DP.spin);
        if (d.phase === 0 && !c && t > 120) d.phase = 1;                       // xətdən/T-dən çıxdı
        else if (d.phase === 1 && c && t > DP.spinMin) { d.phase = 2; d.mark = now; }
        else if (d.phase === 2 && !c) { d.centerMs = (now - d.mark) / 2; dgo('center'); break; }
        if (t > DP.spinTimeout) {                    // fırlana bilmir (stansiyaya çox yaxın / ilişib) → geri çəkil, yenidən
          d.retries = (d.retries || 0) + 1;
          dgo(d.retries > DP.maxRetries ? 'failed' : 'backoff');
        }
        break;
      case 'backoff':
        pwm(-set.dockRev, -set.dockRev);
        if (t >= DP.backoff) { d.lastSeen = now; dgo(c || l || r ? 'follow' : 'seek'); }
        break;
      case 'center':
        pwm(-DP.spin, DP.spin);
        if (t >= d.centerMs) { d.revLimit = DP.revTimeout; dgo('reverse'); }
        break;
      case 'reverse':
        pwm(-set.dockRev, -set.dockRev);
        if (detOk) { dgo('docked'); break; }
        if (t > (d.revLimit || DP.revTimeout)) {
          d.retries = (d.retries || 0) + 1;
          if (d.retries > DP.maxRetries) { dgo('failed'); break; }
          d.retryMs = DP.retryFwd;
          d.revLimit = DP.revTimeout + (DP.retryFwd * set.lineSpd) / set.dockRev;
          d.lastSeen = now;
          dgo('retry');
        }
        break;
      case 'docked':
        pwm(0, 0);
        if (!st.det) { if (now - d.lastDet > DP.detLoss) { d.revLimit = 800; dgo('reverse'); } break; }
        d.lastDet = now;
        if (st.soc >= 0.995) dgo('charged');
        break;
      case 'charged':
        pwm(0, 0);
        break;
      case 'undock':
        if (t < 1500) followDock(c, l, r); else dgo('idle');
        break;
      default:
        pwm(0, 0);
    }
    st.status = DOCK_NAMES[d.s] || '';
  }
  // DET: plunjerlər lentə toxunur VƏ mikrovaç basılıb (dok çərçivəsində quyruğun ucu)
  function computeDet() {
    if (!dockPose || !DK) return false;
    const c = Math.cos(dockPose.h), s = Math.sin(dockPose.h);
    const dx = st.x - dockPose.x, dz = st.z - dockPose.z;
    const lx = dx * c - dz * s, ly = -(dx * s + dz * c);
    const dh = Math.atan2(Math.sin(st.h - dockPose.h), Math.cos(st.h - dockPose.h));
    if (Math.abs(dh) > (4 * Math.PI) / 180) return false;
    // quyruğun ucu (robotun yerli nöqtəsi (0, tail_end)) dok çərçivəsində: kontakt robotun mərkəzindən yox, quyruqdan asılıdır
    const tx = lx - DK.tail_end * Math.sin(dh), ty = ly + DK.tail_end * Math.cos(dh);
    return Math.abs(tx) < 3 && ty <= DK.contact_tail_end && ty <= DK.switch_tail_end;
  }
  // batareya: 2S 800 mAh; yük cərəyanı (ESP32 + sensorlar 0.12 A + mühərriklər) və şarj (IP2326: CC 1 A → CV).
  // Stansiya səhnəsində zaman sürətləndirilir (robotun hərəkəti yox, yalnız batareya): boşalma ×drainX, şarj ×chgX
  function battery(dt) {
    const Iload = 0.12 + (0.45 * (Math.abs(st.vl) + Math.abs(st.vr))) / (2 * set.maxSpd);
    const h = dt / 3600;
    let Ibat;                                          // batareyanın cərəyanı: + şarj, − boşalma
    if (st.det) {
      // IP2326 ~1 A verir: əvvəl robotun öz yükünü qarşılayır, artığı batareyaya gedir; ~90%-dən sonra CV — cərəyan azalır
      const cc = 1.0 - Iload, cv = st.soc < 0.9 ? Infinity : ((1 - st.soc) / 0.1) * 0.9;
      Ibat = Math.max(0, Math.min(cc, cv));
      st.soc += (Ibat * (dockPose ? set.chgX : 1) * h) / 0.8;
    } else {
      Ibat = -Iload;
      st.soc -= (Iload * (dockPose ? set.drainX : 1) * h) / 0.8;
    }
    st.soc = Math.max(0, Math.min(1, st.soc));
    st.vbat = voltsFromPct(st.soc * 100) + 0.12 * Ibat;          // daxili müqavimət ~0.12 Ω
  }

  function controller(dt, now) {
    if (st.soc <= 0.01 && !st.det) { drive(0, 0); st.status = 'BATAREYA BOŞDUR'; return; }
    if (st.mode === 4) return dockUpdate();
    if (st.mode >= 1 && st.mode <= 3 && set.autoHome && dockPose && current.ir.length && st.soc * 100 < set.lowPct) {
      st.mode = 4; dockStart(); return dockUpdate();                 // batareya azdır → avtomatik qayıdış
    }
    if (st.mode === 0) {
      st.status = '';
      if (st.dir !== 'S' && now - st.lastDrive > 700) st.dir = 'S';      // ölü-adam dayanması
      const v = spd(set.tankSpd), k = set.turnK;
      const m = { F: [v, v], B: [-v, -v], L: [-v * k, v * k], R: [v * k, -v * k], S: [0, 0] }[st.dir] || [0, 0];
      return drive(m[0], m[1]);
    }
    const ai = st.ai; ai.t += dt * 1000;
    const go = (s) => { ai.s = s; ai.t = 0; };
    if (st.mode === 1) {                                     // maneədən qaçma
      const v = spd(set.tankSpd);
      const near = (st.dist < set.obsDist) || st.stalled > 150;      // ilişmə = "bamper" kimi
      if (ai.s === 'go') { st.status = 'İRƏLİ — YOL AÇIQ'; drive(v, v); if (near) { go('back'); ai.turn = Math.random() < 0.5 ? -1 : 1; } }
      else if (ai.s === 'back') {
        st.status = 'MANEƏ! GERİ'; drive(-v * 0.8, -v * 0.8);
        if (ai.t > set.obsBack || (st.stalled > 150 && ai.t > 100)) go('turn');
      } else if (ai.s === 'turn') {
        st.status = ai.turn > 0 ? 'DÖNÜR → SAĞ' : 'DÖNÜR ← SOL';
        drive(v * 0.7 * ai.turn, -v * 0.7 * ai.turn);
        if (st.stalled > 150) { ai.turn = -ai.turn; go('back'); }  // kürəyin qanadı ilişdi — geri çəkil, əks tərəfə dön
        else if ((ai.t > 300 && st.dist > set.obsDist + 15) || ai.t > 1600) go('go');
      }
      return;
    }
    if (st.mode === 2) {                                     // line following
      const v = spd(set.lineSpd);
      const on = set.lineInv ? st.blk.map((b) => 1 - b) : st.blk;
      const [c, l, r] = on;
      if (l && !r) { st.status = 'XƏTT ← SOLA'; drive(-v * 0.15, v); ai.last = -1; ai.search = 0; }
      else if (r && !l) { st.status = 'XƏTT → SAĞA'; drive(v, -v * 0.15); ai.last = 1; ai.search = 0; }
      else if (c || (l && r)) { st.status = 'XƏTT İZLƏNİR'; drive(v, v); ai.search = 0; }
      else {
        ai.search += dt * 1000;
        st.status = 'XƏTT İTİB — AXTARIR';
        const d = ai.last || 1;
        if (ai.search < 1800) drive(v * 0.55 * d, -v * 0.55 * d); else drive(v * 0.5, v * 0.5);
        if (ai.search > 2600) ai.search = 0;
      }
      return;
    }
    if (st.mode === 3) {                                     // arena: kubları çıxar
      const v = spd(set.arenaSpd), vs = spd(Math.min(set.tankSpd, 150));
      const edge = current.ir.length ? st.blk.some(Boolean) : false;
      if (ai.s !== 'back' && ai.s !== 'spin' && ai.s !== 'push' && edge) {
        if (st.dist < 18 || (ai.near && st.t - ai.near < 400)) go('push'); else go('back');
        ai.near = 0;
      }
      if (ai.s === 'push') { st.status = 'KUBU KƏNARA İTƏLƏYİR'; drive(v, v); if (ai.t > 260) go('back'); }
      else if (ai.s === 'back') { st.status = 'KƏNAR! GERİ'; drive(-v * 0.7, -v * 0.7); if (ai.t > 520) { go('spin'); ai.turn = Math.random() < 0.5 ? -1 : 1; } }
      else if (ai.s === 'spin') { st.status = 'DÖNÜR'; drive(vs * ai.turn, -vs * ai.turn); if (ai.t > rnd(500, 900)) go('seek'); }
      else if (ai.s === 'attack') {
        // HC-SR04 2 sm-dən yaxını görmür (999 qaytarır): kub kürəyə dirənibsə bunu "təmas" say və itələməyə davam et
        if (st.dist < 12) ai.near = st.t;
        const touching = st.dist >= 900 && ai.near && st.t - ai.near < 2500;
        if (touching) ai.near = st.t;
        st.status = touching ? 'TƏMAS — İTƏLƏYİR' : 'HƏDƏF — HÜCUM';
        drive(v, v);
        if (!touching && st.dist > set.arenaSee * 1.3 && ai.t > 600) { ai.near = 0; go('seek'); }
      } else if (ai.s === 'seek') {
        // fırlan; hədəf görünəndə görünmə müddətini ölç (kubun "eni"), itəndə yarısı qədər geri dön → mərkəzə tuş
        st.status = current.ir.length ? 'KUB AXTARILIR' : 'KUB AXTARILIR (IR YOX — KƏNARI GÖRMÜR)';
        drive(vs * 0.45, -vs * 0.45);
        if (st.dist < set.arenaSee) { ai.seen = (ai.seen || 0) + dt * 1000; }
        else if (ai.seen > 0) { ai.back = ai.seen / 2; ai.seen = 0; go('center'); }
        if (ai.s === 'seek' && ai.t > 4500 && !ai.seen) go('roam');
      } else if (ai.s === 'center') {
        st.status = 'HƏDƏFƏ TUŞLANIR';
        drive(-vs * 0.45, vs * 0.45);
        if (ai.t >= ai.back) go(st.dist < set.arenaSee * 1.2 ? 'attack' : 'seek');
      } else { st.status = 'GƏZİR'; drive(vs, vs); if (ai.t > 900 || st.dist < set.arenaSee) go('seek'); }
      if (st.stalled > 500 && ai.s !== 'back') go('back');
    }
  }

  // ---------- physics
  // robotun yuxarıdan görünüşü: gövdə+tırtıllar düzbucaqlısı + U-kürəyin iki yan qanadı (CAD ölçüləri)
  function bodyRects() {
    const hw = L.track.x + L.track.half_width, bw = current.half_width, fc = current.front_centre, fm = current.front_max;
    const r = [[-hw, hw, L.chassis.rear_y, fc], [-bw, -bw + 6, fc - 8, fm], [bw - 6, bw, fc - 8, fm]];
    if (DK) r.push(DK.tail);                       // arxa şarj quyruğu
    return r;
  }
  const footprints = (x, z, h) => bodyRects().map((r) => rectPoly(x, z, h, r[0], r[1], r[2], r[3]));
  const hitOne = (P, o) => (o.kind === 'box' || o.kind === 'poly' ? satOverlap(P, o.poly) : polyCircle(P, o.x, o.z, o.r));
  let colStatics = null;                         // fizika addımında robotun yaxınındakı maneələr (sürətli yoxlama)
  function hitsStatic(Ps) {
    const S = colStatics || statics;
    return Ps.some((P) => S.some((o) => hitOne(P, o)));
  }
  const robotR = () => Math.max(...bodyRects().flatMap((r) => [Math.hypot(r[0], r[2]), Math.hypot(r[1], r[2]), Math.hypot(r[0], r[3]), Math.hypot(r[1], r[3])]));
  function cubeHitsStatic(c) {
    const r = c.s * 0.5;
    return statics.some((o) => (o.kind === 'box'
      ? (c.x + r > o.x - o.w / 2 && c.x - r < o.x + o.w / 2 && c.z + r > o.z - o.d / 2 && c.z - r < o.z + o.d / 2)
      : o.kind === 'poly' ? polyCircle(o.poly, c.x, c.z, r) : Math.hypot(c.x - o.x, c.z - o.z) < r + o.r));
  }
  function pushCubes(nx, nz, nh) {
    // kublar robotun düzbucaqlısından kənara itələnir; divara sıxılırsa — robot dayanır
    const c0 = Math.cos(nh), s0 = Math.sin(nh);
    const moved = [];
    const rects = bodyRects();
    for (const c of cubes) for (const [x0, x1, y0, y1] of rects) {
      const r = c.s * 0.52;
      const dx = c.x - nx, dz = c.z - nz;
      const lx = dx * c0 - dz * s0, ly = -(dx * s0 + dz * c0);         // robotun yerli koordinatları
      const qx = Math.max(x0, Math.min(x1, lx)), qy = Math.max(y0, Math.min(y1, ly));
      let ex = lx - qx, ey = ly - qy;
      let d = Math.hypot(ex, ey);
      if (d >= r) continue;
      let pen;
      if (d < 1e-6) {                                                    // mərkəz içəridədir
        const opts2 = [[x1 - lx + r, 1, 0], [lx - x0 + r, -1, 0], [y1 - ly + r, 0, 1], [ly - y0 + r, 0, -1]].sort((a, b) => a[0] - b[0])[0];
        pen = opts2[0]; ex = opts2[1]; ey = opts2[2];
      } else { pen = r - d; ex /= d; ey /= d; }
      const wx = ex * c0 - ey * s0, wz = -ex * s0 - ey * c0;              // yerli normal → dünya
      const old = [c.x, c.z];
      c.x += wx * pen; c.z += wz * pen;
      if (ey > 0.5) {                                                    // kürək kubu öz üzünə paralel düzəldir
        const q = Math.PI / 2;
        const diff = ((((nh - c.a) % q) + q + q / 2) % q) - q / 2;
        c.a += diff * 0.1;
      }
      moved.push([c, old]);
    }
    // kub ↔ kub
    for (let i = 0; i < cubes.length; i++) for (let j = i + 1; j < cubes.length; j++) {
      const a = cubes[i], b = cubes[j], dx = b.x - a.x, dz = b.z - a.z, d = Math.hypot(dx, dz), m = (a.s + b.s) * 0.5;
      if (d > 0 && d < m) { const p = (m - d) / 2; a.x -= dx / d * p; a.z -= dz / d * p; b.x += dx / d * p; b.z += dz / d * p; }
    }
    if (cubes.some(cubeHitsStatic)) { moved.forEach(([c, o]) => { c.x = o[0]; c.z = o[1]; }); return false; }
    return true;
  }
  // Divara toxunan robot onun boyu sürüşür (və bir az fırlanır): huni quyruğu boğaza belə yönəldir, divara yanakı
  // dəyən robot da ilişib qalmır. Şərt: əmr olunan hərəkət istiqamətində ən azı bir az irəliləmə — düz divara
  // dirənən robot yana "axmır", dayanır (stalled).
  // Namizəd düzəlişlər: yana sürüşmə (addım uzunluğuna nisbətən — sürətdən asılı olmasın) və kiçik fırlanma.
  // Qısa addımlar (f < 1) + fırlanma robotu "düzləndirir": küncü divara dəyən robot itələdikcə ona paralel olur.
  const G_K = [0.4, 1.0, 2.0], G_A = [0.3, 0.6].map((d) => (d * Math.PI) / 180);
  const GUIDE = [];
  for (const k of G_K) GUIDE.push([k, 0], [-k, 0]);
  for (const a of G_A) GUIDE.push([0, a], [0, -a]);
  for (const k of [0.4, 1.0]) for (const sk of [1, -1]) for (const sa of [1, -1]) GUIDE.push([sk * k, sa * G_A[0]]);
  function tryGuided(nx, nz, nh, tryPose) {
    if (!hitsStatic(footprints(nx, nz, nh))) return false;      // kubla bloklanıb (divarla yox) — sürüşmə yoxdur
    const L = Math.max(0.3, Math.hypot(nx - st.x, nz - st.z), robotR() * Math.abs(nh - st.h));
    const c = Math.cos(nh), s = Math.sin(nh);
    for (const f of [1, 0.5, 0.25, 0.1]) {           // hərəkət istiqamətində ən azı bir az irəliləmə şərtdir
      const bx = st.x + (nx - st.x) * f, bz = st.z + (nz - st.z) * f, bh = st.h + (nh - st.h) * f;
      if (f < 1 && tryPose(bx, bz, bh)) return true;  // dayağa qədər qısa addım (yan sürüşməsiz)
      for (const [k, a] of GUIDE) if (tryPose(bx + k * L * c, bz - k * L * s, bh + a)) return true;
    }
    return false;
  }
  function physics(dt) {
    const v = (st.vl + st.vr) / 2, w = (st.vr - st.vl) / TRACK_W;
    const nh = st.h + w * dt;
    const nx = st.x - Math.sin(nh) * v * dt, nz = st.z - Math.cos(nh) * v * dt;
    let moved = false;
    const tryPose = (x, z, h) => {
      if (hitsStatic(footprints(x, z, h))) return false;
      const snap = cubes.map((c) => [c.x, c.z]);
      if (!pushCubes(x, z, h)) { cubes.forEach((c, i) => { c.x = snap[i][0]; c.z = snap[i][1]; }); return false; }
      st.x = x; st.z = z; st.h = h; return true;
    };
    if (Math.abs(v) + Math.abs(w) < 1e-6) { st.stalled = 0; return; }
    const rr = robotR() + Math.abs(v * dt) + 40;
    colStatics = statics.filter((o) => st.x + rr >= o.bb[0] && st.x - rr <= o.bb[2] && st.z + rr >= o.bb[1] && st.z - rr <= o.bb[3]);
    if (tryPose(nx, nz, nh)) moved = true;
    else if (tryGuided(nx, nz, nh, tryPose)) moved = true;                // divar / huni boyu sürüşmə
    else if (Math.abs(w) > 1e-6 && tryPose(st.x, st.z, nh)) moved = true;
    colStatics = null;
    st.stalled = moved ? 0 : st.stalled + dt * 1000;
    // tırtıllar yalnız robot həqiqətən hərəkət edəndə fırlanır (sürüşmə yoxdur)
    if (moved) { st.sL += st.vl * dt; st.sR += st.vr * dt; }
    for (const c of cubes) if (arenaR && !c.out && Math.hypot(c.x, c.z) > arenaR + arenaBorder) { c.out = true; c.mesh.material = mat('#5a6570'); opts.onEvent && opts.onEvent('cube-out'); }
  }

  // ---------- track / render update
  const tmpM = new THREE.Matrix4(), tmpB = new THREE.Matrix4();
  function trackPose(s) {
    s = ((s % TRACK_LEN) + TRACK_LEN) % TRACK_LEN;
    const Ls = 2 * AY, arc = Math.PI * R;
    if (s < Ls) return [-AY + s, AZ + R, Math.PI / 2];
    s -= Ls;
    if (s < arc) { const th = Math.PI / 2 - s / R; return [AY + R * Math.cos(th), AZ + R * Math.sin(th), th]; }
    s -= arc;
    if (s < Ls) return [AY - s, AZ - R, -Math.PI / 2];
    s -= Ls;
    const th = -Math.PI / 2 - s / R;
    return [-AY + R * Math.cos(th), AZ + R * Math.sin(th), th];
  }
  function updateTracks() {
    const pitch = TRACK_LEN / NLINK;
    for (let side = 0; side < 2; side++) {
      const x = side ? L.track.x : -L.track.x, off = side ? st.sR : st.sL;
      for (let i = 0; i < NLINK; i++) {
        const [y, z, th] = trackPose(i * pitch + off);
        const c = Math.cos(th), s = Math.sin(th);
        tmpM.set(1, 0, 0, x, 0, s, c, y, 0, -c, s, z, 0, 0, 0, 1);
        links.setMatrixAt(side * NLINK + i, tmpM);
      }
    }
    links.instanceMatrix.needsUpdate = true;
    wheels.forEach((w) => { w.m.rotation.x = -(w.side ? st.sR : st.sL) / R; });
  }
  let showHelpers = true;
  function render() {
    robot.position.set(st.x, 0, st.z);
    robot.rotation.y = st.h;
    updateTracks();
    elbow.rotation.x = ((st.elbow - 90) * Math.PI) / 180;
    const open = Math.max(0, Math.min(1, (st.grip - set.grpLo) / Math.max(1, set.grpHi - set.grpLo)));
    fingers[0].position.x = -3 - open * 7; fingers[1].position.x = 3 + open * 7;
    cubes.forEach((c) => { c.mesh.position.set(c.x, c.h / 2, c.z); c.mesh.rotation.y = c.a; });
    // HC-SR04 konusu
    helpers.visible = showHelpers;
    const [ox, oz] = usOrigin();
    const len = st.dist < 400 ? st.dist * 10 : 600;
    cone.scale.set(len * TAN15, len, len * TAN15);
    cone.position.set(ox - Math.sin(st.h) * len / 2, current.us.z + GZ, oz - Math.cos(st.h) * len / 2);
    cone.rotation.set(0, 0, 0);
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, -1, 0), new THREE.Vector3(-Math.sin(st.h), 0, -Math.cos(st.h)));
    coneMat.color.set(st.dist < set.obsDist ? '#ff2244' : '#00e5ff');
    hitDot.visible = !!st.distHit;
    if (st.distHit) hitDot.position.set(st.distHit[0], current.us.z + GZ, st.distHit[1]);
    const c = Math.cos(st.h), s = Math.sin(st.h);
    current.ir.forEach((p, i) => {
      irDots[i].position.set(st.x + p.x * c - p.y * s, 0.6, st.z - p.x * s - p.y * c);
      irDots[i].material.color.set(st.blk[i] ? '#ff2244' : '#39ff14');
    });
    if (ledMats.S_led_green) ledMats.S_led_green.emissiveIntensity = st.det ? 1.4 : 0.0;   // "robot oturub"
  }

  // ---------- camera
  let camMode = 'follow';
  const lastTarget = new THREE.Vector3();
  function snapCamera() {
    const tgt = new THREE.Vector3(st.x, 40, st.z);
    const back = new THREE.Vector3(Math.sin(st.h), 0, Math.cos(st.h)).multiplyScalar(420);
    camera.position.copy(tgt).add(back).add(new THREE.Vector3(0, 300, 0));
    controls.target.copy(tgt); lastTarget.copy(tgt);
    controls.update();
  }
  function updateCamera(dt) {
    const tgt = new THREE.Vector3(st.x, 40, st.z);
    if (camMode === 'follow') {
      camera.position.add(tgt.clone().sub(lastTarget));
      controls.target.copy(tgt);
    } else if (camMode === 'chase') {
      const want = tgt.clone().add(new THREE.Vector3(Math.sin(st.h) * 360, 230, Math.cos(st.h) * 360));
      camera.position.lerp(want, Math.min(1, dt * 4));
      controls.target.copy(tgt);
    } else if (camMode === 'top') {
      camera.position.lerp(new THREE.Vector3(st.x, 2600, st.z + 1), Math.min(1, dt * 3));
      controls.target.copy(tgt);
    } else if (camMode === 'fpv') {
      const [ox, oz] = usOrigin();
      camera.position.set(ox + Math.sin(st.h) * 30, current.us.z + GZ + 45, oz + Math.cos(st.h) * 30);
      controls.target.set(ox - Math.sin(st.h) * 400, 20, oz - Math.cos(st.h) * 400);
    }
    lastTarget.copy(tgt);
    controls.enabled = camMode !== 'fpv';
    controls.update();
  }

  // ---------- loop
  let raf = 0, last = performance.now(), acc = 0, usT = 0, fps = 0, fpsN = 0, fpsT = 0, destroyed = false;
  const DT = 1 / 120;
  function tick() {
    st.t += DT * 1000;
    usT += DT * 1000;
    if (usT >= 60) { usT = 0; measureUS(); }        // HC-SR04 ~16 Hz
    readIR();
    st.det = computeDet();
    controller(DT, st.t);
    physics(DT);
    battery(DT);
  }
  function frameLoop(now) {
    if (destroyed) return;
    raf = requestAnimationFrame(frameLoop);
    let dt = (now - last) / 1000; last = now;
    if (dt > 0.1) dt = 0.1;
    acc += dt;
    while (acc >= DT) { tick(); acc -= DT; }
    render();
    updateCamera(dt);
    renderer.render(scene, camera);
    fpsN++; fpsT += dt; if (fpsT > 0.5) { fps = Math.round(fpsN / fpsT); fpsN = 0; fpsT = 0; }
  }
  function resize() {
    if (headless) return;
    const w = container.clientWidth || 640, h = container.clientHeight || 400;
    renderer.setSize(w, h, false);
    renderer.domElement.style.width = '100%'; renderer.domElement.style.height = '100%';
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }
  const ro = headless ? null : new ResizeObserver(resize);
  if (ro) ro.observe(container);

  // ---------- virtual firmware API (real robotla eyni)
  function status() {
    const s = {
      mode: st.mode, status: st.status, dist: st.dist, rssi: -40, sta: 'brauzer simulyasiyası',
      tankDir: st.mode ? 'AUTO' : ({ F: 'İRƏLİ', B: 'GERİ', L: 'SOL', R: 'SAĞ', S: 'STOP' }[st.dir] || 'STOP'),
      tankSpd: set.tankSpd, gripAngle: st.grip, elbowAngle: st.elbow, sv: 1,
      bat: Math.round(st.soc * 100), vbat: Math.round(st.vbat * 100) / 100, chg: st.det ? 1 : 0,
      dock: st.mode === 4 ? (DOCK_NAMES[st.dk.s] || '') : (st.det ? DOCK_NAMES.docked : ''),
    };
    if (current.ir.length) { s.ir = st.ir.slice(0, current.ir.length); s.blk = st.blk.slice(0, current.ir.length); }
    return s;
  }
  function handle(path, q) {
    const g = (k) => (q && q[k] !== undefined ? String(q[k]) : null);
    switch (path) {
      case '/status': return [200, status()];
      case '/settings': return [200, { g: GROUPS, s: SETTINGS_DEF.map((x) => ({ ...x, v: set[x.k] })) }];
      case '/mode': {
        const m = parseInt(g('m'), 10);
        if (!(m >= 0 && m <= 4)) return [400, 'bad mode'];
        if (m === 2 && !current.ir.length) return [409, 'IR sensor yoxdur (bu konfiqurasiyada)'];
        if (m === 4 && !current.ir.length) return [409, 'stansiyaya xətt üzrə qayıdır — IR sensorlar lazımdır'];
        if (m === 4 && !dockPose) return [409, "bu səhnədə şarj stansiyası yoxdur — 'Şarj stansiyası' səhnəsini seçin"];
        st.mode = m; st.dir = 'S'; st.ai = { s: m === 3 ? 'seek' : 'go', t: 0, turn: 1, last: 0, search: 0 };
        if (m === 4) dockStart(); else st.dk = { s: 'idle', t0: st.t };
        return [200, String(m)];
      }
      case '/tank': {
        if (g('spd') !== null) { set.tankSpd = Math.max(60, Math.min(255, parseInt(g('spd'), 10) || 180)); saveSet(); }
        if (g('dir') !== null) {
          if (st.mode !== 0) return [409, 'auto mode'];
          st.dir = 'FBLRS'.includes(g('dir')) ? g('dir') : 'S';
          st.lastDrive = st.t;
        }
        return [200, status().tankDir];
      }
      case '/servo':
        if (g('grip') !== null) st.grip = Math.max(set.grpLo, Math.min(set.grpHi, parseInt(g('grip'), 10) || 0));
        if (g('elbow') !== null) st.elbow = Math.max(set.elbLo, Math.min(set.elbHi, parseInt(g('elbow'), 10) || 0));
        return [200, 'ok'];
      case '/set': {
        const def = SETTINGS_DEF.find((x) => x.k === g('k'));
        if (!def) return [404, 'unknown key'];
        let v = parseFloat(g('v'));
        if (isNaN(v)) return [400, 'bad value'];
        v = Math.max(def.mn, Math.min(def.mx, v));
        if (def.t !== 'f') v = Math.round(v);
        set[def.k] = v; saveSet();
        return [200, String(v)];
      }
      case '/reset': {
        const gr = g('g');
        SETTINGS_DEF.forEach((x) => { if (gr === 'all' || x.g === gr) set[x.k] = x.d; });
        saveSet();
        return [200, 'ok'];
      }
      default: return [404, 'unknown'];
    }
  }

  setConfig(opts.config || 'A');
  buildScene(opts.scene || 'line');
  resize();
  if (!headless) raf = requestAnimationFrame(frameLoop);

  return {
    handle,
    configs: Object.fromEntries(Object.entries(L.configs).map(([k, c]) => [k, c.name])),
    get mode() { return st.mode; },
    setScene(n) { if (SCENES[n]) { st.mode = 0; st.dir = 'S'; buildScene(n); } },
    setConfig(k) { setConfig(k); resetRobot(); },
    reset() { st.mode = 0; st.dir = 'S'; buildScene(sceneName); },
    get scene() { return sceneName; },
    setCamera(m) { camMode = m; if (m === 'follow') snapCamera(); },
    setHelpers(v) { showHelpers = v; irDots.forEach((d, i) => { d.visible = v && !!current.ir[i]; }); },
    info() {
      return {
        fps, x: Math.round(st.x), z: Math.round(st.z), h: Math.round((((-st.h * 180) / Math.PI) % 360 + 360) % 360),
        v: Math.round(((st.vl + st.vr) / 2)), cubes: cubes.length, out: cubes.filter((c) => c.out).length,
        stalled: st.stalled > 200, mode: st.mode, status: st.status, dist: st.dist, cfg: cfgKey,
        bat: Math.round(st.soc * 100), vbat: st.vbat, chg: st.det, dock: DOCK_NAMES[st.dk.s] || '',
      };
    },
    setBattery(pct) { st.soc = Math.max(0, Math.min(1, pct / 100)); },
    place(x, z, h) { Object.assign(st, { x, z, h, vl: 0, vr: 0, dir: 'S' }); measureUS(); readIR(); snapCamera(); },
    // test üçün: simulyasiya zamanını irəli çək (headless rejimdə)
    step(seconds) { const n = Math.round(seconds / DT); for (let i = 0; i < n; i++) tick(); render(); },
    state() { return { dock: st.dk.s, retries: st.dk.retries || 0, soc: st.soc, vbat: st.vbat, det: st.det, dockPose, ai: st.ai.s, status: st.status, x: st.x, z: st.z, h: st.h, sL: st.sL, sR: st.sR, ir: st.ir.slice(), blk: st.blk.slice(), dist: st.dist, stalled: st.stalled, cubes: cubes.map((c) => ({ x: c.x, z: c.z, out: c.out })), linePath, arenaR }; },
    destroy() {
      destroyed = true; if (raf) cancelAnimationFrame(raf); if (ro) ro.disconnect(); controls.dispose();
      if (renderer) { renderer.dispose(); renderer.domElement.remove(); }
    },
  };
}
