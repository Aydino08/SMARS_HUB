// Dev-only: brauzer simulyatorunu (public/sim3d.js) Node-da headless işlədib davranışı yoxlayır.
//   SIMTEST_MODULES=<@napi-rs/canvas olan node_modules> node tools/sim-check.mjs
import { createRequire } from 'module';
import { readFileSync } from 'fs';
import path from 'path';
import { fileURLToPath } from 'url';

const here = path.dirname(fileURLToPath(import.meta.url));
const pub = path.join(here, '..', 'public');
const req = createRequire(path.join(process.env.SIMTEST_MODULES || '.', 'x.js'));
const { createCanvas } = req('@napi-rs/canvas');

globalThis.fetch = async (u) => {
  const buf = readFileSync(path.join(pub, u));
  return { json: async () => JSON.parse(buf.toString()), arrayBuffer: async () => buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.length) };
};
const mem = {};
globalThis.localStorage = { getItem: (k) => mem[k] ?? null, setItem: (k, v) => { mem[k] = String(v); }, removeItem: (k) => { delete mem[k]; } };

const { createSim } = await import(path.join(pub, 'sim3d.js'));
let failed = 0;
const ok = (c, m) => { console.log((c ? '  ✓ ' : '  ✗ ') + m); if (!c) failed++; };
const mk = (scene, config = 'A') => createSim(null, { headless: true, scene, config, createCanvas });
const hold = (sim, dir, sec) => { for (let t = 0; t < sec; t += 0.15) { sim.handle('/tank', { dir }); sim.step(0.15); } };
const W = 1500 - 40;

console.log('Əl idarəsi');
{
  const sim = await mk('obstacle');
  const s0 = sim.state();
  hold(sim, 'F', 1.0);
  const s1 = sim.state();
  const d = Math.hypot(s1.x - s0.x, s1.z - s0.z);
  const want = (180 / 255) * 300;
  ok(Math.abs(d - want) < 25, `F 1 san → ${d.toFixed(0)} mm (gözlənilən ~${want.toFixed(0)})`);
  ok(Math.abs(s1.h - s0.h) < 1e-6, 'düz gedir (istiqamət dəyişmir)');
  ok(s1.sL > 150 && Math.abs(s1.sL - s1.sR) < 1e-6, 'hər iki tırtıl eyni sürətlə fırlanır');
  sim.step(1.0);
  const s2 = sim.state();
  ok(Math.hypot(s2.x - s1.x, s2.z - s1.z) < 160 * 0.75 + 1, 'əmr kəsiləndə 0.7 san sonra dayanır (ölü-adam)');
  const s3a = sim.state(); sim.step(1); const s3b = sim.state();
  ok(Math.hypot(s3b.x - s3a.x, s3b.z - s3a.z) < 0.01, 'dayandıqdan sonra yerində qalır');
  hold(sim, 'L', 1.0);
  ok(sim.state().h > s2.h + 0.5, 'L → sola (saat əqrəbinin əksinə) dönür');
  ok(sim.handle('/tank', { spd: '999' })[1] && sim.handle('/status')[1].tankSpd === 255, 'sürət 255-ə məhdudlaşdırılır');
  sim.handle('/servo', { grip: '0', elbow: '180' });
  const st = sim.handle('/status')[1];
  ok(st.gripAngle === 20 && st.elbowAngle === 170, `servo limitləri tətbiq olunur (${st.gripAngle}, ${st.elbowAngle})`);
  ok(Array.isArray(st.ir) && st.ir.length === 3 && typeof st.dist === 'number', 'status: ir[3], dist');
  sim.destroy();
}

console.log('Divar / toqquşma');
{
  const sim = await mk('line');
  sim.place(0, 1000, Math.PI);                       // +Z divarına baxır; divarın daxili üzü z=1460
  const want = (1460 - 1000 - 81.97) / 10;
  const d = sim.handle('/status')[1].dist;
  ok(Math.abs(d - want) <= 1.5, `HC-SR04 divara məsafəni ölçür: ${d} sm (həqiqi ${want.toFixed(1)})`);
  hold(sim, 'F', 6);
  const s = sim.state();
  ok(s.z < 1460 - 82 && s.z > 1460 - 100, `divardan keçmir, ona dirənir (z=${s.z.toFixed(0)})`);
  ok(s.stalled > 300, 'dirənəndə "ilişib" vəziyyəti');
  ok(sim.handle('/status')[1].dist === 999, 'dirənəndə sensor kor zonadadır (<2 sm → 999, real HC-SR04 kimi)');
  const sim2 = await mk('obstacle');
  hold(sim2, 'R', 0.6); hold(sim2, 'F', 25);
  const s2 = sim2.state();
  ok(Math.abs(s2.x) < W && Math.abs(s2.z) < W, `maneə səhnəsində meydançadan çıxmır (x=${s2.x.toFixed(0)}, z=${s2.z.toFixed(0)})`);
  sim.destroy(); sim2.destroy();
}

console.log('Line following (60 san)');
{
  const sim = await mk('line');
  sim.handle('/mode', { m: '2' });
  const path0 = sim.state().linePath;
  let onLine = 0, n = 0, ang = 0, prev = null, maxOff = 0;
  for (let t = 0; t < 60; t += 0.1) {
    sim.step(0.1);
    const s = sim.state();
    n++; if (s.blk.some(Boolean)) onLine++;
    const a = Math.atan2(s.z, s.x);
    if (prev !== null) { let da = a - prev; if (da > Math.PI) da -= 2 * Math.PI; if (da < -Math.PI) da += 2 * Math.PI; ang += da; }
    prev = a;
    // mərkəzi sensorun xəttə ən yaxın məsafəsi
    const sx = s.x - 68.15 * Math.sin(s.h), sz = s.z - 68.15 * Math.cos(s.h);
    let best = Infinity; for (const [px, pz] of path0) best = Math.min(best, Math.hypot(px - sx, pz - sz));
    maxOff = Math.max(maxOff, best);
  }
  const laps = Math.abs(ang) / (2 * Math.PI);
  ok(onLine / n > 0.9, `sensorlar vaxtın ${(100 * onLine / n).toFixed(0)}%-ində xəttin üstündədir`);
  ok(maxOff < 40, `xətdən maks. kənarlaşma ${maxOff.toFixed(1)} mm`);
  ok(laps > 0.8, `60 san-də ${laps.toFixed(2)} dövrə`);
  sim.destroy();
}

console.log('Maneədən qaçma (90 san)');
{
  const sim = await mk('obstacle');
  sim.handle('/mode', { m: '1' });
  let stall = 0, travel = 0, n = 0;
  let p = sim.state();
  const cells = new Set();
  for (let t = 0; t < 90; t += 0.1) {
    sim.step(0.1); n++;
    const s = sim.state();
    travel += Math.hypot(s.x - p.x, s.z - p.z); p = s;
    if (s.stalled > 100) stall++;
    cells.add(Math.round(s.x / 300) + ',' + Math.round(s.z / 300));
  }
  ok(stall / n < 0.08, `ilişmə vaxtı ${(100 * stall / n).toFixed(1)}%`);
  ok(travel > 6000, `qət edilən yol ${(travel / 1000).toFixed(1)} m`);
  ok(cells.size >= 8, `meydançanın ${cells.size} müxtəlif hissəsini gəzdi`);
  sim.destroy();
}

console.log('Arena (180 san)');
{
  const sim = await mk('arena');
  sim.handle('/mode', { m: '3' });
  let maxR = 0;
  for (let t = 0; t < 180; t += 0.1) {
    sim.step(0.1);
    const s = sim.state();
    maxR = Math.max(maxR, Math.hypot(s.x, s.z));
  }
  const s = sim.state();
  const out = s.cubes.filter((c) => c.out).length;
  ok(out >= 4, `${out}/${s.cubes.length} kub arenadan çıxarıldı`);
  ok(maxR < s.arenaR + 45 + 20, `robotun mərkəzi arenadan çıxmadı (maks. r=${maxR.toFixed(0)}, kənar ${s.arenaR + 45})`);
  sim.destroy();
}

console.log('Şarj stansiyası');
{
  const sim = await mk('station');
  const dockLocal = () => {                       // robotun pozası dok çərçivəsində (dok = 0,0,0°)
    const s = sim.state(), dp = s.dockPose, c = Math.cos(dp.h), sn = Math.sin(dp.h), dx = s.x - dp.x, dz = s.z - dp.z;
    const lx = dx * c - dz * sn, ly = -(dx * sn + dz * c), dh = ((((s.h - dp.h) * 180) / Math.PI + 540) % 360) - 180;
    return { lx, ly, dh, tx: lx + 84 * Math.sin((dh * Math.PI) / 180) };   // tx: quyruğun ucunun yan xətası
  };
  const toW = (lx, ly) => { const dp = sim.state().dockPose, c = Math.cos(dp.h), sn = Math.sin(dp.h); return [dp.x + lx * c - ly * sn, dp.z - lx * sn - ly * c]; };
  ok(sim.handle('/status')[1].bat === 30 && sim.handle('/status')[1].chg === 0, 'səhnə: batareya 30%, şarj yoxdur');
  ok(sim.handle('/mode', { m: '4' })[0] === 200, '/mode?m=4 qəbul olunur');
  const seen = new Set();
  let tDock = -1;
  for (let t = 0; t < 20 && tDock < 0; t += 0.05) { sim.step(0.05); seen.add(sim.state().dock); if (sim.state().dock === 'docked') tDock = t; }
  ok(tDock > 0 && tDock < 12, `xətt → T → 180° → geri: ${tDock.toFixed(1)} san-də oturdu`);
  ok(['follow', 'stop', 'spin', 'reverse'].every((x) => seen.has(x)), 'vəziyyətlər: ' + [...seen].join(' → '));
  const L = dockLocal();
  ok(Math.abs(L.lx) < 1.3 && Math.abs(L.ly) < 1.3 && Math.abs(L.dh) < 3, `dok dəqiqliyi: yana ${L.lx.toFixed(2)} mm, ${L.dh.toFixed(2)}°`);
  const st1 = sim.handle('/status')[1];
  ok(st1.chg === 1 && st1.dock === 'ŞARJ OLUNUR' && st1.mode === 4, `status: chg=1, dock="${st1.dock}"`);
  const b0 = sim.state().soc; sim.step(10);
  ok(sim.state().soc > b0 + 0.05 && sim.state().det, `şarj: 10 san-də ${(b0 * 100).toFixed(0)}% → ${(sim.state().soc * 100).toFixed(0)}%`);
  sim.setBattery(99); sim.step(8);
  ok(sim.state().dock === 'charged' && sim.handle('/status')[1].dock === 'DOLDU', 'dolduqda → DOLDU');
  // huni: əl ilə geri sürmə — tutma zonası ±19 mm
  const back = (lx, yaw) => {
    sim.reset(); const [x, z] = toW(lx, 140); sim.place(x, z, sim.state().dockPose.h + (yaw * Math.PI) / 180);
    hold(sim, 'B', 3);
    return { ...dockLocal(), det: sim.state().det };
  };
  const c1 = back(15, 0), c2 = back(-8, 2), c3 = back(28, 0);
  ok(c1.det && Math.abs(c1.tx) < 1.3, `huni 15 mm yan xətanı düzəldir → quyruq ${c1.tx.toFixed(2)} mm, ${c1.dh.toFixed(2)}°, DET=${c1.det ? 1 : 0}`);
  ok(c2.det && Math.abs(c2.tx) < 1.3 && Math.abs(c2.dh) < 3, `huni 2° çəp girişi tutur → quyruq ${c2.tx.toFixed(2)} mm, ${c2.dh.toFixed(2)}°, DET=${c2.det ? 1 : 0}`);
  ok(!c3.det && c3.ly > 20, `28 mm kənarda: quyruq stansiyanın üzünə dirənir (DET=0, ly=${c3.ly.toFixed(0)})`);
  // avtomatik qayıdış: xətt rejimində (T-yə çatmamış) batareya 25%-dən aşağı düşür
  sim.reset(); sim.setBattery(25.4);
  sim.handle('/mode', { m: '2' });
  let sw = -1, dk = -1;
  for (let t = 0; t < 60 && dk < 0; t += 0.1) {
    sim.step(0.1);
    if (sw < 0 && sim.state().dock !== 'idle') sw = t;
    if (sim.state().dock === 'docked') dk = t;
  }
  ok(sw > 0 && dk > sw, `avtomatik qayıdış: ${sw.toFixed(1)} san-də rejim 4, ${dk.toFixed(1)} san-də oturdu`);
  // boş batareya
  sim.reset(); sim.handle('/mode', { m: '1' }); sim.setBattery(0.5); sim.step(1);
  const p0 = sim.state(); sim.step(1); const p1 = sim.state();
  ok(Math.hypot(p1.x - p0.x, p1.z - p0.z) < 0.01 && sim.handle('/status')[1].status === 'BATAREYA BOŞDUR', 'batareya boşdur → robot dayanır');
  sim.destroy();
  const sim2 = await mk('line');
  ok(sim2.handle('/mode', { m: '4' })[0] === 409, 'stansiyasız səhnədə "Şarja qayıt" → 409');
  const s0 = sim2.state().soc; sim2.handle('/mode', { m: '2' }); sim2.step(20);
  ok(s0 - sim2.state().soc < 0.01, 'stansiyasız səhnədə batareya real tempdə boşalır');
  sim2.destroy();
}

console.log('Konfiqurasiya B (IR-sız) + API');
{
  const sim = await mk('arena', 'B');
  ok(sim.handle('/mode', { m: '2' })[0] === 409, 'IR-sız konfiqurasiyada line following → 409');
  ok(sim.handle('/status')[1].ir === undefined, 'status-da ir yoxdur');
  const set = sim.handle('/settings')[1];
  ok(set.g.length === 7 && set.s.some((x) => x.k === 'obsDist') && set.g.some((g) => g.i === 'bat'), 'settings: 7 qrup (batareya daxil)');
  ok(sim.handle('/mode', { m: '4' })[0] === 409, 'IR-sız konfiqurasiyada "Şarja qayıt" → 409');
  ok(sim.handle('/set', { k: 'obsDist', v: '500' })[1] === '80', '/set maks. qiymətə məhdudlaşdırır');
  sim.handle('/reset', { g: 'obs' });
  ok(sim.handle('/settings')[1].s.find((x) => x.k === 'obsDist').v === 22, '/reset qrupu default-a qaytarır');
  ok(sim.handle('/wifi')[0] === 404, 'naməlum yol → 404');
  hold(sim, 'F', 0.5);
  sim.handle('/mode', { m: '1' });
  ok(sim.handle('/tank', { dir: 'F' })[0] === 409, 'avto rejimdə əl əmri → 409');
  sim.destroy();
}

console.log(failed ? `\n${failed} yoxlama UĞURSUZ` : '\nBütün simulyasiya yoxlamaları keçdi ✓');
process.exit(failed ? 1 : 0);
