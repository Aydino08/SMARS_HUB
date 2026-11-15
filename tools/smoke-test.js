#!/usr/bin/env node
'use strict';
/* Avtomatik sınaq: serveri, 2 virtual robotu və 1 "köhnə" (ID-siz) robotu başladır, UI kimi qoşulur və yoxlayır. */
const { spawn } = require('child_process');
const http = require('http');
const path = require('path');
const fs = require('fs');
const os = require('os');
const WebSocket = require('ws');

const PORT = 3900 + Math.floor(Math.random() * 90);
const KEY = 'test-robot-key-123';
const PW = 'parol123';
const DATA = fs.mkdtempSync(path.join(process.env.TMPDIR || os.tmpdir(), 'smarshub-'));
const root = path.join(__dirname, '..');
const procs = [];
let failed = 0;
const ok = (c, msg) => { console.log((c ? '  ✓ ' : '  ✗ ') + msg); if (!c) failed++; };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function get(p) {
  return new Promise((res) => {
    http.get({ host: '127.0.0.1', port: PORT, path: p }, (r) => {
      let b = ''; r.on('data', (d) => { b += d; }); r.on('end', () => res({ s: r.statusCode, b, h: r.headers }));
    }).on('error', () => res({ s: 0, b: '' }));
  });
}

function uiClient() {
  const ws = new WebSocket(`ws://127.0.0.1:${PORT}/ws/ui`);
  const st = { ws, list: [], msgs: [], pend: new Map(), id: 1 };
  ws.on('message', (d) => {
    const m = JSON.parse(d.toString());
    st.msgs.push(m);
    if (m.t === 'robots') st.list = m.list;
    if (m.t === 'res' && st.pend.has(m.id)) { st.pend.get(m.id)(m); st.pend.delete(m.id); }
  });
  st.open = new Promise((r) => ws.on('open', r));
  st.req = (r, p, q) => new Promise((res) => {
    const id = st.id++; st.pend.set(id, res);
    ws.send(JSON.stringify({ t: 'req', id, r, p, q }));
    setTimeout(() => { if (st.pend.has(id)) { st.pend.delete(id); res({ c: -1 }); } }, 6000);
  });
  st.wait = async (pred, ms = 4000) => {
    const t0 = Date.now();
    while (Date.now() - t0 < ms) { const m = st.msgs.find(pred); if (m) return m; await sleep(50); }
    return null;
  };
  return st;
}

(async () => {
  const env = { ...process.env, PORT: String(PORT), ROBOT_KEY: KEY, UI_PASSWORD: PW, DATA_DIR: DATA };
  const srv = spawn(process.execPath, ['server.js'], { cwd: root, env, stdio: ['ignore', 'pipe', 'pipe'] });
  procs.push(srv);
  srv.stdout.on('data', (d) => process.env.VERBOSE && process.stdout.write('[srv] ' + d));
  srv.stderr.on('data', (d) => process.stdout.write('[srv!] ' + d));
  await sleep(700);

  console.log('HTTP');
  const h = await get('/healthz'); ok(h.s === 200 && JSON.parse(h.b).configured === true, '/healthz configured');
  for (const p of ['/', '/robots', '/robot/smars-sim-1', '/about', '/guide', '/nope']) {
    const r = await get(p); ok(r.s === 200 && r.b.includes('id="app"'), `${p} → index.html`);
  }
  const js = await get('/app.js'); ok(js.s === 200 && /javascript/.test(js.h['content-type']), '/app.js');
  const css = await get('/app.css'); ok(css.s === 200 && /css/.test(css.h['content-type']), '/app.css');
  const trav = await get('/..%2fserver.js'); ok(trav.s !== 200, `path traversal bloklanır (${trav.s})`);
  ok(/script-src 'self'/.test(js.h['content-security-policy'] || ''), 'CSP başlığı');

  console.log('Robotlar');
  const sim = spawn(process.execPath, ['tools/sim-robot.js', '--url', `ws://127.0.0.1:${PORT}`, '--key', KEY, '--count', '2'], { cwd: root, stdio: 'ignore' });
  procs.push(sim);
  // ID-siz köhnə firmware
  const legacy = new WebSocket(`ws://127.0.0.1:${PORT}/ws/robot`, { headers: { Authorization: 'Bearer ' + KEY } });
  legacy.on('message', (d) => {
    const m = /^R\|(\d+)\|([^?]*)/.exec(d.toString());
    if (m) legacy.send(JSON.stringify({ t: 'res', id: Number(m[1]), c: 200, d: m[2] === '/status' ? { mode: 0, dist: 42, rssi: -60 } : 'ok' }));
  });
  await new Promise((r) => legacy.on('open', r));
  legacy.send(JSON.stringify({ t: 'hello', fw: 'legacy-1' }));
  const bad = new WebSocket(`ws://127.0.0.1:${PORT}/ws/robot`, { headers: { Authorization: 'Bearer wrong-key-000000' } });
  const badRes = await new Promise((r) => { bad.on('unexpected-response', (_, res) => r(res.statusCode)); bad.on('open', () => r(200)); bad.on('error', () => r(-1)); });
  ok(badRes === 401, 'yanlış robot açarı → 401');
  await sleep(1200);

  console.log('UI');
  const w = uiClient(); await w.open;
  w.ws.send(JSON.stringify({ t: 'auth', pw: 'wrong' }));
  ok((await w.wait((m) => m.t === 'auth')).ok === false, 'yanlış şifrə rədd edilir');
  const u = uiClient(); await u.open;
  u.ws.send(JSON.stringify({ t: 'auth', pw: PW }));
  ok((await u.wait((m) => m.t === 'auth')).ok === true, 'düzgün şifrə ilə giriş');
  await u.wait((m) => m.t === 'robots');
  await sleep(300);
  const ids = u.list.map((r) => r.id).sort();
  ok(JSON.stringify(ids) === JSON.stringify(['smars-1', 'smars-sim-1', 'smars-sim-2']), 'siyahıda 3 robot: ' + ids.join(', '));
  ok(u.list.every((r) => r.on), 'hamısı onlayn');
  ok((u.list.find((r) => r.id === 'smars-1') || {}).fw === 'legacy-1', 'köhnə firmware smars-1 kimi qeydə alındı');

  const s = await u.req('smars-sim-1', '/status');
  ok(s.c === 200 && typeof s.d.dist === 'number', '/status smars-sim-1');
  const s2 = await u.req('smars-sim-2', '/settings');
  ok(s2.c === 200 && Array.isArray(s2.d.s), '/settings smars-sim-2');
  ok((await u.req('smars-sim-1', '/mode', { m: '2' })).c === 200, 'line following-ə keçid (icazəli)');
  ok((await u.req('smars-sim-1', '/tank', { dir: 'F' })).c === 409, 'avto rejimdə sürmə robotda rədd edilir');
  ok((await u.req('smars-sim-1', '/wifi')).c === 404, 'icazəsiz yol → 404');
  ok((await u.req('smars-sim-1', '/tank', { dir: 'F;rm' })).c === 400, 'pis query → 400');
  ok((await u.req('yoxdur', '/status')).c === 503, 'olmayan robot → 503');

  console.log('Fərdiləşdirmə');
  u.ws.send(JSON.stringify({ t: 'profile', r: 'smars-sim-1', data: { name: 'Kürək <b>', color: '#ff2244', modes: { line: false }, modules: { servo: false, ir: false }, hiddenGroups: ['line'] } }));
  ok(!!(await u.wait((m) => m.t === 'notice' && m.level === 'ok')), 'profil saxlanıldı');
  await sleep(400);
  const r1 = u.list.find((r) => r.id === 'smars-sim-1');
  ok(r1.profile.modes.line === false && r1.profile.modules.servo === false, 'profil siyahıda yeniləndi');
  ok(r1.name === 'Kürək b', 'ad təmizləndi: ' + r1.name);
  const st2 = await u.req('smars-sim-1', '/status');
  ok(st2.d.mode === 0, 'söndürülmüş rejimdə olan robot avtomatik əl idarəsinə keçirildi');
  ok((await u.req('smars-sim-1', '/mode', { m: '2' })).c === 403, 'söndürülmüş line following → 403');
  ok((await u.req('smars-sim-1', '/servo', { grip: '90' })).c === 403, 'söndürülmüş servo → 403');
  ok((await u.req('smars-sim-2', '/mode', { m: '2' })).c === 200, 'digər robotda line following işləyir');
  await sleep(700);
  const saved = JSON.parse(fs.readFileSync(path.join(DATA, 'profiles.json'), 'utf8'));
  ok(saved['smars-sim-1'] && saved['smars-sim-1'].modes.line === false, 'profiles.json diskə yazıldı');

  console.log('Oflayn / silmə');
  legacy.close();
  await sleep(500);
  const lg = u.list.find((r) => r.id === 'smars-1');
  ok(lg && lg.on === false, 'qopan robot oflayn kimi siyahıda qalır');
  u.ws.send(JSON.stringify({ t: 'forget', r: 'smars-1' }));
  await sleep(400);
  ok(!u.list.find((r) => r.id === 'smars-1'), 'oflayn robot silindi');
  u.ws.send(JSON.stringify({ t: 'forget', r: 'smars-sim-2' }));
  ok(!!(await u.wait((m) => m.t === 'notice' && m.level === 'warn')), 'onlayn robotu silmək olmur');
  const pub = JSON.parse((await get('/api/public')).b);
  ok(pub.online === 2, '/api/public online=' + pub.online);

  procs.forEach((p) => p.kill());
  try { w.ws.close(); u.ws.close(); } catch (_) { /* */ }
  console.log(failed ? `\n${failed} yoxlama UĞURSUZ` : '\nBütün yoxlamalar keçdi ✓');
  process.exit(failed ? 1 : 0);
})().catch((e) => { console.error(e); procs.forEach((p) => p.kill()); process.exit(1); });
