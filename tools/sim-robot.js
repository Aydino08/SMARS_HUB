#!/usr/bin/env node
'use strict';
/*
 * Virtual SMARS robot(lar) — serveri real robot olmadan sınamaq üçün.
 * ESP32 firmware-in protokolunu təqlid edir:  R|gid|/path?qs  →  {"t":"res","id":gid,"c":200,"d":...}
 *
 *   ROBOT_KEY=test-robot-key-123 node tools/sim-robot.js --count 3
 *   node tools/sim-robot.js --url wss://xxx.up.railway.app --key <ROBOT_KEY> --id smars-07 --name "Mavi"
 */
const WebSocket = require('ws');

const args = process.argv.slice(2);
const arg = (n, d) => { const i = args.indexOf('--' + n); return i >= 0 && args[i + 1] ? args[i + 1] : d; };
const URL_BASE = arg('url', process.env.HUB_URL || 'ws://localhost:3000').replace(/\/$/, '');
const KEY = arg('key', process.env.ROBOT_KEY || '');
const COUNT = parseInt(arg('count', '1'), 10);
const BASE_ID = arg('id', 'smars-sim');
const BASE_NAME = arg('name', 'Virtual SMARS');
if (KEY.length < 12) { console.error('ROBOT_KEY (və ya --key) min. 12 simvol olmalıdır'); process.exit(1); }

const SETTINGS_DEF = [
  { k: 'tankSpd', l: 'Standart sürət', t: 'i', d: 180, mn: 60, mx: 255, st: 1, g: 'drive' },
  { k: 'turnK', l: 'Dönmə əmsalı', t: 'f', d: 0.8, mn: 0.2, mx: 1, st: 0.05, g: 'drive' },
  { k: 'obsDist', l: 'Maneə məsafəsi (sm)', t: 'i', d: 25, mn: 5, mx: 80, st: 1, g: 'obs' },
  { k: 'obsBack', l: 'Geri çəkilmə (ms)', t: 'i', d: 400, mn: 100, mx: 1500, st: 50, g: 'obs' },
  { k: 'lineSpd', l: 'Xətt sürəti', t: 'i', d: 140, mn: 60, mx: 255, st: 1, g: 'line' },
  { k: 'irThr', l: 'IR qara həddi (%)', t: 'i', d: 50, mn: 5, mx: 95, st: 1, g: 'line' },
  { k: 'lineInv', l: 'Ağ xətt (invert)', t: 'b', d: 0, mn: 0, mx: 1, st: 1, g: 'line' },
  { k: 'grpLo', l: 'Gripper min', t: 'i', d: 20, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'grpHi', l: 'Gripper max', t: 'i', d: 160, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'elbLo', l: 'Elbow min', t: 'i', d: 10, mn: 0, mx: 180, st: 1, g: 'servo' },
  { k: 'elbHi', l: 'Elbow max', t: 'i', d: 170, mn: 0, mx: 180, st: 1, g: 'servo' },
];
const GROUPS = [
  { i: 'drive', t: 'Sürüş' }, { i: 'obs', t: 'Maneədən qaçma' }, { i: 'line', t: 'Line following' }, { i: 'servo', t: 'Servo limitləri' },
];
const STATUS_TXT = ['', 'İRƏLİ — YOL AÇIQ', 'XƏTT İZLƏNİR', 'KUB AXTARILIR', 'STANSİYAYA GİRİR'];

function startRobot(n) {
  const id = COUNT > 1 ? `${BASE_ID}-${n + 1}` : BASE_ID;
  const name = COUNT > 1 ? `${BASE_NAME} ${n + 1}` : BASE_NAME;
  const s = {
    mode: 0, tankDir: 'STOP', grip: 90, elbow: 90, lastDrive: 0, t: Math.random() * 10,
    bat: 40 + Math.random() * 50, dockT: 0,
    set: Object.fromEntries(SETTINGS_DEF.map((x) => [x.k, x.d])),
  };
  let ws, tick;

  function status() {
    s.t += 0.15;
    const dist = Math.round(60 + 50 * Math.sin(s.t / 3) + 10 * Math.sin(s.t * 1.7));
    const ir = [0, 1, 2].map((i) => Math.max(0, Math.min(100, 50 + 45 * Math.sin(s.t + i * 2.1))));
    return {
      mode: s.mode, status: s.mode ? (s.mode === 1 && dist < s.set.obsDist ? 'MANEƏ! DÖNÜR' : STATUS_TXT[s.mode]) : '',
      dist, rssi: -50 - Math.round(10 * Math.random()), sta: '192.168.1.' + (40 + n),
      tankDir: s.tankDir, tankSpd: s.set.tankSpd,
      ir: ir.map((x) => Math.round(x * 10) / 10), blk: ir.map((x) => (x > s.set.irThr ? 1 : 0)),
      gripAngle: s.grip, elbowAngle: s.elbow, sv: 1,
      ...battery(),
    };
  }
  // saxta batareya: rejim 4-də ~4 san sonra "dok olur" və şarj olunur, qalan vaxt yavaş boşalır
  function battery() {
    const docked = s.mode === 4 && Date.now() - s.dockT > 4000;
    s.bat = Math.max(0, Math.min(100, s.bat + (docked ? 0.6 : -0.02)));
    const v = 6.4 + (2.0 * s.bat) / 100;
    return {
      bat: Math.round(s.bat), vbat: Math.round(v * 100) / 100, chg: docked ? 1 : 0,
      dock: s.mode === 4 ? (docked ? (s.bat >= 99.5 ? 'DOLDU' : 'ŞARJ OLUNUR') : 'STANSİYAYA GİRİR') : '',
    };
  }

  function handle(path, q) {
    switch (path) {
      case '/status': return [200, status()];
      case '/settings': return [200, { g: GROUPS, s: SETTINGS_DEF.map((x) => ({ ...x, v: s.set[x.k] })) }];
      case '/mode': {
        const m = parseInt(q.get('m'), 10);
        if (!(m >= 0 && m <= 4)) return [400, 'bad mode'];
        s.mode = m; s.tankDir = m ? 'AUTO' : 'STOP';
        if (m === 4) s.dockT = Date.now();
        return [200, String(m)];
      }
      case '/tank': {
        if (q.has('spd')) s.set.tankSpd = Math.max(60, Math.min(255, parseInt(q.get('spd'), 10) || 180));
        if (q.has('dir')) {
          if (s.mode !== 0) return [409, 'auto mode'];
          s.tankDir = { F: 'İRƏLİ', B: 'GERİ', L: 'SOL', R: 'SAĞ', S: 'STOP' }[q.get('dir')] || 'STOP';
          s.lastDrive = Date.now();
        }
        return [200, s.tankDir];
      }
      case '/servo':
        if (q.has('grip')) s.grip = Math.max(s.set.grpLo, Math.min(s.set.grpHi, parseInt(q.get('grip'), 10)));
        if (q.has('elbow')) s.elbow = Math.max(s.set.elbLo, Math.min(s.set.elbHi, parseInt(q.get('elbow'), 10)));
        return [200, 'ok'];
      case '/set': {
        const def = SETTINGS_DEF.find((x) => x.k === q.get('k'));
        if (!def) return [404, 'unknown key'];
        let v = parseFloat(q.get('v'));
        if (isNaN(v)) return [400, 'bad value'];
        v = Math.max(def.mn, Math.min(def.mx, v));
        if (def.t !== 'f') v = Math.round(v);
        s.set[def.k] = v;
        return [200, String(v)];
      }
      case '/reset': {
        const g = q.get('g');
        SETTINGS_DEF.forEach((x) => { if (g === 'all' || x.g === g) s.set[x.k] = x.d; });
        return [200, 'ok'];
      }
      default: return [404, 'unknown'];
    }
  }

  function connect() {
    const url = `${URL_BASE}/ws/robot?id=${encodeURIComponent(id)}&name=${encodeURIComponent(name)}`;
    ws = new WebSocket(url, { headers: { Authorization: 'Bearer ' + KEY } });
    ws.on('open', () => {
      console.log(`[${id}] qoşuldu`);
      ws.send(JSON.stringify({ t: 'hello', fw: 'sim-1.0' }));
      // ölü-adam dayanması: əl rejimində 700 ms əmr gəlməsə dayan
      tick = setInterval(() => { if (s.mode === 0 && s.tankDir !== 'STOP' && Date.now() - s.lastDrive > 700) s.tankDir = 'STOP'; }, 100);
    });
    ws.on('message', (data) => {
      const str = data.toString();
      const m = /^R\|(\d+)\|([^?]*)(?:\?(.*))?$/.exec(str);
      if (!m) return;
      const [c, d] = handle(m[2], new URLSearchParams(m[3] || ''));
      ws.send(JSON.stringify({ t: 'res', id: Number(m[1]), c, d }));
    });
    ws.on('close', (code) => {
      clearInterval(tick);
      console.log(`[${id}] bağlandı (${code}), 3 san sonra yenidən`);
      setTimeout(connect, 3000);
    });
    ws.on('error', (e) => console.log(`[${id}] xəta: ${e.message}`));
  }
  connect();
}

for (let i = 0; i < COUNT; i++) startRobot(i);
