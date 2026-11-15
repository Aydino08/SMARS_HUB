'use strict';
/*
 * ============================================================
 *  SMARS Control Hub — Railway server
 *
 *     Brauzer (sayt)  <--WSS-->  BU SERVER  <--WSS-->  SMARS robotlar (ESP32)
 *
 *  - Hər robot özü serverə qoşulur (çıxan bağlantı) — router/port lazım deyil.
 *  - Bir neçə robot eyni anda qoşula bilər; hər birinin öz ID-si var.
 *  - Saytda robotların canlı siyahısı, hər robot üçün idarə paneli və
 *    fərdiləşdirmə (hansı modlar / modullar görünsün) var.
 *
 *  Mühit dəyişənləri (Railway → Variables):
 *    ROBOT_KEY    robotların gizli açarı (min. 12 simvol) — ESP32 kodundakı ilə eyni
 *    UI_PASSWORD  sayta giriş şifrəsi (min. 6 simvol)
 *    DATA_DIR     (istəyə bağlı) profil faylının qovluğu. Railway Volume-u bura bağla, məs. /data
 *    PORT         Railway özü verir
 * ============================================================
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');
const { WebSocketServer } = require('ws');

// ---------------------------------------------------------------- config
const PORT = parseInt(process.env.PORT, 10) || 3000;
const ROBOT_KEY = (process.env.ROBOT_KEY || '').trim();
const UI_PASSWORD = (process.env.UI_PASSWORD || '').trim();
const CONFIGURED = ROBOT_KEY.length >= 12 && UI_PASSWORD.length >= 6;
const DATA_DIR = process.env.DATA_DIR || path.join(__dirname, 'data');
const PROFILE_FILE = path.join(DATA_DIR, 'profiles.json');
const PUBLIC_DIR = path.join(__dirname, 'public');

// UI-ın robota göndərə biləcəyi yollar (robot tərəfdə də ayrıca yoxlanılır)
const ROUTES = new Set(['/tank', '/mode', '/servo', '/status', '/settings', '/set', '/reset']);

// Robotun sürüş modları (firmware-dəki nömrələrlə eyni). 0 = əl idarəsi, həmişə açıqdır.
const MODES = [
  { id: 0, key: 'manual', name: 'Əl idarəsi' },
  { id: 1, key: 'obstacle', name: 'Maneədən qaçma' },
  { id: 2, key: 'line', name: 'Line following' },
  { id: 3, key: 'arena', name: 'Arena (kub çıxar)' },
];
const MODULES = ['ir', 'distance', 'servo', 'settings'];

const REQ_TIMEOUT_MS = 5000;
const MAX_PENDING = 256;
const MAX_PENDING_PER_ROBOT = 48;
const MAX_UI_CLIENTS = 48;
const MAX_ROBOTS = 64;
const MAX_PROFILES = 200;
const HEARTBEAT_MS = 15000;
const POLL_MS = 4000;              // server özü hər robotdan /status soruşur (siyahı üçün)
const LIST_THROTTLE_MS = 1500;
const AUTH_WINDOW_MS = 10 * 60 * 1000;
const AUTH_MAX_FAILS = 5;
const RATE_PER_SEC = 40;
const RATE_BURST = 60;
const ID_RE = /^[A-Za-z0-9_-]{1,32}$/;
const LEGACY_ID = 'smars-1';       // ID göndərməyən köhnə firmware üçün

if (!CONFIGURED) {
  console.error('!!! ROBOT_KEY (min 12) və UI_PASSWORD (min 6) dəyişənləri təyin olunmayıb.');
  console.error('!!! Railway → servis → Variables bölməsində əlavə et. Sayt açılır, amma giriş bağlıdır.');
}

// ---------------------------------------------------------------- helpers
const sha = (s) => crypto.createHash('sha256').update(String(s)).digest();
const safeEqual = (a, b) => crypto.timingSafeEqual(sha(a), sha(b));
const cleanText = (s, max) => String(s == null ? '' : s).replace(/[\u0000-\u001f\u007f<>]/g, '').trim().slice(0, max);

function clientIp(req) {
  const xf = req.headers['x-forwarded-for'];
  if (xf) {
    const parts = String(xf).split(',').map((s) => s.trim()).filter(Boolean);
    if (parts.length) return parts[parts.length - 1];
  }
  return req.socket.remoteAddress || '?';
}

function makeLimiter() {
  const m = new Map();
  return {
    wait(ip) {
      const f = m.get(ip);
      if (!f) return 0;
      const age = Date.now() - f.first;
      if (age > AUTH_WINDOW_MS) { m.delete(ip); return 0; }
      return f.n >= AUTH_MAX_FAILS ? Math.ceil((AUTH_WINDOW_MS - age) / 1000) : 0;
    },
    fail(ip) {
      const f = m.get(ip);
      if (!f || Date.now() - f.first > AUTH_WINDOW_MS) m.set(ip, { n: 1, first: Date.now() });
      else f.n++;
    },
    ok(ip) { m.delete(ip); },
    sweep() {
      const now = Date.now();
      for (const [ip, f] of m) if (now - f.first > AUTH_WINDOW_MS) m.delete(ip);
    },
  };
}
const uiLimiter = makeLimiter();
const robotLimiter = makeLimiter();
setInterval(() => { uiLimiter.sweep(); robotLimiter.sweep(); }, 60000).unref();

function sendJ(ws, obj) {
  if (!ws || ws.readyState !== 1) return;
  try { ws.send(JSON.stringify(obj)); } catch (_) { /* bağlantı bağlanıb */ }
}

// ---------------------------------------------------------------- profiles (fərdiləşdirmə)
function defaultProfile() {
  return {
    name: '',
    color: '#00e5ff',
    modes: { obstacle: true, line: true, arena: true },
    modules: { ir: true, distance: true, servo: true, settings: true },
    hiddenGroups: [],
    updated: 0,
  };
}

function normalizeProfile(input, base) {
  const p = base ? JSON.parse(JSON.stringify(base)) : defaultProfile();
  if (!input || typeof input !== 'object') return p;
  if (input.name !== undefined) p.name = cleanText(input.name, 40);
  if (typeof input.color === 'string' && /^#[0-9a-fA-F]{6}$/.test(input.color)) p.color = input.color.toLowerCase();
  if (input.modes && typeof input.modes === 'object') {
    for (const m of MODES) if (m.id !== 0 && typeof input.modes[m.key] === 'boolean') p.modes[m.key] = input.modes[m.key];
  }
  if (input.modules && typeof input.modules === 'object') {
    for (const k of MODULES) if (typeof input.modules[k] === 'boolean') p.modules[k] = input.modules[k];
  }
  if (Array.isArray(input.hiddenGroups)) {
    p.hiddenGroups = [...new Set(input.hiddenGroups.map(String).filter((g) => /^[A-Za-z0-9_]{1,16}$/.test(g)))].slice(0, 32);
  }
  return p;
}

let profiles = {};
try {
  const raw = JSON.parse(fs.readFileSync(PROFILE_FILE, 'utf8'));
  for (const [id, pr] of Object.entries(raw || {})) {
    if (ID_RE.test(id)) {
      profiles[id] = { ...normalizeProfile(pr), updated: Number(pr.updated) || 0, lastSeen: Number(pr.lastSeen) || null };
    }
  }
  console.log(`Profillər yükləndi: ${Object.keys(profiles).length} (${PROFILE_FILE})`);
} catch (e) {
  if (e.code !== 'ENOENT') console.error('profiles.json oxunmadı:', e.message);
}

let saveT = null;
function saveProfiles() {
  clearTimeout(saveT);
  saveT = setTimeout(() => {
    try {
      fs.mkdirSync(DATA_DIR, { recursive: true });
      const tmp = PROFILE_FILE + '.tmp';
      fs.writeFileSync(tmp, JSON.stringify(profiles, null, 2));
      fs.renameSync(tmp, PROFILE_FILE);
    } catch (e) {
      console.error('profiles.json yazılmadı:', e.message);
    }
  }, 400);
}

const profileOf = (id) => profiles[id] || defaultProfile();
const modeAllowed = (prof, m) => {
  const def = MODES.find((x) => x.id === m);
  if (!def) return false;
  return def.id === 0 || prof.modes[def.key] !== false;
};

// ---------------------------------------------------------------- HTTP
const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.webp': 'image/webp',
  '.ico': 'image/x-icon',
  '.json': 'application/json; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
};

const SEC_HEADERS = {
  'X-Content-Type-Options': 'nosniff',
  'X-Frame-Options': 'DENY',
  'Referrer-Policy': 'no-referrer',
  'Strict-Transport-Security': 'max-age=31536000',
  'Permissions-Policy': 'camera=(), microphone=(), geolocation=()',
  'Content-Security-Policy':
    "default-src 'self'; script-src 'self'; " +
    "style-src 'self' 'unsafe-inline' https://fonts.googleapis.com; " +
    "font-src 'self' https://fonts.gstatic.com; connect-src 'self' ws: wss:; " +
    "img-src 'self' data:; base-uri 'none'; form-action 'none'; frame-ancestors 'none'",
};

function sendFile(res, file, method, status = 200) {
  fs.readFile(file, (err, buf) => {
    if (err) {
      res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8', ...SEC_HEADERS });
      return res.end('Not found');
    }
    const ext = path.extname(file).toLowerCase();
    res.writeHead(status, {
      'Content-Type': MIME[ext] || 'application/octet-stream',
      'Cache-Control': ext === '.html' ? 'no-store' : 'no-cache',
      ...SEC_HEADERS,
    });
    res.end(method === 'HEAD' ? undefined : buf);
  });
}

function sendJson(res, obj, status = 200) {
  res.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store', ...SEC_HEADERS });
  res.end(JSON.stringify(obj));
}

const server = http.createServer((req, res) => {
  let url;
  try { url = decodeURIComponent((req.url || '/').split('?')[0]); } catch (_) { url = '/'; }

  if (req.method !== 'GET' && req.method !== 'HEAD') {
    res.writeHead(405, { Allow: 'GET, HEAD', ...SEC_HEADERS });
    return res.end();
  }

  if (url === '/healthz') return sendJson(res, { ok: true, configured: CONFIGURED });
  if (url === '/api/public') {
    return sendJson(res, { configured: CONFIGURED, online: robots.size, known: new Set([...robots.keys(), ...Object.keys(profiles)]).size });
  }

  // statik fayllar (uzantılı yollar)
  if (path.extname(url)) {
    const file = path.normalize(path.join(PUBLIC_DIR, url));
    if (!file.startsWith(PUBLIC_DIR + path.sep)) {
      res.writeHead(403, SEC_HEADERS);
      return res.end();
    }
    return sendFile(res, file, req.method);
  }

  // SPA səhifələri: /, /robots, /robot/<id>, /about, /guide ... hamısı index.html
  if (url.startsWith('/ws/') || url.startsWith('/api/')) {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8', ...SEC_HEADERS });
    return res.end('Not found');
  }
  return sendFile(res, path.join(PUBLIC_DIR, 'index.html'), req.method);
});

// ---------------------------------------------------------------- WebSocket state
const wssUI = new WebSocketServer({ noServer: true, maxPayload: 8192 });
const wssRobot = new WebSocketServer({ noServer: true, maxPayload: 64 * 1024 });

const robots = new Map();         // id -> robot
const uiClients = new Set();      // { ws, ip, authed, tokens, last, watch }
const pending = new Map();        // gid -> { c, id, robot, p, q, timer, t0 }
let gidCounter = 0;

function rejectUpgrade(socket, code, text) {
  try {
    socket.write(`HTTP/1.1 ${code} ${text}\r\nConnection: close\r\nContent-Length: 0\r\n\r\n`);
  } catch (_) { /* ignore */ }
  socket.destroy();
}

server.on('upgrade', (req, socket, head) => {
  socket.on('error', () => {});
  const [rawPath, rawQs] = (req.url || '').split('?');
  const ip = clientIp(req);

  if (rawPath === '/ws/robot') {
    if (!CONFIGURED) return rejectUpgrade(socket, 503, 'Service Unavailable');
    if (robotLimiter.wait(ip)) return rejectUpgrade(socket, 429, 'Too Many Requests');
    const m = /^Bearer\s+(.+)$/i.exec(req.headers['authorization'] || '');
    if (!m || !safeEqual(m[1].trim(), ROBOT_KEY)) {
      robotLimiter.fail(ip);
      console.log(`[robot] RED: yanlış açar (${ip})`);
      return rejectUpgrade(socket, 401, 'Unauthorized');
    }
    robotLimiter.ok(ip);
    const qs = new URLSearchParams(rawQs || '');
    return wssRobot.handleUpgrade(req, socket, head, (ws) => onRobot(ws, ip, qs.get('id'), qs.get('name')));
  }

  if (rawPath === '/ws/ui') {
    if (uiClients.size >= MAX_UI_CLIENTS) return rejectUpgrade(socket, 503, 'Service Unavailable');
    return wssUI.handleUpgrade(req, socket, head, (ws) => onUI(ws, ip));
  }

  return rejectUpgrade(socket, 404, 'Not Found');
});

// ---------------------------------------------------------------- robot list
function viewersOf(id) {
  let n = 0;
  for (const c of uiClients) if (c.authed && c.watch === id) n++;
  return n;
}

function robotList() {
  const ids = new Set([...robots.keys(), ...Object.keys(profiles)]);
  const out = [];
  for (const id of ids) {
    const r = robots.get(id);
    const prof = profileOf(id);
    out.push({
      id,
      on: !!r,
      name: prof.name || (r && r.name) || id,
      fw: r ? r.fw : null,
      since: r ? r.since : null,
      lastSeen: r ? Date.now() : (prof.lastSeen || null),
      rtt: r ? r.rtt : null,
      last: r ? r.last : null,
      viewers: viewersOf(id),
      profile: prof,
    });
  }
  out.sort((a, b) => (b.on - a.on) || a.name.localeCompare(b.name));
  return out;
}

let listT = null;
let listLast = 0;
function broadcastList(now) {
  const run = () => {
    listT = null;
    listLast = Date.now();
    const msg = { t: 'robots', list: robotList(), modes: MODES };
    for (const c of uiClients) if (c.authed) sendJ(c.ws, msg);
  };
  if (now) { clearTimeout(listT); return run(); }
  if (listT) return;
  listT = setTimeout(run, Math.max(0, LIST_THROTTLE_MS - (Date.now() - listLast)));
}

// ---------------------------------------------------------------- robot side
function failPending(code, text, filter) {
  for (const [gid, p] of pending) {
    if (!filter(p)) continue;
    clearTimeout(p.timer);
    pending.delete(gid);
    if (p.c && code) sendJ(p.c.ws, { t: 'res', id: p.id, r: p.robot.id, c: code, d: text });
  }
}

function sendToRobot(robot, p, qs, c, id) {
  let n = 0;
  for (const x of pending.values()) if (x.robot === robot) n++;
  if (pending.size >= MAX_PENDING || n >= MAX_PENDING_PER_ROBOT) return false;
  gidCounter = (gidCounter % 2000000000) + 1;
  const gid = gidCounter;
  const timer = setTimeout(() => {
    const x = pending.get(gid);
    if (!x) return;
    pending.delete(gid);
    if (x.c) sendJ(x.c.ws, { t: 'res', id: x.id, r: robot.id, c: 504, d: 'timeout' });
    if (!x.c && x.p === '/status') robot.pollBusy = false;
  }, REQ_TIMEOUT_MS);
  pending.set(gid, { c, id, robot, p, qs, timer, t0: Date.now() });
  try {
    robot.ws.send(`R|${gid}|${p}${qs ? '?' + qs : ''}`);
  } catch (_) {
    clearTimeout(timer);
    pending.delete(gid);
    return false;
  }
  return true;
}

function parseBody(d) {
  if (d && typeof d === 'object') return d;
  if (typeof d !== 'string') return null;
  try { return JSON.parse(d); } catch (_) { return null; }
}

function absorbStatus(robot, d) {
  const s = parseBody(d);
  if (!s) return;
  const modeChanged = Number.isInteger(s.mode) && (!robot.last || robot.last.mode !== s.mode);
  robot.last = {
    mode: Number.isInteger(s.mode) ? s.mode : (robot.last ? robot.last.mode : null),
    status: typeof s.status === 'string' ? s.status.slice(0, 40) : null,
    dist: typeof s.dist === 'number' ? s.dist : null,
    rssi: typeof s.rssi === 'number' ? s.rssi : null,
    tankDir: typeof s.tankDir === 'string' ? s.tankDir.slice(0, 12) : null,
    at: Date.now(),
  };
  broadcastList(modeChanged);
}

function registerRobot(me, id) {
  const old = robots.get(id);
  if (old && old !== me) {
    try { old.ws.close(4000, 'replaced'); } catch (_) { /* ignore */ }
    failPending(503, 'robot reconnected', (p) => p.robot === old);
    old.replaced = true;
  }
  if (me.id && me.id !== id) {
    if (robots.get(me.id) === me) robots.delete(me.id);
    // müvəqqəti (avtomatik yaradılmış, toxunulmamış) profili sil
    const tmp = profiles[me.id];
    if (tmp && !tmp.updated && !robots.has(me.id)) { delete profiles[me.id]; saveProfiles(); }
  }
  me.id = id;
  robots.set(id, me);
  // robotu yadda saxla ki, oflayn olanda da siyahıda görünsün
  if (!profiles[id] && Object.keys(profiles).length < MAX_PROFILES) {
    profiles[id] = { ...defaultProfile(), lastSeen: Date.now() };
    saveProfiles();
  }
}

function onRobot(ws, ip, qid, qname) {
  if (robots.size >= MAX_ROBOTS) { try { ws.close(4003, 'too many robots'); } catch (_) { /* */ } return; }
  const me = {
    id: null, ws, ip, since: Date.now(), fw: null,
    name: cleanText(qname, 40) || null, last: null, rtt: null, pollBusy: false, replaced: false,
  };
  registerRobot(me, ID_RE.test(qid || '') ? qid : LEGACY_ID);
  ws.isAlive = true;

  ws.on('pong', () => { ws.isAlive = true; });
  ws.on('message', (data, isBinary) => {
    ws.isAlive = true;
    if (isBinary) return;
    let m;
    try { m = JSON.parse(data.toString()); } catch (_) { return; }
    if (!m || typeof m !== 'object') return;

    if (m.t === 'res') {
      const p = pending.get(m.id);
      if (!p || p.robot !== me) return;
      clearTimeout(p.timer);
      pending.delete(m.id);
      const ok = m.c >= 200 && m.c < 300;
      if (p.p === '/status') {
        me.rtt = Date.now() - p.t0;
        if (ok) absorbStatus(me, m.d);
        if (!p.c) me.pollBusy = false;
      }
      if (p.p === '/mode' && ok) {
        const mm = /(?:^|&)m=(\d+)/.exec(p.qs || '');
        if (mm) { me.last = { ...(me.last || {}), mode: Number(mm[1]), at: Date.now() }; broadcastList(true); }
      }
      if (p.c) sendJ(p.c.ws, { t: 'res', id: p.id, r: me.id, c: m.c, d: m.d });
    } else if (m.t === 'hello') {
      me.fw = typeof m.fw === 'string' ? m.fw.slice(0, 20) : null;
      if (m.name) me.name = cleanText(m.name, 40) || me.name;
      if (typeof m.id === 'string' && ID_RE.test(m.id) && m.id !== me.id) {
        console.log(`[robot] ${me.id} → ${m.id} (hello)`);
        registerRobot(me, m.id);
      }
      broadcastList(true);
    }
  });
  ws.on('close', () => {
    if (robots.get(me.id) === me) {
      robots.delete(me.id);
      if (profiles[me.id]) { profiles[me.id].lastSeen = Date.now(); saveProfiles(); }
    }
    failPending(503, 'robot offline', (p) => p.robot === me);
    broadcastList(true);
    console.log(`[robot] qopdu: ${me.id} (${ip})`);
  });
  ws.on('error', () => {});

  console.log(`[robot] qoşuldu: ${me.id} (${ip})`);
  broadcastList(true);
  setTimeout(() => pollRobot(me), 300);
}

function pollRobot(r) {
  if (r.pollBusy || robots.get(r.id) !== r) return;
  r.pollBusy = sendToRobot(r, '/status', '', null, null);
}
setInterval(() => { for (const r of robots.values()) pollRobot(r); }, POLL_MS).unref();

// ---------------------------------------------------------------- UI side
function onUI(ws, ip) {
  const c = { ws, ip, authed: false, tokens: RATE_BURST, last: Date.now(), watch: null };
  uiClients.add(c);
  ws.isAlive = true;
  const authTimer = setTimeout(() => { if (!c.authed) ws.close(4001, 'auth timeout'); }, 10000);

  ws.on('pong', () => { ws.isAlive = true; });
  ws.on('message', (data, isBinary) => {
    ws.isAlive = true;
    if (isBinary) return;
    handleUI(c, data.toString());
  });
  ws.on('close', () => {
    clearTimeout(authTimer);
    uiClients.delete(c);
    failPending(0, '', (p) => p.c === c);
    if (c.watch) broadcastList(false);
  });
  ws.on('error', () => {});
}

function takeToken(c) {
  const now = Date.now();
  c.tokens = Math.min(RATE_BURST, c.tokens + ((now - c.last) / 1000) * RATE_PER_SEC);
  c.last = now;
  if (c.tokens < 1) return false;
  c.tokens -= 1;
  return true;
}

function handleUI(c, text) {
  let m;
  try { m = JSON.parse(text); } catch (_) { return; }
  if (!m || typeof m !== 'object') return;

  if (m.t === 'auth') {
    if (c.authed) return;
    if (!CONFIGURED) return sendJ(c.ws, { t: 'auth', ok: false, msg: 'Server tənzimlənməyib (ROBOT_KEY / UI_PASSWORD yoxdur).' });
    const wait = uiLimiter.wait(c.ip);
    if (wait) return sendJ(c.ws, { t: 'auth', ok: false, msg: `Çox səhv cəhd. ${wait} san sonra yenidən yoxla.` });
    if (typeof m.pw === 'string' && m.pw.length <= 200 && safeEqual(m.pw, UI_PASSWORD)) {
      uiLimiter.ok(c.ip);
      c.authed = true;
      sendJ(c.ws, { t: 'auth', ok: true });
      sendJ(c.ws, { t: 'robots', list: robotList(), modes: MODES });
      console.log(`[ui] giriş (${c.ip})`);
    } else {
      uiLimiter.fail(c.ip);
      console.log(`[ui] RED: yanlış şifrə (${c.ip})`);
      sendJ(c.ws, { t: 'auth', ok: false, msg: 'Şifrə yanlışdır' });
    }
    return;
  }

  if (!c.authed) return;

  switch (m.t) {
    case 'req': return relayReq(c, m);
    case 'list': return sendJ(c.ws, { t: 'robots', list: robotList(), modes: MODES });
    case 'watch': {
      const id = typeof m.r === 'string' && ID_RE.test(m.r) ? m.r : null;
      if (c.watch !== id) { c.watch = id; broadcastList(false); }
      return;
    }
    case 'profile': return saveProfile(c, m);
    case 'forget': {
      if (typeof m.r !== 'string' || !ID_RE.test(m.r)) return;
      if (robots.has(m.r)) return sendJ(c.ws, { t: 'notice', level: 'warn', msg: 'Onlayn robotu silmək olmaz.' });
      if (profiles[m.r]) { delete profiles[m.r]; saveProfiles(); broadcastList(true); }
      return;
    }
    default:
  }
}

function saveProfile(c, m) {
  if (typeof m.r !== 'string' || !ID_RE.test(m.r)) return;
  if (!profiles[m.r] && Object.keys(profiles).length >= MAX_PROFILES) {
    return sendJ(c.ws, { t: 'notice', level: 'error', msg: 'Profil limiti dolub.' });
  }
  const prof = normalizeProfile(m.data, profileOf(m.r));
  prof.updated = Date.now();
  profiles[m.r] = prof;
  saveProfiles();
  sendJ(c.ws, { t: 'notice', level: 'ok', msg: 'Fərdiləşdirmə yadda saxlanıldı.', r: m.r });

  // robot hazırda söndürülmüş modda işləyirsə — əl idarəsinə qaytar (dayandır)
  const r = robots.get(m.r);
  if (r && r.last && Number.isInteger(r.last.mode) && !modeAllowed(prof, r.last.mode)) {
    sendToRobot(r, '/mode', 'm=0', null, null);
  }
  broadcastList(true);
}

function relayReq(c, m) {
  if (!Number.isSafeInteger(m.id)) return;
  const id = m.id;
  const rid = typeof m.r === 'string' && ID_RE.test(m.r) ? m.r : null;
  const reply = (code, d) => sendJ(c.ws, { t: 'res', id, r: rid, c: code, d });

  const p = typeof m.p === 'string' ? m.p : '';
  if (!ROUTES.has(p)) return reply(404, 'unknown');
  if (!rid) return reply(400, 'robot id');
  const robot = robots.get(rid);
  if (!robot) return reply(503, 'robot offline');
  if (!takeToken(c)) return reply(429, 'too many requests');

  let qs = '';
  const q = {};
  if (m.q !== undefined && m.q !== null) {
    if (typeof m.q !== 'object' || Array.isArray(m.q)) return reply(400, 'bad query');
    const ent = Object.entries(m.q);
    if (ent.length > 6) return reply(400, 'bad query');
    const parts = [];
    for (const [k, v] of ent) {
      const sv = String(v);
      if (!/^[A-Za-z0-9_]{1,16}$/.test(k) || !/^[A-Za-z0-9._-]{0,24}$/.test(sv)) return reply(400, 'bad query');
      parts.push(`${k}=${sv}`);
      q[k] = sv;
    }
    qs = parts.join('&');
  }

  // fərdiləşdirmə qaydaları serverdə də tətbiq olunur
  const prof = profileOf(rid);
  if (p === '/mode') {
    const mm = parseInt(q.m, 10);
    if (!modeAllowed(prof, mm)) return reply(403, 'bu mod bu robot üçün söndürülüb');
  }
  if (p === '/servo' && prof.modules.servo === false) return reply(403, 'servo modulu söndürülüb');

  if (!sendToRobot(robot, p, qs, c, id)) return reply(429, 'busy');
}

// ---------------------------------------------------------------- heartbeat / shutdown
const hb = setInterval(() => {
  for (const w of [...wssUI.clients, ...wssRobot.clients]) {
    if (w.isAlive === false) { w.terminate(); continue; }
    w.isAlive = false;
    try { w.ping(); } catch (_) { /* ignore */ }
  }
}, HEARTBEAT_MS);

function shutdown(sig) {
  console.log(`${sig}: söndürülür…`);
  clearInterval(hb);
  for (const w of [...wssUI.clients, ...wssRobot.clients]) {
    try { w.close(1001, 'server restart'); } catch (_) { /* ignore */ }
  }
  server.close();
  setTimeout(() => process.exit(0), 1500);
}
process.on('SIGTERM', () => shutdown('SIGTERM'));
process.on('SIGINT', () => shutdown('SIGINT'));
process.on('unhandledRejection', (e) => console.error('unhandledRejection:', e));

server.listen(PORT, '0.0.0.0', () => {
  console.log(`SMARS Hub hazırdır: port ${PORT} (konfiqurasiya: ${CONFIGURED ? 'OK' : 'EKSİKDİR'})`);
});
