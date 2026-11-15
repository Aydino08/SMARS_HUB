/* SMARS Control Hub — frontend (vanilla JS, build addımı yoxdur) */
'use strict';
(() => {
  // ================================================================ helpers
  const $ = (s, el = document) => el.querySelector(s);
  const $$ = (s, el = document) => [...el.querySelectorAll(s)];
  const esc = (s) => String(s == null ? '' : s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  const initials = (name) => (String(name || '?').replace(/[^A-Za-z0-9ƏəÖöÜüĞğÇçŞşİı ]/g, ' ').trim().split(/\s+/).map((w) => w[0]).join('').slice(0, 2) || '?').toUpperCase();

  function ago(ts) {
    if (!ts) return '—';
    const s = Math.max(0, Math.round((Date.now() - ts) / 1000));
    if (s < 60) return s + ' san əvvəl';
    if (s < 3600) return Math.round(s / 60) + ' dəq əvvəl';
    if (s < 86400) return Math.round(s / 3600) + ' saat əvvəl';
    return Math.round(s / 86400) + ' gün əvvəl';
  }
  function dur(ts) {
    if (!ts) return '—';
    const s = Math.max(0, Math.round((Date.now() - ts) / 1000));
    const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60);
    return h ? `${h} saat ${m} dəq` : m ? `${m} dəq` : `${s} san`;
  }

  function toast(msg, level) {
    const el = document.createElement('div');
    el.className = 'toast ' + (level || '');
    el.textContent = msg;
    $('#toasts').appendChild(el);
    setTimeout(() => el.remove(), 3800);
  }

  const MODES_FALLBACK = [
    { id: 0, key: 'manual', name: 'Əl idarəsi' },
    { id: 1, key: 'obstacle', name: 'Maneədən qaçma' },
    { id: 2, key: 'line', name: 'Line following' },
    { id: 3, key: 'arena', name: 'Arena (kub çıxar)' },
  ];
  const MODE_DESC = {
    manual: 'D-pad və klaviatura ilə birbaşa sürmə. Həmişə açıqdır — robotu dayandırmaq üçün lazımdır.',
    obstacle: 'Ultrasəs sensoru ilə qarşıdakı maneələrdən avtomatik yayınma.',
    line: 'IR sensorlarla qara xətt üzrə hərəkət.',
    arena: 'Arenadakı kubları kənara itələmək (sumo/kürək rejimi).',
  };
  const MODULES = [
    { key: 'distance', name: 'Ultrasəs məsafə', desc: 'HC-SR04 məsafə göstəricisi' },
    { key: 'ir', name: 'IR sensorlar', desc: 'Xətt sensorlarının canlı dəyərləri' },
    { key: 'servo', name: 'Servo / qısqac', desc: 'Gripper və elbow servoları (söndürüləndə serverdə də bloklanır)' },
    { key: 'settings', name: 'Robot ayarları', desc: 'Robotun öz parametrləri paneli' },
  ];
  const PRESETS = {
    all: { label: 'Hamısı açıq', modes: { obstacle: true, line: true, arena: true }, modules: { ir: true, distance: true, servo: true, settings: true } },
    manual: { label: 'Yalnız əl idarəsi', modes: { obstacle: false, line: false, arena: false }, modules: { ir: false, distance: true, servo: false, settings: true } },
    dozer: { label: 'Kürək + HC-SR04', modes: { obstacle: true, line: false, arena: true }, modules: { ir: false, distance: true, servo: false, settings: true } },
    line: { label: 'Line follower', modes: { obstacle: false, line: true, arena: false }, modules: { ir: true, distance: false, servo: false, settings: true } },
  };
  const COLORS = ['#00e5ff', '#39ff14', '#ffcc00', '#ff9f00', '#ff2244', '#b44fff', '#4d8dff', '#ff5fd2', '#e8f6fa'];

  // ================================================================ Hub (WebSocket client)
  const Hub = {
    ws: null, open: false, authed: false, pw: '', remember: true, loginFailed: false, authMsg: '',
    retryMs: 1000, retryT: null, robots: [], modes: MODES_FALLBACK, listReady: false,
    nextId: 1, pend: new Map(), subs: new Set(), watching: null,

    on(fn) { this.subs.add(fn); return () => this.subs.delete(fn); },
    emit(type) { this.subs.forEach((f) => { try { f(type); } catch (e) { console.error(e); } }); },

    start() {
      let saved = '';
      try { saved = localStorage.getItem('smarsPw') || ''; } catch (_) { /* */ }
      if (saved) { this.pw = saved; this.remember = true; this.connect(); }
    },
    login(pw, remember) {
      this.pw = pw; this.remember = remember; this.loginFailed = false; this.authMsg = '';
      this.reconnect();
      this.emit('auth');
    },
    logout() {
      try { localStorage.removeItem('smarsPw'); } catch (_) { /* */ }
      this.pw = ''; this.loginFailed = false; this.authed = false; this.listReady = false; this.robots = [];
      const w = this.ws; this.ws = null; this.open = false;
      try { if (w) w.close(); } catch (_) { /* */ }
      clearTimeout(this.retryT);
      this.failAll('logout');
      this.emit('auth');
    },
    reconnect() {
      const w = this.ws; this.ws = null; this.open = false; this.authed = false;
      try { if (w) w.close(); } catch (_) { /* */ }
      this.connect();
    },
    connect() {
      clearTimeout(this.retryT);
      if (!this.pw) return;
      const proto = location.protocol === 'https:' ? 'wss://' : 'ws://';
      let s;
      try { s = new WebSocket(proto + location.host + '/ws/ui'); } catch (_) { this.retry(); return; }
      this.ws = s;
      s.onopen = () => {
        if (this.ws !== s) return;
        this.open = true; this.retryMs = 1000;
        s.send(JSON.stringify({ t: 'auth', pw: this.pw }));
        this.emit('conn');
      };
      s.onmessage = (e) => {
        if (this.ws !== s) return;
        let m; try { m = JSON.parse(e.data); } catch (_) { return; }
        this.onMsg(m);
      };
      s.onclose = () => {
        if (this.ws !== s) return;
        this.open = false; this.authed = false;
        this.failAll('closed');
        this.emit('conn');
        if (!this.loginFailed && this.pw) this.retry();
      };
      s.onerror = () => {};
    },
    retry() {
      clearTimeout(this.retryT);
      this.retryT = setTimeout(() => this.connect(), this.retryMs);
      this.retryMs = Math.min(Math.round(this.retryMs * 1.7), 8000);
    },
    failAll(msg) { this.pend.forEach((p) => { clearTimeout(p.t); p.rej(new Error(msg)); }); this.pend.clear(); },
    onMsg(m) {
      if (m.t === 'auth') {
        if (m.ok) {
          this.authed = true; this.authMsg = '';
          try { if (this.remember) localStorage.setItem('smarsPw', this.pw); else localStorage.removeItem('smarsPw'); } catch (_) { /* */ }
          if (this.watching) this.send({ t: 'watch', r: this.watching });
        } else {
          this.loginFailed = true; this.authed = false; this.authMsg = m.msg || 'Şifrə yanlışdır';
          try { localStorage.removeItem('smarsPw'); } catch (_) { /* */ }
          try { this.ws.close(); } catch (_) { /* */ }
        }
        this.emit('auth');
      } else if (m.t === 'robots') {
        this.robots = Array.isArray(m.list) ? m.list : [];
        if (Array.isArray(m.modes) && m.modes.length) this.modes = m.modes;
        this.listReady = true;
        this.emit('robots');
      } else if (m.t === 'res') {
        const p = this.pend.get(m.id);
        if (!p) return;
        this.pend.delete(m.id); clearTimeout(p.t);
        if (m.c >= 200 && m.c < 300) p.res(m.d);
        else p.rej(Object.assign(new Error(typeof m.d === 'string' ? m.d : 'xəta'), { code: m.c }));
      } else if (m.t === 'notice') {
        toast(m.msg, m.level);
        this.emit('notice');
      }
    },
    send(o) { if (this.ws && this.ws.readyState === 1) this.ws.send(JSON.stringify(o)); },
    api(robotId, url) {
      return new Promise((res, rej) => {
        if (!this.ws || this.ws.readyState !== 1 || !this.authed) { rej(new Error('offline')); return; }
        const qi = url.indexOf('?');
        const p = qi < 0 ? url : url.slice(0, qi);
        const q = {};
        if (qi >= 0) new URLSearchParams(url.slice(qi + 1)).forEach((v, k) => { q[k] = v; });
        const id = this.nextId++;
        const t = setTimeout(() => { this.pend.delete(id); rej(new Error('timeout')); }, 6500);
        this.pend.set(id, { res, rej, t });
        this.send({ t: 'req', id, r: robotId, p, q });
      });
    },
    json(robotId, url) { return this.api(robotId, url).then((d) => (typeof d === 'string' ? JSON.parse(d) : d)); },
    text(robotId, url) { return this.api(robotId, url).then((d) => (typeof d === 'string' ? d : JSON.stringify(d))); },
    watch(id) { this.watching = id; this.send({ t: 'watch', r: id }); },
    robot(id) { return this.robots.find((r) => r.id === id) || null; },
    modeName(i) { const m = this.modes.find((x) => x.id === i); return m ? m.name : '—'; },
    gated() { return !this.pw || this.loginFailed; },
  };

  function updateChrome() {
    const pill = $('#connPill');
    let txt = '● giriş yoxdur', ok = false;
    if (Hub.pw && !Hub.loginFailed) {
      if (!Hub.open) txt = '○ qoşulur…';
      else if (!Hub.authed) txt = '○ giriş…';
      else { ok = true; txt = '● ' + Hub.robots.filter((r) => r.on).length + ' robot onlayn'; }
    }
    pill.textContent = txt;
    pill.classList.toggle('ok', ok);
    $('#logoutBtn').hidden = !Hub.authed;
  }
  Hub.on(updateChrome);

  let publicInfo = null;
  function loadPublic() {
    return Promise.resolve().then(() => fetch('/api/public', { cache: 'no-store' }))
      .then((r) => r.json()).then((d) => { publicInfo = d; return d; }).catch(() => null);
  }

  // ================================================================ router
  let page = null;
  let curProtected = false;
  let lastGate = null;

  const routes = [
    [/^\/$/, HomePage, 'home', false],
    [/^\/robots\/?$/, RobotsPage, 'robots', true],
    [/^\/robot\/([A-Za-z0-9_-]{1,32})\/?$/, ControlPage, 'robots', true],
    [/^\/guide\/?$/, GuidePage, 'guide', false],
    [/^\/about\/?$/, AboutPage, 'about', false],
  ];

  function render() {
    if (page && page.unmount) { try { page.unmount(); } catch (e) { console.error(e); } }
    const path = location.pathname;
    let P = NotFoundPage, args = [], nav = '', prot = false;
    for (const [re, C, n, pr] of routes) {
      const m = re.exec(path);
      if (m) { P = C; args = m.slice(1); nav = n; prot = pr; break; }
    }
    curProtected = prot;
    lastGate = Hub.gated();
    if (prot && lastGate) P = LoginPage;
    $$('[data-nav]').forEach((a) => a.classList.toggle('active', a.dataset.nav === nav));
    $('#links').classList.remove('open');
    page = P(...args);
    const app = $('#app');
    app.innerHTML = page.html;
    document.title = (page.title ? page.title + ' · ' : '') + 'SMARS Hub';
    if (page.mount) page.mount(app);
    updateChrome();
  }
  function go(href) {
    if (href === location.pathname) return;
    history.pushState(null, '', href);
    render();
    window.scrollTo(0, 0);
  }
  // giriş vəziyyəti dəyişəndə qorunan səhifəni yenidən çək
  Hub.on(() => {
    const g = Hub.gated();
    if (g !== lastGate) { lastGate = g; if (curProtected) render(); }
  });

  document.addEventListener('click', (e) => {
    const a = e.target.closest('a[data-link]');
    if (a && !e.metaKey && !e.ctrlKey && !e.shiftKey && a.getAttribute('href').startsWith('/')) {
      e.preventDefault();
      go(a.getAttribute('href'));
    }
  });
  window.addEventListener('popstate', render);

  // ================================================================ pages
  const ROBOT_SVG = `
<svg viewBox="0 0 400 400" xmlns="http://www.w3.org/2000/svg" aria-hidden="true">
  <defs>
    <linearGradient id="g1" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#16303f"/><stop offset="1" stop-color="#0b1823"/></linearGradient>
    <radialGradient id="g2" cx=".5" cy=".5" r=".5"><stop offset="0" stop-color="#00e5ff" stop-opacity=".25"/><stop offset="1" stop-color="#00e5ff" stop-opacity="0"/></radialGradient>
  </defs>
  <circle cx="200" cy="210" r="180" fill="url(#g2)"/>
  <ellipse cx="200" cy="330" rx="150" ry="14" fill="#000" opacity=".45"/>
  <rect x="95" y="140" width="190" height="105" rx="14" fill="url(#g1)" stroke="#00e5ff" stroke-width="3"/>
  <rect x="150" y="98" width="88" height="50" rx="12" fill="#0d1520" stroke="#00e5ff" stroke-width="3"/>
  <circle cx="176" cy="123" r="14" fill="#04101a" stroke="#00e5ff" stroke-width="3"/>
  <circle cx="212" cy="123" r="14" fill="#04101a" stroke="#00e5ff" stroke-width="3"/>
  <circle cx="176" cy="123" r="5" fill="#00e5ff"/><circle cx="212" cy="123" r="5" fill="#00e5ff"/>
  <rect x="62" y="225" width="256" height="92" rx="46" fill="#0b1823" stroke="#ff9f00" stroke-width="5"/>
  <g fill="#ff9f00">${Array.from({ length: 11 }, (_, i) => `<rect x="${78 + i * 22}" y="309" width="12" height="11" rx="2"/>`).join('')}</g>
  <circle cx="108" cy="271" r="34" fill="#16303f" stroke="#00e5ff" stroke-width="4"/>
  <circle cx="272" cy="271" r="34" fill="#16303f" stroke="#00e5ff" stroke-width="4"/>
  <circle cx="108" cy="271" r="8" fill="#00e5ff"/><circle cx="272" cy="271" r="8" fill="#00e5ff"/>
  <path d="M300 205 q42 18 44 72 l-14 0 q-6 -40 -36 -52z" fill="#39ff14" opacity=".85"/>
  <path d="M120 165 h60 M120 182 h40" stroke="#00e5ff" stroke-width="3" opacity=".5" stroke-linecap="round"/>
</svg>`;

  function HomePage() {
    return {
      title: 'Ana səhifə',
      html: `
<section class="hero">
  <div>
    <div class="tag">SMARS · ESP32 · WEBSOCKET</div>
    <h1>SMARS robotlarını <span>internetdən</span> canlı idarə et</h1>
    <p>SMARS Control Hub bir neçə SMARS robotunu eyni yerdə birləşdirir. Robotlar özü serverə qoşulur,
       sən isə brauzerdən istənilən birini seçib sürürsən, avtonom rejimlərə keçirirsən və hər robot üçün
       lazım olan funksiyaları özün seçirsən.</p>
    <div class="row">
      <a class="btn" href="/robots" data-link>Robotlara bax →</a>
      <a class="btn-ghost" href="/guide" data-link>Robotu necə qoşum?</a>
    </div>
    <div class="stats">
      <div class="stat"><b id="stOnline">—</b><span>onlayn robot</span></div>
      <div class="stat"><b id="stKnown">—</b><span>tanınan robot</span></div>
      <div class="stat"><b>4</b><span>sürüş rejimi</span></div>
    </div>
    <div class="msg" id="cfgWarn" hidden></div>
  </div>
  <div class="hero-art">${ROBOT_SVG}</div>
</section>

<section class="section">
  <h2>Nə edə bilərsən?</h2>
  <p class="lead">Bütün idarəetmə bir səhifədədir — telefondan da, kompüterdən də.</p>
  <div class="grid g4">
    <div class="card feat"><div class="ico">📡</div><h4>Canlı robot siyahısı</h4><p>WebSocket ilə qoşulan bütün SMARS-lar real vaxtda görünür: rejim, məsafə, Wi-Fi siqnalı, gecikmə.</p></div>
    <div class="card feat"><div class="ico">🎮</div><h4>Uzaqdan idarəetmə</h4><p>D-pad, klaviatura (↑↓←→ / WASD), sürət, servo. Əlaqə qopsa robot özü dayanır.</p></div>
    <div class="card feat"><div class="ico">🧩</div><h4>Fərdiləşdirmə</h4><p>Hər robot üçün line following, maneədən qaçma, arena və modulları ayrıca aç-söndür.</p></div>
    <div class="card feat"><div class="ico">🔒</div><h4>Təhlükəsiz</h4><p>Robotlar gizli açarla, sayt şifrə ilə qorunur. Söndürülmüş rejimlər serverdə də bloklanır.</p></div>
  </div>
</section>

<section class="section">
  <h2>Necə işləyir?</h2>
  <p class="lead">Robotun olduğu şəbəkədə port açmaq lazım deyil — robot çölə doğru serverə qoşulur.</p>
  <div class="flow">
    <div class="node"><b>BRAUZER</b><span class="muted small">telefon / kompüter</span></div>
    <div class="arrow">⇄ WSS ⇄</div>
    <div class="node"><b>SMARS HUB</b><span class="muted small">Railway serveri</span></div>
    <div class="arrow">⇄ WSS ⇄</div>
    <div class="node"><b>SMARS × N</b><span class="muted small">ESP32 robotlar</span></div>
  </div>
</section>

<section class="section">
  <h2>3 addımda başla</h2>
  <div class="steps">
    <div class="step"><div><b>Serveri Railway-ə yerləşdir</b><br><span class="muted">ROBOT_KEY və UI_PASSWORD dəyişənlərini əlavə et.</span></div></div>
    <div class="step"><div><b>Robotu qoş</b><br><span class="muted">ESP32-də server ünvanını, açarı və robotun ID-sini yaz.</span></div></div>
    <div class="step"><div><b>Sayta daxil ol və sür</b><br><span class="muted">Robotlar səhifəsində robotu seç, rejimi və modulları fərdiləşdir.</span></div></div>
  </div>
</section>

<section class="section cta">
  <h2>Hazırsan?</h2>
  <p class="muted" style="margin:8px 0 18px">Şifrə ilə daxil ol və onlayn robotları gör.</p>
  <a class="btn" href="/robots" data-link>Robotlar səhifəsinə keç</a>
</section>`,
      mount() {
        loadPublic().then((d) => {
          if (!d) return;
          $('#stOnline') && ($('#stOnline').textContent = d.online);
          $('#stKnown') && ($('#stKnown').textContent = d.known);
          const w = $('#cfgWarn');
          if (w && !d.configured) { w.hidden = false; w.textContent = '⚠ Server hələ tənzimlənməyib: Railway-də ROBOT_KEY və UI_PASSWORD dəyişənlərini əlavə et.'; }
        });
      },
    };
  }

  function LoginPage() {
    let unsub = null;
    return {
      title: 'Giriş',
      html: `
<div class="login-wrap">
  <div class="card">
    <h3>Giriş</h3>
    <h4>Robotları görmək üçün daxil ol</h4>
    <p class="muted small" style="margin-bottom:16px">Şifrə serverin <code>UI_PASSWORD</code> dəyişənidir.</p>
    <form id="loginForm">
      <div class="field"><label for="pwIn">Şifrə</label>
        <input class="input" type="password" id="pwIn" autocomplete="current-password" required></div>
      <label class="chk" style="margin-bottom:16px"><input type="checkbox" id="remPw" checked> Bu cihazda yadda saxla</label>
      <button class="btn" style="width:100%" type="submit">Daxil ol</button>
    </form>
    <div class="msg" id="loginMsg"></div>
  </div>
</div>`,
      mount(app) {
        const msg = $('#loginMsg', app);
        const show = () => {
          if (Hub.authMsg) msg.textContent = Hub.authMsg;
          else if (Hub.pw && !Hub.loginFailed) msg.textContent = 'Yoxlanılır…';
        };
        $('#loginForm', app).addEventListener('submit', (e) => {
          e.preventDefault();
          const v = $('#pwIn', app).value;
          if (!v) return;
          msg.textContent = 'Yoxlanılır…';
          Hub.login(v, $('#remPw', app).checked);
        });
        unsub = Hub.on(show);
        show();
        loadPublic().then((d) => { if (d && !d.configured) msg.textContent = '⚠ Server tənzimlənməyib (ROBOT_KEY / UI_PASSWORD yoxdur).'; });
        setTimeout(() => { const i = $('#pwIn', app); if (i) i.focus(); }, 50);
      },
      unmount() { if (unsub) unsub(); },
    };
  }

  function robotCard(r) {
    const prof = r.profile || {};
    const color = prof.color || '#00e5ff';
    const last = r.last || {};
    const enabled = Hub.modes.filter((m) => m.id === 0 || !prof.modes || prof.modes[m.key] !== false);
    const dist = last.dist != null && last.dist >= 0 && last.dist < 900 ? last.dist + ' sm' : '—';
    return `
<a class="card rcard ${r.on ? '' : 'off'}" href="/robot/${esc(r.id)}" data-link style="--rc:${esc(color)}">
  <div class="stripe"></div>
  <div class="rtop">
    <div class="ravatar">${esc(initials(r.name))}</div>
    <div style="flex:1;min-width:0">
      <div class="rname">${esc(r.name)}</div>
      <div class="rid">${esc(r.id)}${r.fw ? ' · fw ' + esc(r.fw) : ''}</div>
    </div>
    <span class="pill ${r.on ? 'on' : 'off'}">${r.on ? '● ONLAYN' : '○ OFLAYN'}</span>
  </div>
  ${r.on ? `
  <div class="row"><span class="pill mode">${esc(last.mode != null ? Hub.modeName(last.mode) : '…')}</span>
    ${last.status ? `<span class="muted small">${esc(last.status)}</span>` : ''}</div>
  <div class="rmeta">
    <div><span>Məsafə</span><b>${esc(dist)}</b></div>
    <div><span>Wi-Fi</span><b>${last.rssi != null ? esc(last.rssi) + ' dBm' : '—'}</b></div>
    <div><span>Gecikmə</span><b>${r.rtt != null ? esc(r.rtt) + ' ms' : '—'}</b></div>
  </div>` : `<div class="muted small">Son görülmə: ${esc(ago(r.lastSeen))}</div>`}
  <div class="chips">${enabled.map((m) => `<span class="chip">${esc(m.name)}</span>`).join('')}
    ${r.viewers ? `<span class="chip">👁 ${r.viewers}</span>` : ''}</div>
</a>`;
  }

  function RobotsPage() {
    let unsub = null, tick = null;
    const ui = { q: '', onlyOn: false };
    return {
      title: 'Robotlar',
      html: `
<div class="page-head">
  <div><div class="crumbs">SMARS Hub</div><h1>Robotlar</h1>
    <p class="muted">Serverə qoşulan bütün SMARS-lar. İdarə etmək üçün robotun üzərinə bas.</p></div>
</div>
<div class="toolbar">
  <input class="input" id="q" placeholder="Ad və ya ID ilə axtar…" style="flex:1;min-width:200px">
  <label class="chk"><input type="checkbox" id="onlyOn"> Yalnız onlayn</label>
</div>
<div id="grid"></div>`,
      mount(app) {
        const grid = $('#grid', app);
        const draw = () => {
          if (!Hub.authed && !Hub.listReady) {
            grid.innerHTML = '<div class="empty"><div class="big">⏳</div>Serverə qoşulur…</div>';
            return;
          }
          const q = ui.q.toLowerCase();
          const list = Hub.robots.filter((r) => (!ui.onlyOn || r.on) && (!q || r.name.toLowerCase().includes(q) || r.id.toLowerCase().includes(q)));
          if (!Hub.robots.length) {
            grid.innerHTML = `<div class="empty"><div class="big">🤖</div><b>Hələ heç bir robot qoşulmayıb</b>
              <p style="margin:8px 0 16px">Robot serverə qoşulan kimi burada avtomatik görünəcək.</p>
              <a class="btn-ghost" href="/guide" data-link>Robotu necə qoşum?</a></div>`;
            return;
          }
          if (!list.length) { grid.innerHTML = '<div class="empty">Axtarışa uyğun robot yoxdur.</div>'; return; }
          grid.innerHTML = `<div class="grid g3">${list.map(robotCard).join('')}</div>`;
        };
        $('#q', app).addEventListener('input', (e) => { ui.q = e.target.value; draw(); });
        $('#onlyOn', app).addEventListener('change', (e) => { ui.onlyOn = e.target.checked; draw(); });
        Hub.watch(null);
        unsub = Hub.on((t) => { if (t === 'robots' || t === 'auth' || t === 'conn') draw(); });
        tick = setInterval(draw, 15000);       // "son görülmə" mətnləri üçün
        draw();
      },
      unmount() { if (unsub) unsub(); clearInterval(tick); },
    };
  }

  // ---------------------------------------------------------------- control page
  function ControlPage(id) {
    const st = {
      online: false, mode: 0, lastSwitch: 0, tLast: '', active: null, repT: null, pollT: null, pollBusy: false,
      BYK: {}, groups: [], settingsLoaded: false, settingsBusy: false, profKey: '', draft: null, dirty: false,
      thrT: {}, thrL: {}, setT: {}, cleanup: [],
    };
    const api = (url) => Hub.api(id, url);
    const robot = () => Hub.robot(id);
    const profile = () => { const r = robot(); return (r && r.profile) || { modes: {}, modules: {}, hiddenGroups: [], color: '#00e5ff', name: '' }; };
    const modeAllowed = (m) => { const d = Hub.modes.find((x) => x.id === m); return !!d && (d.id === 0 || profile().modes[d.key] !== false); };
    const modOn = (k) => profile().modules[k] !== false;
    const listen = (target, ev, fn, opt) => { target.addEventListener(ev, fn, opt); st.cleanup.push(() => target.removeEventListener(ev, fn, opt)); };
    let app = null;

    function sendThrottled(key, url) {
      const now = Date.now();
      clearTimeout(st.thrT[key]);
      if (now - (st.thrL[key] || 0) >= 100) { st.thrL[key] = now; api(url).catch(() => {}); }
      else st.thrT[key] = setTimeout(() => { st.thrL[key] = Date.now(); api(url).catch(() => {}); }, 100);
    }
    const setRange = (el, v) => {
      if (!el) return;
      el.value = v;
      const mn = parseFloat(el.min), mx = parseFloat(el.max);
      el.style.setProperty('--p', (((v - mn) / (mx - mn || 1)) * 100).toFixed(1) + '%');
    };

    // ---------- header / profile
    function drawHeader() {
      const r = robot();
      const prof = profile();
      const name = (r && r.name) || id;
      const color = prof.color || '#00e5ff';
      app.style.setProperty('--rc', color);
      $('#cAvatar', app).textContent = initials(name);
      $('#cAvatar', app).style.background = color;
      $('#cName', app).textContent = name;
      $('#cCrumb', app).textContent = name;
      document.title = name + ' · SMARS Hub';
      const on = !!(r && r.on);
      const pill = $('#cOn', app);
      pill.className = 'pill ' + (on ? 'on' : 'off');
      pill.textContent = on ? '● ONLAYN' : (Hub.authed ? '○ OFLAYN' : '○ QOŞULUR');
      $('#cViewers', app).textContent = r && r.viewers > 1 ? `👁 ${r.viewers} nəfər baxır` : '';
      $('#cRtt', app).textContent = r && r.on && r.rtt != null ? r.rtt + ' ms' : '';
      $('#offBanner', app).hidden = on || !Hub.authed;
      $('#offBanner', app).innerHTML = r
        ? `Robot hazırda oflayndır (son görülmə: ${esc(ago(r.lastSeen))}). Qoşulan kimi panel avtomatik aktivləşəcək.`
        : 'Bu ID ilə robot tapılmadı. Robot qoşulan kimi burada görünəcək.';
      $('#forgetBtn', app).disabled = on || !r;
      drawInfo();
    }

    function applyProfile() {
      const prof = profile();
      // mod düymələri
      const allowed = Hub.modes.filter((m) => modeAllowed(m.id));
      $('#modeGrid', app).innerHTML = allowed.map((m) =>
        `<button class="mbtn ${m.id === st.mode ? 'active' : ''}" data-mode="${m.id}">${esc(m.name)}</button>`).join('');
      if (!modeAllowed(st.mode)) applyModeUI(0);
      // modullar
      $('#cardDist', app).hidden = !modOn('distance');
      $('#cardIr', app).hidden = !modOn('ir');
      $('#cardServo', app).hidden = !modOn('servo');
      $('#settingsWrap', app).hidden = !modOn('settings');
      const hidden = new Set(prof.hiddenGroups || []);
      $$('[data-group]', app).forEach((d) => { d.hidden = hidden.has(d.dataset.group); });
      $('#sideCol', app).hidden = !modOn('distance') && !modOn('ir') && !modOn('servo');
      if (!st.dirty) loadDraft();
    }

    // ---------- modes
    function applyModeUI(m) {
      st.mode = m;
      $('#modeLabel', app).textContent = Hub.modeName(m);
      $('#autoBar', app).hidden = m === 0;
      $('#dpad', app).classList.toggle('disabled', m !== 0);
      $$('[data-mode]', app).forEach((b) => b.classList.toggle('active', Number(b.dataset.mode) === m));
      if (m !== 0) $('#tdir', app).textContent = 'AUTO';
    }
    function switchMode(m) {
      if (!modeAllowed(m)) return;
      const prev = st.mode;
      st.lastSwitch = Date.now();
      stopRepeat(); st.tLast = '';
      applyModeUI(m);
      api('/mode?m=' + m).catch((e) => {
        toast('Rejim dəyişmədi: ' + (e.code === 403 ? 'bu robotda söndürülüb' : e.message), 'error');
        applyModeUI(prev);
      });
    }

    // ---------- drive
    function stopRepeat() { if (st.repT) { clearInterval(st.repT); st.repT = null; } }
    function press(dir, el) {
      if (st.mode !== 0) return;
      if (dir === 'S') { stopRepeat(); st.tLast = ''; el.classList.add('on'); api('/tank?dir=S').catch(() => {}); return; }
      if (st.active && st.active !== el) st.active.classList.remove('on');
      el.classList.add('on'); st.active = el;
      if (dir === st.tLast) return;
      st.tLast = dir;
      stopRepeat();
      // robot ~700 ms əmr almasa özü dayanır — basılı olduqca təkrar göndər
      st.repT = setInterval(() => { api('/tank?dir=' + dir).catch(() => {}); }, 150);
      api('/tank?dir=' + dir).then((d) => { $('#tdir', app).textContent = String(d); }).catch(() => {});
    }
    function release(el) {
      el.classList.remove('on');
      if (el.dataset.dir === 'S') return;
      if (st.active === el) st.active = null;
      if (st.mode !== 0) return;
      stopRepeat(); st.tLast = '';
      api('/tank?dir=S').then((d) => { $('#tdir', app).textContent = String(d); }).catch(() => {});
    }
    function stopAll() {
      stopRepeat();
      if (st.active) { st.active.classList.remove('on'); st.active = null; }
      if (st.tLast !== '') { st.tLast = ''; api('/tank?dir=S').catch(() => {}); }
    }

    // ---------- servo
    function servoLimits() {
      const B = st.BYK;
      const lim = (lo, hi, el) => {
        if (!B[lo] || !B[hi] || !el) return;
        const a = Math.min(B[lo].v, B[hi].v), b = Math.max(B[lo].v, B[hi].v);
        el.min = a; el.max = b; setRange(el, clamp(Number(el.value), a, b));
      };
      lim('grpLo', 'grpHi', $('#grip', app));
      lim('elbLo', 'elbHi', $('#elbow', app));
    }
    function showServo(which, v) {
      $('#' + which + 'V', app).textContent = v + '°';
      $('#' + which + 'O', app).textContent = v;
    }

    // ---------- robot settings (robotun öz /settings siyahısı)
    const fmt = (s, v) => (s.t === 'f' ? String(parseFloat(Number(v).toFixed(2))) : String(Math.round(v)));
    function settingRow(s) {
      const k = esc(s.k);
      if (s.t === 'b') {
        return `<div class="srow bool"><div class="slabel">${esc(s.l)}</div>
          <label class="tgl"><input type="checkbox" data-sk="${k}" data-kind="b"><span></span></label>
          <button class="rst" title="Default" data-rst="${k}">↺</button></div>`;
      }
      return `<div class="srow"><div class="shead"><span class="slabel">${esc(s.l)}</span><span class="sdef">def ${esc(fmt(s, s.d))}</span>
          <button class="rst" title="Default" data-rst="${k}">↺</button></div>
        <div class="sctl"><input type="range" data-sk="${k}" data-kind="r" min="${esc(s.mn)}" max="${esc(s.mx)}" step="${esc(s.st)}">
          <input type="number" class="snum" data-sk="${k}" data-kind="n" min="${esc(s.mn)}" max="${esc(s.mx)}" step="${esc(s.st)}"></div></div>`;
    }
    function buildSettings(d) {
      st.BYK = {};
      st.groups = Array.isArray(d.g) ? d.g : [];
      (d.s || []).forEach((s) => { st.BYK[s.k] = s; });
      $('#settings', app).innerHTML = st.groups.map((g) => `
        <details class="card" data-group="${esc(g.i)}"><summary>${esc(g.t)}</summary>
          ${(d.s || []).filter((s) => s.g === g.i).map(settingRow).join('')}
          <button class="btn-ghost sm" data-rgroup="${esc(g.i)}">⟲ Bu qrupu default-a qaytar</button>
        </details>`).join('') || '<div class="muted small">Robot ayar siyahısı göndərmədi.</div>';
      (d.s || []).forEach((s) => reflect(s.k, s.v));
      st.settingsLoaded = true;
      applyProfile();
      drawGroupToggles();
    }
    function reflect(k, v) {
      const s = st.BYK[k];
      if (!s) return;
      s.v = v;
      $$(`[data-sk="${CSS.escape(k)}"]`, app).forEach((el) => {
        if (el.dataset.kind === 'b') el.checked = v >= 0.5;
        else if (el.dataset.kind === 'r') setRange(el, v);
        else el.value = v;
      });
      if (k === 'tankSpd') { setRange($('#spd', app), v); $('#spdO', app).textContent = v; $('#tspd', app).textContent = v; }
      if (/^(grp|elb)(Lo|Hi)$/.test(k)) servoLimits();
    }
    function setSet(k, v) {
      api('/set?k=' + encodeURIComponent(k) + '&v=' + encodeURIComponent(v)).then((t) => {
        const nv = parseFloat(t);
        if (!isNaN(nv)) reflect(k, nv);
        $('#svInd', app).textContent = '… saxlanılır';
      }).catch((e) => toast('Ayar yazılmadı: ' + e.message, 'error'));
    }
    function loadSettings() {
      if (st.settingsBusy) return;
      st.settingsBusy = true;
      Hub.json(id, '/settings').then((d) => { st.settingsBusy = false; buildSettings(d); })
        .catch(() => { st.settingsBusy = false; if (st.online) setTimeout(loadSettings, 2000); });
    }

    // ---------- status poll
    function poll() {
      if (!st.online || st.pollBusy || document.hidden) return;
      st.pollBusy = true;
      Hub.json(id, '/status').then((d) => {
        if (d.mode !== undefined && d.mode !== st.mode && Date.now() - st.lastSwitch > 1500) applyModeUI(d.mode);
        if (d.tankSpd !== undefined) { $('#tspd', app).textContent = d.tankSpd; if (document.activeElement !== $('#spd', app)) { setRange($('#spd', app), d.tankSpd); $('#spdO', app).textContent = d.tankSpd; } }
        $('#svInd', app).textContent = d.sv === 1 ? '✓ saxlanıldı' : d.sv === 0 ? '… saxlanılır' : '';
        if (st.mode === 0) { if (d.tankDir) $('#tdir', app).textContent = d.tankDir; }
        else {
          $('#tdir', app).textContent = d.status || 'AUTO';
          $('#autoStat', app).textContent = d.status || '—';
        }
        const okDist = typeof d.dist === 'number' && d.dist >= 0 && d.dist < 900;
        $('#autoDist', app).textContent = okDist ? d.dist + ' sm' : '—';
        $('#autoBar', app).classList.toggle('danger', st.mode === 1 && okDist && d.dist < 25);
        const big = $('#distBig', app);
        big.innerHTML = okDist ? `${esc(d.dist)} <small>sm</small>` : '— <small>sm</small>';
        big.classList.toggle('near', okDist && d.dist < 20);
        $('#distBar', app).style.width = okDist ? clamp(100 - (d.dist / 200) * 100, 2, 100) + '%' : '0%';
        if (Array.isArray(d.ir)) {
          $('#irRows', app).innerHTML = d.ir.map((p, i) => {
            const blk = Array.isArray(d.blk) ? d.blk[i] : null;
            const names = ['S1 · ORTA', 'S2 · SOL', 'S3 · SAĞ'];
            return `<div class="ir-row"><div class="ir-name">${esc(names[i] || 'S' + (i + 1))}</div>
              <div class="bar"><i style="width:${clamp(Number(p) || 0, 0, 100)}%"></i></div>
              <div class="ir-pct">${esc((Number(p) || 0).toFixed(1))}%</div>
              <div class="ir-tag ${blk == null ? '' : blk ? 'black' : 'white'}">${blk == null ? '—' : blk ? 'QARA' : 'AĞ'}</div></div>`;
          }).join('');
        }
        if (d.gripAngle !== undefined && document.activeElement !== $('#grip', app)) { setRange($('#grip', app), d.gripAngle); showServo('grip', d.gripAngle); }
        if (d.elbowAngle !== undefined && document.activeElement !== $('#elbow', app)) { setRange($('#elbow', app), d.elbowAngle); showServo('elbow', d.elbowAngle); }
        st.lastStatus = d;
      }).catch(() => {}).then(() => { st.pollBusy = false; });
    }

    function onOnline() {
      st.settingsLoaded = false;
      Hub.json(id, '/status').then((d) => { if (d.mode !== undefined) applyModeUI(d.mode); }).catch(() => {});
      loadSettings();
    }

    // ---------- customization
    function loadDraft() {
      st.draft = JSON.parse(JSON.stringify(profile()));
      st.draft.modes = st.draft.modes || {};
      st.draft.modules = st.draft.modules || {};
      st.draft.hiddenGroups = st.draft.hiddenGroups || [];
      st.dirty = false;
      drawCustom();
    }
    function markDirty() { st.dirty = true; $('#saveState', app).textContent = 'Yadda saxlanmamış dəyişikliklər var'; }
    function opt(name, desc, attr, checked, disabled) {
      return `<div class="opt"><div class="txt"><b>${esc(name)}</b><span>${esc(desc)}</span></div>
        <label class="tgl"><input type="checkbox" ${attr} ${checked ? 'checked' : ''} ${disabled ? 'disabled' : ''}><span></span></label></div>`;
    }
    function drawCustom() {
      const d = st.draft;
      if (!d) return;
      const r = robot();
      $('#pName', app).value = d.name || '';
      $('#pName', app).placeholder = (r && r.name) || id;
      $('#swatches', app).innerHTML = COLORS.map((c) =>
        `<button class="sw ${d.color === c ? 'sel' : ''}" data-color="${c}" style="--c:${c}" title="${c}"></button>`).join('');
      $('#pModes', app).innerHTML = Hub.modes.map((m) => opt(m.name, MODE_DESC[m.key] || '', `data-pmode="${esc(m.key)}"`,
        m.id === 0 || d.modes[m.key] !== false, m.id === 0)).join('');
      $('#pModules', app).innerHTML = MODULES.map((m) => opt(m.name, m.desc, `data-pmod="${m.key}"`, d.modules[m.key] !== false)).join('');
      drawGroupToggles();
      $('#saveState', app).textContent = st.dirty ? 'Yadda saxlanmamış dəyişikliklər var' : (profile().updated ? 'Son dəyişiklik: ' + ago(profile().updated) : 'Default profil');
    }
    function drawGroupToggles() {
      const box = $('#pGroups', app);
      if (!box || !st.draft) return;
      if (!st.settingsLoaded || !st.groups.length) {
        box.innerHTML = '<div class="muted small">Robot onlayn olanda onun ayar qrupları burada görünəcək.</div>';
        return;
      }
      const hidden = new Set(st.draft.hiddenGroups);
      box.innerHTML = st.groups.map((g) => opt(g.t, 'Ayarlar panelində göstər', `data-pgroup="${esc(g.i)}"`, !hidden.has(g.i))).join('');
    }

    // ---------- info tab
    function drawInfo() {
      const r = robot();
      const s = st.lastStatus || {};
      const last = (r && r.last) || {};
      $('#infoKv', app).innerHTML = `
        <dt>ID</dt><dd>${esc(id)}</dd>
        <dt>Vəziyyət</dt><dd>${r && r.on ? 'onlayn' : 'oflayn'}</dd>
        <dt>Firmware</dt><dd>${esc((r && r.fw) || '—')}</dd>
        <dt>Qoşulub</dt><dd>${r && r.on ? esc(dur(r.since)) + ' əvvəl' : '—'}</dd>
        <dt>Son görülmə</dt><dd>${r && !r.on ? esc(ago(r.lastSeen)) : 'indi'}</dd>
        <dt>Gecikmə</dt><dd>${r && r.rtt != null ? esc(r.rtt) + ' ms' : '—'}</dd>
        <dt>Wi-Fi</dt><dd>${esc(s.sta || '—')} · ${esc(s.rssi != null ? s.rssi : (last.rssi != null ? last.rssi : '—'))} dBm</dd>
        <dt>Rejim</dt><dd>${esc(last.mode != null ? Hub.modeName(last.mode) : '—')}</dd>
        <dt>Baxanlar</dt><dd>${r ? esc(r.viewers) : 0}</dd>`;
    }

    return {
      title: id,
      html: `
<div class="crumbs"><a href="/robots" data-link>Robotlar</a> / <span id="cCrumb">${esc(id)}</span></div>
<div class="ctrl-head">
  <div class="ravatar" id="cAvatar">?</div>
  <div><h1 id="cName">${esc(id)}</h1>
    <div class="row" style="margin-top:4px"><span class="rid">${esc(id)}</span>
      <span class="pill off" id="cOn">○</span><span class="muted small" id="cRtt"></span><span class="muted small" id="cViewers"></span></div></div>
  <div class="right"><a class="btn-ghost sm" href="/robots" data-link>← Bütün robotlar</a></div>
</div>
<div class="tabs" id="tabs">
  <button data-tab="control" class="active">🎮 İdarəetmə</button>
  <button data-tab="custom">🧩 Fərdiləşdirmə</button>
  <button data-tab="info">ℹ Məlumat</button>
</div>
<div class="offline-banner" id="offBanner" hidden></div>

<section data-pane="control">
  <div class="ctrl-grid">
    <div class="col">
      <div class="card">
        <h3>Sürüş rejimi</h3>
        <div class="row" style="justify-content:space-between;margin-bottom:12px">
          <span class="muted small">Aktiv rejim</span><b id="modeLabel" style="font-family:var(--disp);font-size:.8rem;color:var(--acc);letter-spacing:1px">—</b></div>
        <div class="mode-grid" id="modeGrid"></div>
        <div class="auto-bar" id="autoBar" hidden>
          <div><div class="label">VƏZİYYƏT</div><div class="val" id="autoStat">—</div></div>
          <div style="text-align:right"><div class="label">MƏSAFƏ</div><div class="val" id="autoDist" style="color:var(--acc)">—</div></div>
        </div>
      </div>
      <div class="card">
        <h3>Tank drive</h3>
        <div class="sbar"><div>İSTİQAMƏT: <b id="tdir">STOP</b></div><div>SÜRƏT: <b id="tspd">—</b></div></div>
        <div class="slrow"><label>SÜRƏT</label><input type="range" id="spd" min="60" max="255" value="180"><output id="spdO">180</output></div>
        <div class="dpad" id="dpad">
          <div class="dbtn blank"></div><div class="dbtn" data-dir="F">▲</div><div class="dbtn blank"></div>
          <div class="dbtn" data-dir="L">◀</div><div class="dbtn stop" data-dir="S">STOP</div><div class="dbtn" data-dir="R">▶</div>
          <div class="dbtn blank"></div><div class="dbtn" data-dir="B">▼</div><div class="dbtn blank"></div>
        </div>
        <div class="hint">Klaviatura: ↑ ↓ ← → və ya W A S D · Boşluq = STOP · Esc = əl idarəsinə qayıt</div>
      </div>
    </div>
    <div class="col" id="sideCol">
      <div class="card" id="cardDist">
        <h3>Ultrasəs məsafə (HC-SR04)</h3>
        <div class="dist-big" id="distBig">— <small>sm</small></div>
        <div class="bar"><i id="distBar"></i></div>
        <div class="hint">Zolaq dolduqca maneə yaxınlaşır (0–200 sm)</div>
      </div>
      <div class="card" id="cardIr">
        <h3>IR sensorlar</h3>
        <div id="irRows"><div class="muted small">Məlumat gözlənilir…</div></div>
      </div>
      <div class="card" id="cardServo">
        <h3>Servo</h3>
        <div class="servo-vals"><div><span>GRIPPER</span><b id="gripV" style="color:var(--srv1)">90°</b></div>
          <div><span>ELBOW</span><b id="elbowV" style="color:var(--srv2)">90°</b></div></div>
        <div class="slrow"><label>GRIPPER</label><input type="range" id="grip" min="0" max="180" value="90" style="--c:var(--srv1)"><output id="gripO">90</output></div>
        <div class="slrow"><label>ELBOW</label><input type="range" id="elbow" min="0" max="180" value="90" style="--c:var(--srv2)"><output id="elbowO">90</output></div>
        <button class="btn-ghost sm" id="centerBtn" style="width:100%">⟲ Mərkəzə gətir (90°)</button>
      </div>
    </div>
  </div>
  <div id="settingsWrap" style="margin-top:22px">
    <div class="page-head" style="margin-bottom:12px"><h2 style="font-family:var(--disp);font-size:1rem;letter-spacing:2px">⚙ Robot ayarları</h2>
      <span class="muted small" id="svInd"></span></div>
    <div class="grid g2" id="settings"><div class="muted small">Robot onlayn olanda ayarlar yüklənəcək.</div></div>
    <div class="row" style="margin-top:14px"><button class="btn-ghost sm warn" data-rgroup="all">⟲ Bütün ayarları default-a qaytar</button></div>
  </div>
</section>

<section data-pane="custom" hidden>
  <div class="grid g2">
    <div class="col">
      <div class="card">
        <h3>Görünüş</h3>
        <div class="field"><label for="pName">Görünən ad</label><input class="input" id="pName" maxlength="40"></div>
        <div class="field"><label>Rəng</label><div class="swatches" id="swatches"></div></div>
      </div>
      <div class="card">
        <h3>Hazır şablonlar</h3>
        <div class="row">${Object.entries(PRESETS).map(([k, p]) => `<button class="btn-ghost sm" data-preset="${k}">${esc(p.label)}</button>`).join('')}</div>
        <p class="muted small" style="margin-top:10px">Şablon yalnız formu doldurur — tətbiq etmək üçün “Yadda saxla” düyməsini bas.</p>
      </div>
      <div class="card">
        <h3>Ayar qrupları</h3>
        <div id="pGroups"></div>
      </div>
    </div>
    <div class="col">
      <div class="card"><h3>Sürüş rejimləri</h3><div id="pModes"></div></div>
      <div class="card"><h3>Modullar / panellər</h3><div id="pModules"></div></div>
    </div>
  </div>
  <div class="save-bar">
    <span class="muted small" id="saveState" style="margin-right:auto"></span>
    <button class="btn-ghost" id="pReset">Dəyişiklikləri ləğv et</button>
    <button class="btn" id="pSave">Yadda saxla</button>
  </div>
  <p class="muted small" style="margin-top:12px">Fərdiləşdirmə serverdə saxlanılır və bu robota baxan hər kəs üçün tətbiq olunur.
    Söndürülmüş rejimlər serverdə də bloklanır; robot hazırda söndürülmüş rejimdədirsə, avtomatik əl idarəsinə (STOP) keçirilir.</p>
</section>

<section data-pane="info" hidden>
  <div class="grid g2">
    <div class="card"><h3>Robot</h3><dl class="kv" id="infoKv"></dl></div>
    <div class="card"><h3>Təhlükə zonası</h3>
      <p class="muted small" style="margin-bottom:12px">Oflayn robotu siyahıdan və profilini serverdən sil. Robot yenidən qoşulsa, default profillə geri gələcək.</p>
      <button class="btn-ghost warn" id="forgetBtn">Robotu siyahıdan sil</button></div>
  </div>
</section>`,

      mount(root) {
        app = root;
        Hub.watch(id);

        // tabs
        listen($('#tabs', app), 'click', (e) => {
          const b = e.target.closest('[data-tab]');
          if (!b) return;
          $$('[data-tab]', app).forEach((x) => x.classList.toggle('active', x === b));
          $$('[data-pane]', app).forEach((p) => { p.hidden = p.dataset.pane !== b.dataset.tab; });
          if (b.dataset.tab !== 'control') stopAll();
        });

        // modes
        listen($('#modeGrid', app), 'click', (e) => { const b = e.target.closest('[data-mode]'); if (b) switchMode(Number(b.dataset.mode)); });

        // d-pad (pointer events: mouse + touch + pen)
        $$('[data-dir]', app).forEach((el) => {
          el.addEventListener('pointerdown', (e) => { e.preventDefault(); try { el.setPointerCapture(e.pointerId); } catch (_) { /* */ } press(el.dataset.dir, el); });
          const up = () => release(el);
          el.addEventListener('pointerup', up);
          el.addEventListener('pointercancel', up);
          el.addEventListener('lostpointercapture', () => { if (el.classList.contains('on')) release(el); });
        });
        const KM = { ArrowUp: 'F', ArrowDown: 'B', ArrowLeft: 'L', ArrowRight: 'R', w: 'F', s: 'B', a: 'L', d: 'R', W: 'F', S: 'B', A: 'L', D: 'R', ' ': 'S' };
        const btnOf = (dir) => $(`[data-dir="${dir}"]`, app);
        listen(document, 'keydown', (e) => {
          if (/INPUT|TEXTAREA|SELECT/.test(e.target.tagName)) return;
          if ($('[data-pane="control"]', app).hidden) return;
          if (e.key === 'Escape') { switchMode(0); return; }
          const dir = KM[e.key];
          if (!dir || st.mode !== 0) return;
          e.preventDefault();
          if (e.repeat) return;
          press(dir, btnOf(dir));
        });
        listen(document, 'keyup', (e) => {
          const dir = KM[e.key];
          if (!dir || dir === 'S') { if (dir === 'S') btnOf('S').classList.remove('on'); return; }
          const el = btnOf(dir);
          if (el.classList.contains('on')) release(el);
        });
        listen(window, 'blur', stopAll);
        listen(document, 'visibilitychange', () => { if (document.hidden) stopAll(); });

        // speed + servo
        listen($('#spd', app), 'input', (e) => {
          const v = e.target.value;
          setRange(e.target, v);
          $('#spdO', app).textContent = v; $('#tspd', app).textContent = v;
          sendThrottled('spd', '/tank?spd=' + v);
          if (st.BYK.tankSpd) st.BYK.tankSpd.v = Number(v);
        });
        ['grip', 'elbow'].forEach((w) => listen($('#' + w, app), 'input', (e) => {
          const v = Number(e.target.value);
          setRange(e.target, v); showServo(w, v);
          sendThrottled(w, '/servo?' + w + '=' + v);
        }));
        listen($('#centerBtn', app), 'click', () => {
          ['grip', 'elbow'].forEach((w) => { setRange($('#' + w, app), 90); showServo(w, 90); });
          api('/servo?grip=90').catch(() => {}); api('/servo?elbow=90').catch(() => {});
        });

        // robot settings (event delegation)
        const sbox = $('#settingsWrap', app);
        listen(sbox, 'input', (e) => {
          const el = e.target, k = el.dataset.sk;
          if (!k || el.dataset.kind !== 'r') return;
          const v = parseFloat(el.value);
          setRange(el, v);
          const n = $(`[data-sk="${CSS.escape(k)}"][data-kind="n"]`, app); if (n) n.value = v;
          clearTimeout(st.setT[k]);
          st.setT[k] = setTimeout(() => setSet(k, v), 200);
        });
        listen(sbox, 'change', (e) => {
          const el = e.target, k = el.dataset.sk;
          if (!k) return;
          if (el.dataset.kind === 'b') setSet(k, el.checked ? 1 : 0);
          else if (el.dataset.kind === 'n') { const v = parseFloat(el.value); if (!isNaN(v)) setSet(k, v); }
        });
        listen(sbox, 'click', (e) => {
          const r = e.target.closest('[data-rst]');
          if (r && st.BYK[r.dataset.rst]) { setSet(r.dataset.rst, st.BYK[r.dataset.rst].d); return; }
          const g = e.target.closest('[data-rgroup]');
          if (!g) return;
          const all = g.dataset.rgroup === 'all';
          if (!confirm(all ? 'Bütün ayarlar default-a qaytarılsın?' : 'Bu qrupun ayarları default-a qaytarılsın?')) return;
          api('/reset?g=' + encodeURIComponent(g.dataset.rgroup)).then(() => Hub.json(id, '/settings'))
            .then((d) => (d.s || []).forEach((s) => reflect(s.k, s.v))).catch((er) => toast('Sıfırlanmadı: ' + er.message, 'error'));
        });

        // customization
        const cust = $('[data-pane="custom"]', app);
        listen($('#pName', app), 'input', (e) => { st.draft.name = e.target.value; markDirty(); });
        listen(cust, 'click', (e) => {
          const sw = e.target.closest('[data-color]');
          if (sw) { st.draft.color = sw.dataset.color; markDirty(); $$('.sw', app).forEach((x) => x.classList.toggle('sel', x === sw)); return; }
          const pr = e.target.closest('[data-preset]');
          if (pr) {
            const p = PRESETS[pr.dataset.preset];
            st.draft.modes = { ...p.modes }; st.draft.modules = { ...p.modules };
            markDirty(); drawCustom(); markDirty();
          }
        });
        listen(cust, 'change', (e) => {
          const el = e.target;
          if (el.dataset.pmode) { st.draft.modes[el.dataset.pmode] = el.checked; markDirty(); }
          else if (el.dataset.pmod) { st.draft.modules[el.dataset.pmod] = el.checked; markDirty(); }
          else if (el.dataset.pgroup) {
            const set = new Set(st.draft.hiddenGroups);
            if (el.checked) set.delete(el.dataset.pgroup); else set.add(el.dataset.pgroup);
            st.draft.hiddenGroups = [...set]; markDirty();
          }
        });
        listen($('#pSave', app), 'click', () => {
          Hub.send({ t: 'profile', r: id, data: { name: st.draft.name, color: st.draft.color, modes: st.draft.modes, modules: st.draft.modules, hiddenGroups: st.draft.hiddenGroups } });
          st.dirty = false;
          $('#saveState', app).textContent = 'Göndərilir…';
        });
        listen($('#pReset', app), 'click', () => loadDraft());
        listen($('#forgetBtn', app), 'click', () => {
          if (!confirm('Robot siyahıdan silinsin?')) return;
          Hub.send({ t: 'forget', r: id });
          go('/robots');
        });

        // hub updates
        const sync = () => {
          const r = robot();
          const on = !!(r && r.on && Hub.authed);
          if (on && !st.online) { st.online = true; onOnline(); }
          if (!on && st.online) { st.online = false; stopRepeat(); st.tLast = ''; }
          $('#dpad', app).classList.toggle('disabled', !on || st.mode !== 0);
          const key = JSON.stringify(r ? r.profile : null);
          if (key !== st.profKey) { st.profKey = key; applyProfile(); }
          drawHeader();
        };
        st.cleanup.push(Hub.on((t) => { if (t === 'robots' || t === 'auth' || t === 'conn') sync(); }));
        st.pollT = setInterval(poll, 600);
        st.infoT = setInterval(() => { drawInfo(); }, 5000);
        applyModeUI(0);
        loadDraft();
        sync();
      },

      unmount() {
        stopAll();
        clearInterval(st.pollT); clearInterval(st.infoT);
        Object.values(st.thrT).forEach(clearTimeout);
        Object.values(st.setT).forEach(clearTimeout);
        st.cleanup.forEach((f) => f());
        Hub.watch(null);
      },
    };
  }

  function GuidePage() {
    return {
      title: 'Qoşulma bələdçisi',
      html: `
<div class="doc">
  <div class="crumbs">SMARS Hub</div>
  <h1 style="font-family:var(--disp);letter-spacing:2px;font-size:1.6rem;margin-bottom:10px">Qoşulma bələdçisi</h1>
  <p class="lead">Serveri Railway-ə yerləşdirmək və SMARS robotlarını ona qoşmaq üçün addımlar.</p>

  <h2>1. Serveri Railway-ə yerləşdir</h2>
  <ol>
    <li>Layihəni GitHub repozitoriyasına yüklə (<code>node_modules</code> və <code>data/</code> yüklənmir).</li>
    <li><a href="https://railway.com" target="_blank" rel="noopener">railway.com</a> → <b>New Project → Deploy from GitHub repo</b>.</li>
    <li>Servis → <b>Variables</b>:
      <table><tr><th>Ad</th><th>Dəyər</th></tr>
        <tr><td><code>ROBOT_KEY</code></td><td>robotların gizli açarı, min. 12 simvol (<code>openssl rand -hex 16</code>)</td></tr>
        <tr><td><code>UI_PASSWORD</code></td><td>sayta giriş şifrəsi, min. 6 simvol</td></tr>
        <tr><td><code>DATA_DIR</code></td><td>istəyə bağlı: <code>/data</code> (aşağıdakı Volume ilə)</td></tr></table></li>
    <li><b>Fərdiləşdirmələr itməsin deyə:</b> servisə <b>Volume</b> əlavə et, mount path <code>/data</code>, və <code>DATA_DIR=/data</code> yaz. Volume olmasa, profillər hər deploy-da sıfırlanır.</li>
    <li><b>Settings → Networking → Generate Domain</b>. Yoxlama: <code>https://SƏNİN-DOMEN/healthz</code> → <code>{"ok":true,"configured":true}</code>.</li>
  </ol>

  <h2>2. Robotu qoş (ESP32 firmware)</h2>
  <p>Robot serverə WebSocket ilə qoşulur və hər robotun öz <b>ID</b>-si olmalıdır (hərf, rəqəm, <code>-</code>, <code>_</code>; maks. 32 simvol).
     ID-ni yolda ver:</p>
<pre>wss://SƏNİN-DOMEN/ws/robot?id=smars-01&amp;name=Qırmızı%20SMARS
Authorization: Bearer &lt;ROBOT_KEY&gt;</pre>
  <p>Arduino <code>WebSocketsClient</code> ilə nümunə:</p>
<pre>webSocket.setExtraHeaders("Authorization: Bearer " CLOUD_KEY);
webSocket.beginSSL(CLOUD_HOST, 443, "/ws/robot?id=smars-01&amp;name=Kurek");</pre>
  <p>Alternativ olaraq ID-ni qoşulandan sonra <code>hello</code> mesajında göndərmək olar:</p>
<pre>{"t":"hello","fw":"2.1.0","id":"smars-01","name":"Kürək SMARS"}</pre>
  <p>ID göndərməyən köhnə firmware <code>smars-1</code> adı ilə qeydə alınır (belə robotdan yalnız biri qoşula bilər).
     Eyni ID ilə ikinci robot qoşulsa, köhnə bağlantı bağlanır.</p>

  <h2>3. Protokol (firmware ilə eyni)</h2>
  <table>
    <tr><th>İstiqamət</th><th>Format</th></tr>
    <tr><td>Server → robot</td><td><code>R|&lt;gid&gt;|/yol?a=1&amp;b=2</code></td></tr>
    <tr><td>Robot → server</td><td><code>{"t":"res","id":&lt;gid&gt;,"c":200,"d":&lt;cavab&gt;}</code></td></tr>
    <tr><td>Robot → server</td><td><code>{"t":"hello","fw":"…","id":"…","name":"…"}</code></td></tr>
  </table>
  <p>İcazəli yollar: <code>/tank /mode /servo /status /settings /set /reset</code>. Server hər robotdan 4 saniyədən bir
     <code>/status</code> soruşur ki, siyahıda rejim, məsafə və Wi-Fi görünsün.</p>
  <table>
    <tr><th>Yol</th><th>Nümunə</th></tr>
    <tr><td><code>/mode</code></td><td><code>m=0</code> əl · <code>1</code> maneədən qaçma · <code>2</code> line following · <code>3</code> arena</td></tr>
    <tr><td><code>/tank</code></td><td><code>dir=F|B|L|R|S</code>, <code>spd=60..255</code></td></tr>
    <tr><td><code>/servo</code></td><td><code>grip=0..180</code>, <code>elbow=0..180</code></td></tr>
    <tr><td><code>/status</code></td><td>JSON: <code>mode, status, dist, rssi, sta, tankDir, tankSpd, ir[], blk[], gripAngle, elbowAngle, sv</code></td></tr>
    <tr><td><code>/settings</code></td><td>JSON: <code>{"g":[{"i","t"}],"s":[{"k","l","t","d","mn","mx","st","v","g"}]}</code></td></tr>
  </table>

  <h2>4. Simulyatorla sınaq (robotsuz)</h2>
<pre>npm install
ROBOT_KEY=test-robot-key-123 UI_PASSWORD=parol123 npm start
# ikinci terminalda — 3 virtual SMARS:
ROBOT_KEY=test-robot-key-123 npm run sim -- --count 3</pre>
  <p>Brauzerdə <code>http://localhost:3000</code> aç, <code>parol123</code> ilə daxil ol.</p>

  <h2>Problemlər</h2>
  <table>
    <tr><th>Əlamət</th><th>Həll</th></tr>
    <tr><td>Giriş: “Server tənzimlənməyib”</td><td><code>ROBOT_KEY</code> (≥12) və <code>UI_PASSWORD</code> (≥6) Variables-də yoxdur.</td></tr>
    <tr><td>Robot siyahıda görünmür</td><td>Wi-Fi 2.4 GHz olmalıdır; host-da <code>https://</code> yazma; açar <code>ROBOT_KEY</code> ilə eyni olsun.</td></tr>
    <tr><td>İki robot bir-birini “atır”</td><td>Hər robota ayrı <code>id</code> ver.</td></tr>
    <tr><td>Fərdiləşdirmə deploy-dan sonra itir</td><td>Volume + <code>DATA_DIR=/data</code> əlavə et.</td></tr>
    <tr><td>“Rejim dəyişmədi: söndürülüb”</td><td>Həmin rejim Fərdiləşdirmə bölməsində bağlıdır.</td></tr>
  </table>
</div>`,
    };
  }

  function AboutPage() {
    return {
      title: 'Haqqında',
      html: `
<div class="doc">
  <div class="crumbs">SMARS Hub</div>
  <h1 style="font-family:var(--disp);letter-spacing:2px;font-size:1.6rem;margin-bottom:10px">Haqqında</h1>
  <p class="lead">SMARS Control Hub — SMARS robotlarını brauzerdən, internet üzərindən idarə etmək üçün açıq veb platformadır.</p>

  <h2>SMARS nədir?</h2>
  <p><b>SMARS</b> (Screwless/Screwed Modular Assemblable Robotic System) — təhsil məqsədli, 3D çap olunan, modul tırtıllı robotdur.
     Az detaldan ibarətdir, vintsiz yığılır və üzərinə sensor və alət modulları (ultrasəs sensoru, xətt sensorları, kürək, qısqac) taxmaq olur.
     SMARS 3D modelləri öz müəllifinin lisenziyası (CC BY-NC-SA) ilə yayılır.</p>

  <h2>Bu platforma nə edir?</h2>
  <div class="grid g2" style="margin:14px 0">
    <div class="card"><h4>Çox robot, bir sayt</h4><p class="muted small">Hər robot öz ID-si ilə qoşulur. Siyahıda hamısı canlı görünür; istənilən birini seçib idarə edirsən.</p></div>
    <div class="card"><h4>Robot üzrə fərdiləşdirmə</h4><p class="muted small">Line following, maneədən qaçma, arena rejimlərini və IR / ultrasəs / servo panellərini hər robot üçün ayrıca aç-söndür.</p></div>
    <div class="card"><h4>Real vaxt</h4><p class="muted small">WebSocket üzərindən aşağı gecikmə ilə əmrlər; status 0.6 saniyədən bir yenilənir.</p></div>
    <div class="card"><h4>Təhlükəsizlik</h4><p class="muted small">Açar/şifrə yoxlaması, səhv cəhd limiti, icazəli yollar siyahısı, sorğu sürəti limiti, ölü-adam dayanması.</p></div>
  </div>

  <h2>Texnologiya</h2>
  <ul>
    <li>Server: Node.js + <code>ws</code> (bir asılılıq), Railway-də işləyir.</li>
    <li>Sayt: build addımı olmayan vanilla JavaScript SPA (Ana səhifə, Robotlar, İdarəetmə, Bələdçi, Haqqında).</li>
    <li>Robot: ESP32 + SMARS şassisi, Wi-Fi üzərindən TLS WebSocket (<code>wss://</code>).</li>
  </ul>

  <h2>Təhlükəsizlik qeydləri</h2>
  <ul>
    <li>Əl rejimində robot 0.7 san ərzində yeni əmr almasa dayanır; tab gizlənəndə və ya pəncərədən çıxanda sayt STOP göndərir.</li>
    <li>Wi-Fi, cloud ünvanı, açar və zavod sıfırlaması saytdan dəyişdirilə bilməz — yalnız robotun yerli panelindən.</li>
    <li>Söndürülmüş rejimə keçid serverdə rədd edilir (403).</li>
  </ul>
</div>`,
    };
  }

  function NotFoundPage() {
    return {
      title: 'Tapılmadı',
      html: `<div class="empty" style="margin-top:40px"><div class="big">🧭</div><h2 style="font-family:var(--disp)">404</h2>
        <p style="margin:8px 0 16px">Bu səhifə tapılmadı.</p><a class="btn" href="/" data-link>Ana səhifəyə qayıt</a></div>`,
    };
  }

  // ================================================================ boot
  $('#year').textContent = new Date().getFullYear();
  $('#burger').addEventListener('click', () => $('#links').classList.toggle('open'));
  $('#logoutBtn').addEventListener('click', () => { Hub.logout(); if (curProtected) render(); });
  window.addEventListener('unhandledrejection', (e) => e.preventDefault());
  Hub.start();
  render();
})();
