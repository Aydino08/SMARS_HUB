#!/usr/bin/env node
'use strict';
/*
 * Dev-only: saytın frontend-ini (public/app.js) jsdom-da real server + virtual robotlarla işlədir.
 *   cd <jsdom olan qovluq> && npm i jsdom ws
 *   NODE_PATH=<o qovluq>/node_modules node tools/dom-check.js
 */
const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs');
const { JSDOM, VirtualConsole } = require('jsdom');

const PORT = 3991;
const KEY = 'test-robot-key-123';
const PW = 'parol123';
const root = path.join(__dirname, '..');
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const procs = [];
const errors = [];
let failed = 0;
const ok = (c, msg) => { console.log((c ? '  ✓ ' : '  ✗ ') + msg); if (!c) failed++; };

// robotun real vəziyyətini UI-dan asılı olmadan, ayrıca WebSocket ilə soruş
function robotMode(rid) {
  const WebSocket = require('ws');
  return new Promise((res) => {
    const s = new WebSocket(`ws://127.0.0.1:${PORT}/ws/ui`);
    s.on('open', () => s.send(JSON.stringify({ t: 'auth', pw: PW })));
    s.on('message', (d) => {
      const m = JSON.parse(d.toString());
      if (m.t === 'auth' && m.ok) s.send(JSON.stringify({ t: 'req', id: 1, r: rid, p: '/status' }));
      if (m.t === 'res') { s.close(); res(typeof m.d === 'string' ? JSON.parse(m.d).mode : m.d.mode); }
    });
    setTimeout(() => res(null), 4000);
  });
}

(async () => {
  const dataDir = fs.mkdtempSync(path.join(process.env.TMPDIR || '/tmp', 'hubdom-'));
  procs.push(spawn(process.execPath, ['server.js'], { cwd: root, env: { ...process.env, PORT: String(PORT), ROBOT_KEY: KEY, UI_PASSWORD: PW, DATA_DIR: dataDir }, stdio: 'ignore' }));
  await sleep(600);
  procs.push(spawn(process.execPath, ['tools/sim-robot.js', '--url', `ws://127.0.0.1:${PORT}`, '--key', KEY, '--count', '3', '--name', 'SMARS'], { cwd: root, stdio: 'ignore' }));
  await sleep(1500);

  const vc = new VirtualConsole();
  vc.on('jsdomError', (e) => { if (!/Could not load link|stylesheet|fonts\.googleapis/i.test(e.message)) errors.push('jsdomError: ' + e.message); });
  vc.on('error', (...a) => errors.push('console.error: ' + a.join(' ')));

  const base = `http://127.0.0.1:${PORT}`;
  const dom = await JSDOM.fromURL(base + '/', {
    runScripts: 'dangerously', resources: 'usable', pretendToBeVisual: true, virtualConsole: vc,
    beforeParse(w) {
      w.CSS = w.CSS || {}; w.CSS.escape = w.CSS.escape || ((s) => String(s).replace(/[^a-zA-Z0-9_-]/g, (c) => '\\' + c));
      w.scrollTo = () => {};
      // jsdom-da fetch yoxdur — Node-un fetch-i ilə əvəz et (yalnız test üçün)
      w.fetch = (u, o) => fetch(new URL(u, base), o);
      w.confirm = () => true;
      w.PointerEvent = w.PointerEvent || w.MouseEvent;
      w.HTMLFormElement.prototype.requestSubmit = w.HTMLFormElement.prototype.requestSubmit || function () { this.dispatchEvent(new w.Event('submit', { cancelable: true })); };
    },
  });
  const w = dom.window, d = w.document;
  const $ = (s) => d.querySelector(s);
  const $$ = (s) => [...d.querySelectorAll(s)];
  const waitFor = async (fn, ms = 5000) => { const t0 = Date.now(); while (Date.now() - t0 < ms) { try { if (fn()) return true; } catch (_) { /* */ } await sleep(80); } return false; };
  const click = (el) => el.dispatchEvent(new w.MouseEvent('click', { bubbles: true, cancelable: true }));
  const navTo = (href) => { const a = d.createElement('a'); a.href = href; a.setAttribute('data-link', ''); d.body.appendChild(a); click(a); a.remove(); };

  console.log('Ana səhifə');
  await waitFor(() => $('#app .hero'));
  ok(!!$('.hero h1'), 'hero render olundu');
  ok(await waitFor(() => $('#stOnline').textContent === '3'), 'onlayn robot sayı (/api/public) = ' + ($('#stOnline') || {}).textContent);
  ok($$('.links a[data-nav]').length === 5, 'nav linkləri (Simulyasiya daxil)');

  console.log('Robotlar (giriş)');
  navTo('/robots');
  ok(await waitFor(() => $('#loginForm')), 'qorunan səhifə giriş formasını göstərir');
  $('#pwIn').value = 'səhv-şifrə';
  $('#loginForm').requestSubmit();
  ok(await waitFor(() => /yanlış/i.test($('#loginMsg').textContent)), 'səhv şifrə mesajı: ' + $('#loginMsg').textContent);
  $('#pwIn').value = PW;
  $('#loginForm').requestSubmit();
  ok(await waitFor(() => $$('.rcard').length === 3), 'giriş → 3 robot kartı');
  ok(/3 robot onlayn/.test($('#connPill').textContent), 'nav pill: ' + $('#connPill').textContent);
  ok(!$('#logoutBtn').hidden, 'çıxış düyməsi görünür');
  ok(w.localStorage.getItem('smarsPw') === PW, 'şifrə yadda saxlanıldı');
  const q = $('#q'); q.value = 'sim-2'; q.dispatchEvent(new w.Event('input', { bubbles: true }));
  ok($$('.rcard').length === 1, 'axtarış filtri');
  q.value = ''; q.dispatchEvent(new w.Event('input', { bubbles: true }));
  ok(await waitFor(() => $$('.rcard .pill.mode').every((p) => p.textContent !== '…')), 'kartlarda rejim göstərilir');

  console.log('İdarəetmə');
  click($$('.rcard').find((c) => c.getAttribute('href') === '/robot/smars-sim-1'));
  ok(await waitFor(() => w.location.pathname === '/robot/smars-sim-1' && $('#modeGrid')), 'robot səhifəsi açıldı');
  ok(await waitFor(() => $('#cOn').textContent.includes('ONLAYN')), 'onlayn pill');
  ok(await waitFor(() => $$('[data-mode]').length === 4), '4 rejim düyməsi');
  ok(await waitFor(() => $$('[data-group]').length === 4), 'robot ayarları yükləndi (4 qrup)');
  ok(await waitFor(() => /\d+/.test($('#distBig').textContent)), 'məsafə canlı: ' + $('#distBig').textContent.trim());
  ok(await waitFor(() => $$('#irRows .ir-row').length === 3), 'IR sətirləri');
  const F = $('[data-dir="F"]');
  F.dispatchEvent(new w.PointerEvent('pointerdown', { bubbles: true, pointerId: 1 }));
  ok(await waitFor(() => $('#tdir').textContent === 'İRƏLİ', 2000), 'F basılı → İRƏLİ');
  await sleep(900);
  ok($('#tdir').textContent === 'İRƏLİ', 'basılı saxlananda robot dayanmır (təkrar göndərilir)');
  F.dispatchEvent(new w.PointerEvent('pointerup', { bubbles: true, pointerId: 1 }));
  ok(await waitFor(() => $('#tdir').textContent === 'STOP', 2000), 'buraxanda STOP');
  d.dispatchEvent(new w.KeyboardEvent('keydown', { key: 'ArrowLeft', bubbles: true }));
  ok(await waitFor(() => $('#tdir').textContent === 'SOL', 2000), 'klaviatura ← → SOL');
  d.dispatchEvent(new w.KeyboardEvent('keyup', { key: 'ArrowLeft', bubbles: true }));
  ok(await waitFor(() => $('#tdir').textContent === 'STOP', 2000), 'klaviatura buraxanda STOP');
  click($('[data-mode="2"]'));
  ok(await waitFor(() => $('#modeLabel').textContent === 'Line following' && !$('#autoBar').hidden), 'line following rejimi, avto panel görünür');
  ok($('#dpad').classList.contains('disabled'), 'avto rejimdə D-pad bağlıdır');
  const slider = $('[data-sk="obsDist"][data-kind="r"]');
  slider.value = '40'; slider.dispatchEvent(new w.Event('input', { bubbles: true }));
  ok(await waitFor(() => $('[data-sk="obsDist"][data-kind="n"]').value === '40'), 'robot ayarı slider → rəqəm sahəsi');

  console.log('Fərdiləşdirmə');
  click($('[data-tab="custom"]'));
  ok(!$('[data-pane="custom"]').hidden && $('[data-pane="control"]').hidden, 'fərdiləşdirmə tabı');
  ok($('[data-pmode="manual"]').disabled, 'əl idarəsi həmişə açıq (söndürmək olmur)');
  ok(await waitFor(() => $$('[data-pgroup]').length === 4), 'ayar qrupu açarları');
  const ch = (sel, val) => { const el = $(sel); el.checked = val; el.dispatchEvent(new w.Event('change', { bubbles: true })); };
  ch('[data-pmode="line"]', false);
  ch('[data-pmod="servo"]', false);
  ch('[data-pmod="ir"]', false);
  ch('[data-pgroup="line"]', false);
  const n = $('#pName'); n.value = 'Kürək SMARS'; n.dispatchEvent(new w.Event('input', { bubbles: true }));
  click($('[data-color="#ff9f00"]'));
  ok(/saxlanmamış/.test($('#saveState').textContent), 'dəyişikliklər "saxlanmamış" kimi işarələnir');
  click($('#pSave'));
  ok(await waitFor(() => $('#cName').textContent === 'Kürək SMARS'), 'ad başlıqda yeniləndi');
  ok(await waitFor(() => $('#modeLabel').textContent === 'Əl idarəsi'), 'söndürülmüş rejimdəki robot əl idarəsinə qaytarıldı');
  ok(!$$('[data-mode]').some((b) => b.dataset.mode === '2'), 'line following düyməsi itdi');
  ok($('#cardServo').hidden && $('#cardIr').hidden && !$('#cardDist').hidden, 'servo + IR gizləndi, məsafə görünür');
  ok($('[data-group="line"]').hidden && !$('[data-group="obs"]').hidden, 'line ayar qrupu gizləndi');
  await sleep(600);
  const realMode = await robotMode('smars-sim-1');
  ok(realMode === 0, 'robotun özü də əl idarəsinə keçdi (status.mode=' + realMode + ')');
  click($('[data-preset="dozer"]'));
  ok($('[data-pmode="arena"]').checked && !$('[data-pmod="ir"]').checked, 'şablon formu doldurdu');
  click($('#pReset'));
  ok(!$('[data-pmode="line"]').checked, 'ləğv et → saxlanmış profilə qayıdır');

  console.log('Siyahı / çıxış');
  click($('a[href="/robots"][data-link]'));
  ok(await waitFor(() => $$('.rcard').length === 3), 'robotlar səhifəsinə qayıdış');
  ok(await waitFor(() => ($('.rcard[href="/robot/smars-sim-1"] .pill.mode') || {}).textContent === 'Əl idarəsi', 1200),
    'kartdakı rejim 1.2 san ərzində "Əl idarəsi"-nə yeniləndi');
  const card = $('.rcard[href="/robot/smars-sim-1"]');
  const chips = card ? [...card.querySelectorAll('.chip')].map((c) => c.textContent) : [];
  ok(card && card.textContent.includes('Kürək SMARS') && !chips.includes('Line following'),
    'kartda yeni ad və rejimlər: ' + (card ? card.querySelector('.rname').textContent + ' · ' + chips.join(', ') + ' · pill=' + (card.querySelector('.pill.mode') || {}).textContent : '?'));
  navTo('/guide'); ok(await waitFor(() => /Qoşulma bələdçisi/.test($('#app').textContent)), 'bələdçi səhifəsi');
  navTo('/about'); ok(await waitFor(() => /SMARS nədir/.test($('#app').textContent)), 'haqqında səhifəsi');
  navTo('/blabla'); ok(await waitFor(() => /404/.test($('#app').textContent)), '404 səhifəsi');
  navTo('/robots');
  click($('#logoutBtn'));
  ok(await waitFor(() => $('#loginForm')), 'çıxış → giriş forması');
  ok(!w.localStorage.getItem('smarsPw'), 'yadda saxlanan şifrə silindi');

  console.log('Simulyasiya səhifəsi (girişsiz)');
  {
    // sim3d.js Node-da headless işləyir (jsdom WebGL/ES modul dəstəkləmir); səhifənin qalanı realdır
    const nodeFetch = globalThis.fetch;
    globalThis.fetch = (u, o) => nodeFetch(new URL(u, base), o);
    globalThis.localStorage = w.localStorage;
    const { createCanvas } = require('@napi-rs/canvas');
    const mod = await import(path.join(root, 'public', 'sim3d.js'));
    const hook = { load: async () => mod, opts: { headless: true, createCanvas } };
    w.__smarsSimTest = hook;
    navTo('/sim');
    ok(await waitFor(() => hook.instance && !$('#simLoading'), 8000), '3D simulyator yükləndi (CAD modelləri: smars.bin)');
    const sim = hook.instance;
    // real tempdə: panel əmri real vaxtla 150 ms-dən bir təkrarlayır, ölü-adam taymeri isə sim vaxtı ilə işləyir
    const run = async (sec) => { for (let t = 0; t < sec; t += 0.05) { sim.step(0.05); await sleep(50); } };
    ok(await waitFor(() => $('#cOn').textContent.includes('SİMULYASİYA')), 'panel simulyasiyaya qoşuldu');
    ok(await waitFor(() => $$('[data-mode]').length === 4), '4 rejim (konfiqurasiya A)');
    ok(await waitFor(() => $$('[data-group]').length === 6), 'simulyasiya ayarları yükləndi (6 qrup)');
    ok(await waitFor(() => $$('#irRows .ir-row').length === 3), 'IR sensorlar göstərilir');
    const p0 = sim.state();
    const F = $('[data-dir="F"]');
    F.dispatchEvent(new w.PointerEvent('pointerdown', { bubbles: true, pointerId: 2 }));
    await run(1.2);
    ok(await waitFor(() => $('#tdir').textContent === 'İRƏLİ', 2000), 'D-pad F → İRƏLİ');
    F.dispatchEvent(new w.PointerEvent('pointerup', { bubbles: true, pointerId: 2 }));
    const p1 = sim.state();
    ok(Math.hypot(p1.x - p0.x, p1.z - p0.z) > 150, `robot 3D səhnədə irəli getdi (${Math.hypot(p1.x - p0.x, p1.z - p0.z).toFixed(0)} mm)`);
    await run(1);
    ok(await waitFor(() => $('#tdir').textContent === 'STOP', 2000), 'buraxanda STOP');
    click($('[data-mode="2"]'));
    await run(3);
    ok(await waitFor(() => /XƏTT/.test($('#autoStat').textContent), 3000), 'line following: ' + $('#autoStat').textContent);
    ok(await waitFor(() => $$('#irRows .ir-tag.black').length >= 1, 3000), 'IR sensor qara xətti görür');
    click($('[data-mode="0"]'));
    ok(await waitFor(() => /\d/.test($('#simStats').textContent)), 'HUD statistikası: ' + $('#simStats').textContent.slice(0, 50));
    // fərdiləşdirmə simulyasiyada da işləyir
    click($('[data-tab="custom"]'));
    const ch = (sel, val) => { const el = $(sel); el.checked = val; el.dispatchEvent(new w.Event('change', { bubbles: true })); };
    ch('[data-pmode="arena"]', false);
    click($('#pSave'));
    click($('[data-tab="control"]'));
    ok(await waitFor(() => !$$('[data-mode]').some((b) => b.dataset.mode === '3')), 'fərdiləşdirmə: arena düyməsi gizləndi');
    // konfiqurasiya B (IR-sız) + arena səhnəsi
    const cfg = $('#simCfg'); cfg.value = 'B'; cfg.dispatchEvent(new w.Event('change', { bubbles: true }));
    ok(await waitFor(() => $('#cardIr').hidden && !$$('[data-mode]').some((b) => b.dataset.mode === '2')), 'konfiqurasiya B: IR paneli və line following yoxdur');
    const sc = $('#simScene'); sc.value = 'arena'; sc.dispatchEvent(new w.Event('change', { bubbles: true }));
    await run(0.5);
    ok(await waitFor(() => /kub çıxarılıb: 0\/6/.test($('#simStats').textContent), 2000), 'arena səhnəsi: 6 kub');
    sc.value = 'line'; sc.dispatchEvent(new w.Event('change', { bubbles: true }));
    cfg.value = 'A'; cfg.dispatchEvent(new w.Event('change', { bubbles: true }));
    navTo('/');
    ok(await waitFor(() => $('.hero')), 'simulyasiyadan çıxış (destroy) xətasızdır');
    globalThis.fetch = nodeFetch;
  }

  ok(errors.length === 0, 'JS xətası yoxdur' + (errors.length ? ': ' + errors.join(' | ') : ''));
  dom.window.close();
  procs.forEach((p) => p.kill());
  console.log(failed ? `\n${failed} yoxlama UĞURSUZ` : '\nBütün frontend yoxlamaları keçdi ✓');
  process.exit(failed ? 1 : 0);
})().catch((e) => { console.error(e); procs.forEach((p) => p.kill()); process.exit(1); });
