# SMARS Control Hub

Bir neçə SMARS robotunu (ESP32) internet üzərindən idarə etmək üçün sayt. Railway-də işləyir.

```
Brauzer (sayt)  <--WSS-->  SMARS Hub (Railway)  <--WSS-->  SMARS robot × N
```

Hər robot serverə özü qoşulur (çıxan bağlantı), ona görə router-də port açmaq lazım deyil.

https://smarshub-production.up.railway.app sayta bu linkdən baxa bilərsiniz. 

## Saytda nələr var

| Səhifə | Nə edir |
|---|---|
| **Ana səhifə** `/` | Layihə haqqında, onlayn robot sayı, necə işləyir |
| **Robotlar** `/robots` | WebSocket ilə qoşulan bütün SMARS-ların canlı siyahısı. Hər kartda rejim, məsafə, Wi-Fi, gecikmə və neçə nəfərin baxdığı görünür. Axtarış və "yalnız onlayn" filtri var |
| **İdarəetmə** `/robot/<id>` | Seçilmiş robotu idarə etmək üçün səhifə (aşağıda ətraflı) |
| **Simulyasiya** `/sim` | Brauzerdə 3D SMARS: real robotun bütün funksiyaları, giriş tələb etmir (aşağıda ətraflı) |
| **Qoşulma** `/guide` | Railway deploy, firmware qoşulması, protokol, problemlər |
| **Haqqında** `/about` | SMARS və platforma haqqında |

İdarəetmə səhifəsində:

* **İdarəetmə tabı:**
  * sürüş rejimləri;
  * D-pad, klaviatura (↑↓←→ / WASD, boşluq = STOP, Esc = əl idarəsi) və sürət;
  * HC-SR04 məsafəsi, IR sensorlar, servo;
  * robotun öz ayarları.
* **Fərdiləşdirmə tabı:** hər robot üçün ayrıca:
  * görünən ad və rəng;
  * **rejimləri aç-söndür:** maneədən qaçma, line following, arena;
  * **panelləri aç-söndür:** IR, ultrasəs, servo, ayarlar;
  * robot ayar qruplarını gizlətmək;
  * hazır şablonlar: "Yalnız əl idarəsi", "Kürək + HC-SR04", "Line follower".

  Fərdiləşdirmə serverdə saxlanılır və bu robota baxan hər kəsə tətbiq olunur. Söndürülmüş rejim
  **serverdə də bloklanır** (403). Robot həmin vaxt o rejimdə işləyirsə, avtomatik əl idarəsinə
  (STOP) keçirilir.
* **Məlumat tabı:** firmware, nə vaxtdan qoşulub, gecikmə, Wi-Fi. Oflayn robotu siyahıdan silmək
  də buradandır.

## 3D simulyasiya (`/sim`)

Robot modeli SMARS CAD fayllarından gəlir: `chassis_SL`, master/slave təkərlər, `mechanical_track`
halqaları, TCRT5000 kaseti, kürək və orijinal ultrasəs korpusu. Fayllar `public/models/smars.bin`
və `smars.json`-dadır. Arduino lövhəsi və servo qolu sxematikdir, çünki onlar CAD-də yoxdur.

Simulyasiya **real robotla eyni protokolla** işləyən virtual robotdur
(`/status /mode /tank /servo /settings /set /reset`). Ona görə idarə paneli dəyişmədən ona qoşulur:
D-pad, klaviatura, sürət, rejimlər, HC-SR04, IR, servo, ayarlar və fərdiləşdirmə hamısı eyni işləyir.

| Nə | Necə simulyasiya olunur |
|---|---|
| Sürmə | Tank (differensial) kinematikası. Tırtıl halqaları və təkərlər hərəkətə uyğun fırlanır. Əl rejimində 0.7 san əmr gəlməsə robot dayanır (ölü-adam dayanması) |
| HC-SR04 | ±15° konus, 2–400 sm, ~16 Hz. Obyektin hündürlüyü nəzərə alınır. 2 sm-dən yaxın kor zona var (999) |
| TCRT5000 ×3 | Hər sensor yerin rəngini öz CAD mövqeyindən (±16.1 mm, 68 mm irəlidə) oxuyur, faiz və qara/ağ olaraq |
| Toqquşma | Gövdə, tırtıllar və U-kürəyin qanadları divar və maneələrə dəyir. Kublar kürəklə itələnir |
| Rejimlər | Əl idarəsi; maneədən qaçma (geri çəkil + dön, bamper kimi ilişmə); line following (3 sensor); arena (kubu tap → tuşla → itələ, kənarı IR ilə gör) |

Səhnələr: **Xətt trası**, **Maneələr**, **Arena (kub çıxar)**.

Konfiqurasiyalar:
* **A:** TCRT5000 ×3 + kürək + HC-SR04.
* **B:** kürək + HC-SR04, IR sensoru yoxdur. Bu konfiqurasiyada line following olmur (409).

Kamera rejimləri: izlə, arxadan, yuxarıdan, sensor gözü. Ayarlar və fərdiləşdirmə brauzerin
`localStorage`-ında saxlanılır.

Modelləri CAD-dən yenidən yaratmaq üçün (`cad-tools/`):

```bash
pip install -r cad-tools/requirements.txt
python cad-tools/extract_3mf.py /yol/SMARS.3mf
# (make_combo3.py, make_kurek4.py — kürək hissələri), sonra:
python cad-tools/export_web_models.py public/models
```

## Fayllar

| Fayl | Nə üçündür |
|---|---|
| `server.js` | HTTP + WebSocket relay, robot siyahısı, profillər (asılılıq: yalnız `ws`) |
| `public/index.html`, `app.css`, `app.js` | Sayt (build addımı yoxdur, vanilla JS SPA) |
| `railway.json` | Railway: start əmri, `/healthz` healthcheck, restart siyasəti |
| `tools/sim-robot.js` | Virtual SMARS robot(lar): real robotsuz sınaq üçün |
| `tools/smoke-test.js` | `npm test`: server + robotlar + UI üçün 36 avtomatik yoxlama |
| `public/sim3d.js` | 3D simulyator: səhnə, fizika, sensorlar, virtual firmware |
| `public/models/` | SMARS CAD modelləri (sadələşdirilmiş) + yerləşmə məlumatı |
| `public/vendor/three/` | three.js r187 + OrbitControls (MIT, `LICENSE`); CDN yoxdur, CSP buna görə sərtdir |
| `cad-tools/` | Python CAD skriptləri: 3MF → hissələr → kürək/kaset → veb modelləri |
| `tools/dom-check.js` | (istəyə bağlı) sayt və simulyasiya səhifəsi üçün jsdom testi, 62 yoxlama. `jsdom` və `@napi-rs/canvas` tələb edir |
| `tools/sim-check.mjs` | (istəyə bağlı) simulyatorun headless davranış testi: sürmə, toqquşma, line following, maneədən qaçma, arena. `@napi-rs/canvas` tələb edir |

## Railway-ə deploy

1. Bu qovluğu GitHub repozitoriyasına yüklə. `node_modules/` və `data/` yüklənmir (`.gitignore`).
2. [railway.com](https://railway.com) → **New Project → Deploy from GitHub repo** → repozitoriyanı seç.
3. Servis → **Variables**:

   | Ad | Məcburi | Dəyər |
   |---|---|---|
   | `ROBOT_KEY` | bəli | robotların gizli açarı, **min. 12** simvol (`openssl rand -hex 16`) |
   | `UI_PASSWORD` | bəli | sayta giriş şifrəsi, **min. 6** simvol |
   | `DATA_DIR` | tövsiyə | `/data` (4-cü addımdakı Volume ilə) |

4. **Fərdiləşdirmələr itməsin deyə:** servisə **Volume** əlavə et (mount path `/data`) və
   `DATA_DIR=/data` yaz. Volume olmasa, profillər hər redeploy-da sıfırlanır.
5. **Settings → Networking → Generate Domain**.
6. Yoxla: `https://<domen>/healthz` → `{"ok":true,"configured":true}`.

CLI ilə: `npm i -g @railway/cli && railway login && railway init && railway up && railway domain`.

Köhnə serverlə **eyni** `ROBOT_KEY` və `UI_PASSWORD` dəyərlərini istifadə etsən, mövcud robot
dəyişiklik etmədən qoşulacaq.

## Robotu qoşmaq (ESP32)

Protokol əvvəlki serverlə **eynidir**: `R|gid|/yol?qs` → `{"t":"res","id":gid,"c":200,"d":…}`.
Yeganə yenilik odur ki, hər robota **ID** vermək lazımdır. Bunu yolda edə bilərsən:

```cpp
webSocket.setExtraHeaders("Authorization: Bearer " CLOUD_KEY);
webSocket.beginSSL(CLOUD_HOST, 443, "/ws/robot?id=smars-01&name=Kurek%20SMARS");
```

və ya `hello` mesajında:

```json
{"t":"hello","fw":"2.1.0","id":"smars-01","name":"Kürək SMARS"}
```

* ID: hərf, rəqəm, `-`, `_`, maksimum 32 simvol. Hər robotun ID-si fərqli olmalıdır. Eyni ID ilə
  ikinci robot qoşulsa, birinci bağlantı bağlanır.
* ID göndərməyən köhnə firmware `smars-1` adı ilə qeydə alınır. Belə robotdan eyni anda yalnız biri
  işləyə bilər.
* Server hər robotdan 4 saniyədən bir `/status` soruşur ki, siyahıda rejim, məsafə və Wi-Fi görünsün.
* Saytın göndərə biləcəyi yollar: `/tank /mode /servo /status /settings /set /reset`. Wi-Fi, açar,
  reboot və zavod sıfırlaması saytdan bağlıdır.

Rejim nömrələri: `0` əl idarəsi · `1` maneədən qaçma · `2` line following · `3` arena.

## Lokal sınaq

```bash
npm install
ROBOT_KEY=test-robot-key-123 UI_PASSWORD=parol123 npm start
# ikinci terminal — 3 virtual SMARS:
ROBOT_KEY=test-robot-key-123 npm run sim -- --count 3
# brauzer: http://localhost:3000  (şifrə: parol123)

npm test        # avtomatik yoxlamalar
```

Virtual robotu Railway-dəki serverə qoşmaq:
`node tools/sim-robot.js --url wss://<domen> --key <ROBOT_KEY> --id test-01 --name Test`

## Təhlükəsizlik

* Robotlar `ROBOT_KEY`, sayt `UI_PASSWORD` ilə qorunur. Səhv cəhdlər IP üzrə məhdudlaşdırılır:
  10 dəqiqədə 5 səhv olarsa, blok düşür.
* Sorğu sürəti limiti, icazəli yollar siyahısı, query simvol yoxlaması, sərt CSP
  (`script-src 'self'`, inline JS yoxdur).
* **Ölü-adam dayanması:**
  * əl rejimində düymə basılı olduqca əmr 150 ms-dən bir təkrarlanır;
  * robot 0.7 san ərzində əmr almasa dayanır;
  * tab gizlənəndə və ya pəncərə fokusu itəndə sayt STOP göndərir.

## Problemlər

| Əlamət | Həll |
|---|---|
| Girişdə "Server tənzimlənməyib" | `ROBOT_KEY` (≥12) / `UI_PASSWORD` (≥6) Variables-də yoxdur |
| Robot siyahıda görünmür | Wi-Fi 2.4 GHz olmalıdır; host-a `https://` yazma; açar eyni olmalıdır; Railway → Logs-da `[robot] RED` axtar |
| İki robot bir-birini "atır" | Hər robota ayrı `id` ver |
| Fərdiləşdirmə redeploy-dan sonra itir | Volume + `DATA_DIR=/data` |
| "Rejim dəyişmədi: bu robotda söndürülüb" | Həmin rejim Fərdiləşdirmə tabında bağlıdır |
| Deploy uğursuz | Railway → Deployments → Logs. Node ≥ 18 lazımdır (Railpack özü seçir) |
