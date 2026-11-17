"""
SMARS şarj sistemi
  * Robot tərəfi: arxa extension yuvasına taxılan "şarj quyruğu" (shoe) + qapaq
      - quyruğun iki yan üzündə mis lent kontaktları (+ / −)
      - içində IP2326 (5V → 2S 8.4V) şarj modulu üçün bölmə
  * Stansiya: "qaraj" — huni (funnel), iki yaylı plunjer kontakt (M3 vint + yay), dayaq divarı,
    güc açarı kimi mikrovaç, 2 LED, USB kabel girişi; alt qapaq; lent yerləşdirmə şablonu

Koordinatlar: "F çərçivəsi" = SMARS chassis_SL çərçivəsi 180° çevrilmiş (quyruq +Y tərəfdə).
Real robotda quyruq ARXA yuvadadır: chassis = F-in Z ətrafında 180° fırlanmışı.
mm; z = 0 şassi mərkəzi; yer (robot ən aşağı vəziyyətdə) z = −28.75.
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass, asdict
from pathlib import Path

import manifold3d as mf
import numpy as np
import trimesh
from shapely.geometry import LineString, Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import (box, hull, vcyl, yz_prism, to_trimesh, from_trimesh, build_module,  # noqa: E402
                        GROUND_Z_LOW, GROUND_Z_HIGH, WALL_Y0, WALL_Y1, WALL_TOP_Z,
                        FR_X, FR_Y0, FR_Y1, FR_Z0, FR_Z1, NOTCH_X, NOTCH_Y0, NOTCH_Y1)

ROOT = Path(__file__).resolve().parent.parent
ADD = mf.OpType.Add
ZF = GROUND_Z_LOW                      # yer səviyyəsi (robot ən aşağı)
BOB = GROUND_Z_LOW - GROUND_Z_HIGH     # səkkizbucaqlı təkər: robot 1.17 mm qalxıb-enir
SQ2 = 2 ** 0.5


# ================================================================== parameters
@dataclass
class ShoeP:
    tw2: float = 22.0                  # quyruğun yarı eni (tam 44 mm)
    arm_y0: float = WALL_Y1 + 0.25     # 35.25
    arm_t: float = 3.35
    ty1: float = 84.0                  # quyruğun ucu
    tz0: float = ZF + 10.0             # quyruğun altı yerdən 10 mm yuxarıda
    tz1: float = 4.0                   # bölmənin üst kənarı (qapaq bunun üstündə)
    wall: float = 1.6
    lid_t: float = 1.6
    end_ch: float = 6.0                # ucdakı şaquli tilləri 45° faska (plunjerləri itələyir)
    tape_w: float = 10.0               # mis lent eni
    tape_z: float = -6.0               # lent (kontakt) mərkəzinin hündürlüyü
    tape_y0: float = 47.6
    tape_y1: float = 77.8
    slot_y: tuple = (46.4, 47.6)       # lentin bölməyə keçdiyi yarıq
    boss_xy: tuple = ((19.2, 41.6), (19.2, 74.0))
    seat_y: float = 33.8
    rib: float = 0.20                  # arxa divara söykənən kontakt qabırğası
    rib_z: tuple = (-10.0, -8.0)

    @property
    def arm_y1(self):
        return self.arm_y0 + self.arm_t


@dataclass
class StationP:
    y0: float = 49.0                   # ön üz (robot tərəfi) — tırtılların ucu 47-dədir
    y1: float = 126.0
    hw: float = 50.0
    cover_t: float = 2.0
    wall: float = 2.4
    roof_t: float = 2.4
    roof_h: float = 40.0               # damın alt üzü yerdən 40 mm
    c: float = 1.25                    # boğazda quyruq ↔ divar boşluğu (hər tərəf)
    funnel_open: float = 18.0          # huninin ağzında əlavə yarı en
    funnel_y: float = 65.0
    back_y: float = 84.3               # dayaq divarının üzü (quyruğun ucu 84.0)
    back_t: float = 3.0
    py: float = 72.0                   # plunjerin Y mövqeyi
    pz: float = -6.0                   # plunjerin hündürlüyü (= lentin mərkəzi)
    bore_d: float = 6.6
    bore_depth: float = 9.0
    bore_wall: float = 1.6
    hole_d: float = 3.4
    boss_d: float = 12.0
    head_free_protrusion: float = 3.75  # sərbəst plunjer başlığının boğaza çıxıntısı

    @property
    def zf(self):
        return ZF

    @property
    def th(self):                      # boğazın daxili yarı eni
        return ShoeP().tw2 + self.c

    @property
    def mouth(self):
        return self.th + self.funnel_open

    @property
    def roof_under(self):
        return ZF + self.roof_h


SP, TP = ShoeP(), StationP()


# ================================================================== helpers
def ccw(pts):
    a = np.asarray(pts, float)
    x, y = a[:, 0], a[:, 1]
    return a if 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) > 0 else a[::-1].copy()


def ext_xy(poly, z0, z1):
    if isinstance(poly, Polygon):
        poly = list(poly.exterior.coords)[:-1]
    cs = mf.CrossSection([ccw(poly)])
    return cs.extrude(z1 - z0).translate([0, 0, z0])


def cyl_x(x0, x1, y, z, r, seg=40):
    return mf.Manifold.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


def cyl_y(y0, y1, x, z, r, seg=40):
    return mf.Manifold.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])


def tear_x(x0, x1, y, z, r, seg=32):
    """x oxlu dəlik; uc (−z) tərəfə — stansiya başıaşağı çap olunanda yuxarı baxır."""
    pts = []
    for x in (x0, x1):
        for a in np.linspace(0, 2 * np.pi, seg, endpoint=False):
            pts.append((x, y + r * np.cos(a), z + r * np.sin(a)))
        pts.append((x, y, z - r * SQ2))
    return hull(pts)


def tear_y(y0, y1, x, z, r, seg=32):
    pts = []
    for y in (y0, y1):
        for a in np.linspace(0, 2 * np.pi, seg, endpoint=False):
            pts.append((x + r * np.cos(a), y, z + r * np.sin(a)))
        pts.append((x, y, z - r * SQ2))
    return hull(pts)


def hex_x(x0, x1, y, z, s=5.5):
    R = s / np.sqrt(3)
    pts = [(x, y + R * np.cos(a), z + R * np.sin(a)) for x in (x0, x1) for a in np.radians(np.arange(0, 360, 60) + 30)]
    return hull(pts)


def cone_z(x, y, z0, z1, r0, r1, seg=40):
    return mf.Manifold.cylinder(z1 - z0, r0, r1, seg).translate([x, y, z0])


def union(ms):
    ms = [m for m in ms if m is not None]
    return mf.Manifold.batch_boolean(ms, ADD) if ms else mf.Manifold()


def rot180(m: mf.Manifold) -> mf.Manifold:
    return m.rotate([0, 0, 180])


def plus_sign(cx, cy, size, z0, z1):
    a, b = size / 2, size / 7
    return box(cx - a, cx + a, cy - b, cy + b, z0, z1) + box(cx - b, cx + b, cy - a, cy + a, z0, z1)


def minus_sign(cx, cy, size, z0, z1):
    a, b = size / 2, size / 7
    return box(cx - a, cx + a, cy - b, cy + b, z0, z1)


def plus_on_xface(x0, x1, cy, cz, size):          # yan üzdə (x normallı) "+" oyuğu
    a, b = size / 2, size / 7
    return box(x0, x1, cy - a, cy + a, cz - b, cz + b) + box(x0, x1, cy - b, cy + b, cz - a, cz + a)


def minus_on_xface(x0, x1, cy, cz, size):
    a, b = size / 2, size / 7
    return box(x0, x1, cy - a, cy + a, cz - b, cz + b)


def bolt_poly(cx, cy, s):
    """ildırım işarəsi (xy)"""
    p = [(0.15, 1.0), (-0.45, -0.05), (-0.05, -0.05), (-0.2, -1.0), (0.45, 0.15), (0.05, 0.15), (0.35, 1.0)]
    return [(cx + x * s, cy + y * s) for x, y in p]


# ================================================================== robot: şarj quyruğu
def tail_outline(y_from=None):
    t = SP
    y0 = t.arm_y0 if y_from is None else y_from
    return Polygon([(-t.tw2, y0), (t.tw2, y0), (t.tw2, t.ty1 - t.end_ch), (t.tw2 - t.end_ch, t.ty1),
                    (-t.tw2 + t.end_ch, t.ty1), (-t.tw2, t.ty1 - t.end_ch)])


def build_shoe() -> mf.Manifold:
    t = SP
    solid = [
        box(-FR_X, FR_X, FR_Y0, FR_Y1, FR_Z0, FR_Z1),                       # connector_a ilə eyni çərçivə
        box(-t.tw2, t.tw2, t.arm_y0, t.arm_y1, t.tz0, FR_Z1),               # şaquli qol
        ext_xy(tail_outline(), t.tz0, t.tz1),                               # quyruq
        box(-20, 20, t.arm_y0 - t.rib, t.arm_y0 + 0.5, *t.rib_z),           # kontakt qabırğası
    ]
    part = union(solid)
    cut = [
        box(NOTCH_X, FR_X + 5, NOTCH_Y0, NOTCH_Y1, FR_Z0 - 5, FR_Z1 + 5),
        box(-FR_X - 5, -NOTCH_X, NOTCH_Y0, NOTCH_Y1, FR_Z0 - 5, FR_Z1 + 5),
        yz_prism([(t.seat_y, FR_Z0 - 0.01), (FR_Y0 - 2, FR_Z0 - 0.01), (FR_Y0 - 2, FR_Z0 + (t.seat_y - FR_Y0 + 2))],
                 -FR_X - 6, FR_X + 6),
    ]
    bay = tail_outline().buffer(-t.wall, join_style=2).intersection(sbox(-50, t.arm_y1, 50, 200))
    cut.append(ext_xy(bay, t.tz0 + t.wall, t.tz1 + 1))
    part = part - union(cut)
    # qapaq dayaqları (bölmənin yan divarlarına bitişik)
    bosses = [vcyl(s * bx, by, 2.7, t.tz0 + t.wall - 0.01, t.tz1, seg=28) for bx, by in t.boss_xy for s in (-1, 1)]
    part = part + union(bosses)
    cut = []
    for bx, by in t.boss_xy:
        for s in (-1, 1):
            cut.append(vcyl(s * bx, by, 1.3, t.tz1 - 10, t.tz1 + 1, seg=20))     # M3 öz-açan vint üçün
    for s in (-1, 1):
        x_in, x_out = sorted((s * (t.tw2 - t.wall - 0.2), s * (t.tw2 + 0.2)))
        cut.append(box(x_in, x_out, t.slot_y[0], t.slot_y[1], t.tape_z - 5.5, t.tape_z + 5.5))   # lent yarığı
        x_in, x_out = sorted((s * (t.tw2 - 0.2), s * (t.tw2 + 1)))
        cut.append(box(x_in, x_out, t.tape_y0, t.tape_y1, t.tape_z - 5.3, t.tape_z + 5.3))         # 0.2 mm lent yuvası
        # polyarlıq işarəsi: F +x = "+", F −x = "−"   (real robotda: arxadan baxanda SAĞ = "+"? bax README)
        xa, xb = sorted((s * (t.tw2 - 0.5), s * (t.tw2 + 1)))
        mark = plus_on_xface(xa, xb, 62.0, t.tz0 + 3.6, 5.0) if s > 0 else minus_on_xface(xa, xb, 62.0, t.tz0 + 3.6, 5.0)
        cut.append(mark)
    cut.append(box(-4, 4, FR_Y0 - 1, t.arm_y1 + 1, FR_Z1 - 2.5, FR_Z1 + 1))           # naqil kanalı (çərçivənin üstü)
    return part - union(cut)


def build_lid() -> mf.Manifold:
    t = SP
    z0, z1 = t.tz1, t.tz1 + t.lid_t
    lid = ext_xy(tail_outline(y_from=t.arm_y1 + 0.2), z0, z1)
    cut = [vcyl(s * bx, by, 1.7, z0 - 1, z1 + 1, seg=24) for bx, by in t.boss_xy for s in (-1, 1)]
    w = Polygon([(-13, 52), (13, 52), (13, 64), (-13, 64)]).buffer(2.0)                 # LED pəncərəsi
    cut.append(ext_xy(w, z0 - 1, z1 + 1))
    cut.append(box(-5, 5, t.arm_y1 - 1, t.arm_y1 + 3.2, z0 - 1, z1 + 1))                # naqil çıxışı
    cut.append(ext_xy(bolt_poly(0, 74.5, 4.2), z1 - 0.5, z1 + 1))                      # ⚡ işarəsi
    return lid - union(cut)


# ================================================================== stansiya
def throat_wall_poly(s):
    t = TP
    inner = [(s * t.mouth, t.y0), (s * t.th, t.funnel_y), (s * t.th, t.back_y + t.back_t)]
    ls = LineString(inner)
    a = ls.offset_curve(t.wall, join_style=2)
    b = ls.offset_curve(-t.wall, join_style=2)
    # boğaz mərkəzindən uzaq tərəfdəki offset-i seç
    outer = a if abs(np.mean([p[0] for p in a.coords])) > abs(np.mean([p[0] for p in b.coords])) else b
    oc = list(outer.coords)
    if np.hypot(oc[0][0] - inner[0][0], oc[0][1] - inner[0][1]) > np.hypot(oc[-1][0] - inner[0][0], oc[-1][1] - inner[0][1]):
        oc = oc[::-1]
    return Polygon(inner + oc[::-1]).buffer(0)


def station_switch_box():
    """KW11-3Z tipli mikrovaç (20×10×6.4 mm) yan üstə uzanır; düymə üzü boğaza baxır.
    Gövdə dayaq divarının yarığına qədər irəli gəlir → ləçək boğaza ~3 mm çıxır,
    quyruq son 3 mm-də onu mütləq basır (ləçək ucu ən çox dayaq üzünə qədər gedə bilər)."""
    t = TP
    y0 = t.back_y + 0.8
    return dict(x0=-4.0, x1=16.0, y0=y0, y1=y0 + 10.0, z0=t.pz - 3.2, z1=t.pz + 3.2)


def build_station_body() -> mf.Manifold:
    t = TP
    zb = ZF + t.cover_t
    ru = t.roof_under
    shell = box(-t.hw, t.hw, t.y0, t.y1, zb, ru + t.roof_t)
    shell = shell - box(-t.hw + t.wall, t.hw - t.wall, t.y0 + t.wall, t.y1 - t.wall, zb - 1, ru)
    shell = shell - box(-t.mouth, t.mouth, t.y0 - 1, t.y0 + t.wall + 1, zb - 1, ru)
    solid = [shell]
    for s in (-1, 1):
        solid.append(ext_xy(throat_wall_poly(s), zb, ru + 0.01))
    back = box(-(t.th + t.wall), t.th + t.wall, t.back_y, t.back_y + t.back_t, zb, ru + 0.01)
    back = back - box(-17, 19, t.back_y - 1, t.back_y + t.back_t + 1, t.pz - 5, t.pz + 5)   # mikrovaç ləçəyi üçün yarıq
    solid.append(back)
    for s in (-1, 1):                                                                      # plunjer gövdələri
        x0, x1 = sorted((s * (t.th - 0.01), s * (t.th + t.bore_depth + t.bore_wall)))
        solid.append(cyl_x(x0, x1, t.py, t.pz, t.boss_d / 2))
        # gövdəni dama bağlayan sütun: başıaşağı çapda silindr havada qalmasın (dayaq lazım olmasın)
        solid.append(box(x0, x1, t.py - t.boss_d / 2, t.py + t.boss_d / 2, t.pz, ru + 0.01))
    sw = station_switch_box()
    fz1 = sw["z0"] - 0.15
    solid += [
        box(sw["x0"] - 0.5, sw["x1"] + 1.9, sw["y0"] - 0.3, sw["y1"] + 1.9, fz1 - 1.6, fz1),                     # beşik döşəməsi
        box(sw["x0"] - 0.5, sw["x1"] + 1.9, sw["y1"] + 0.3, sw["y1"] + 1.9, fz1 - 1.6, ru + 0.01),               # beşiyin arxası
        box(sw["x1"] + 0.3, sw["x1"] + 1.9, sw["y0"] - 0.3, sw["y1"] + 1.9, fz1 - 1.6, ru + 0.01),              # beşiyin ucu
    ]
    for px in (2.0, 12.0):
        solid.append(vcyl(px, (sw["y0"] + sw["y1"]) / 2, 2.2, sw["z1"] + 0.15, ru + 0.01, seg=24))
    for lx in (-12.0, 12.0):                                                               # LED borucuqları
        solid.append(vcyl(lx, 110.0, 3.6, ru - 6, ru + 0.01, seg=28))
    for px in (19.0, 31.0):                                                                # kabel bağı dirəkləri
        solid.append(vcyl(px, 116.0, 2.0, zb + 3, ru + 0.01, seg=20))
    bosses = [(-45, 55.5), (45, 55.5), (-45, 120.0), (45, 120.0)]
    for bx, by in bosses:
        solid.append(vcyl(bx, by, 3.5, zb, ru + 0.01, seg=28))
    body = union(solid)
    cut = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (t.th - 1), s * (t.th + t.bore_depth)))
        cut.append(tear_x(x0, x1, t.py, t.pz, t.bore_d / 2))
        x0, x1 = sorted((s * (t.th + t.bore_depth - 0.5), s * (t.th + t.bore_depth + t.bore_wall + 1)))
        cut.append(tear_x(x0, x1, t.py, t.pz, t.hole_d / 2))
    cut.append(box(sw["x0"] + 1, sw["x1"] - 1, sw["y1"] - 1, sw["y1"] + 3, sw["z0"] + 0.8, sw["z1"] - 0.8))   # terminallar
    for px in (2.0, 12.0):
        cut.append(vcyl(px, (sw["y0"] + sw["y1"]) / 2, 0.85, sw["z1"] - 1, sw["z1"] + 8, seg=16))           # M2 pilot
    for lx in (-12.0, 12.0):
        cut.append(vcyl(lx, 110.0, 2.55, ru - 7, ru + t.roof_t + 1, seg=28))
    cut.append(tear_y(t.y1 - t.wall - 1, t.y1 + 1, 25.0, ZF + 14.0, 3.25))                 # USB kabel girişi
    for bx, by in bosses:
        cut.append(vcyl(bx, by, 1.3, zb - 1, zb + 12, seg=20))
    rt = ru + t.roof_t
    cut.append(ext_xy(bolt_poly(0, 100.0, 9.0), rt - 0.6, rt + 1))                       # ⚡ damın üstündə
    cut.append(plus_sign(36.0, t.py, 8.0, rt - 0.6, rt + 1))                               # F +x = +5V
    cut.append(minus_sign(-36.0, t.py, 8.0, rt - 0.6, rt + 1))
    return body - union(cut)


def build_station_cover() -> mf.Manifold:
    t = TP
    c = box(-t.hw, t.hw, t.y0, t.y1, ZF, ZF + t.cover_t)
    cut = [yz_prism([(t.y0 - 0.01, ZF + t.cover_t + 0.01), (t.y0 + 1.2, ZF + t.cover_t + 0.01), (t.y0 - 0.01, ZF + t.cover_t - 1.2)],
                    -t.hw - 1, t.hw + 1)]
    for bx, by in [(-45, 55.5), (45, 55.5), (-45, 120.0), (45, 120.0)]:
        cut.append(vcyl(bx, by, 1.7, ZF - 1, ZF + t.cover_t + 1, seg=24))
        cut.append(cone_z(bx, by, ZF - 0.01, ZF + 1.6, 3.3, 1.7))
    # rezin ayaqlar düz alt səthə yapışdırılır (yuva yoxdur — ilk qatda körpü yaranmasın)
    return c - union(cut)


# lent yerləşdirmə şablonu (stansiyanın önünə dayanır)
LINE_START_Y = 40.0          # xətt stansiyanın ön üzündən 9 mm əvvəl başlayır
T_Y = (-17.6, 1.9)           # T zolağı (19 mm lent üçün 19.5 mm yarıq) — F çərçivəsində
T_HALF = 50.25
SPIN_Y = -80.0               # robot T-də dayanıb 180° fırlananda onun mərkəzi (F çərçivəsi)


def build_template() -> mf.Manifold:
    t = TP
    z0, z1 = ZF, ZF + 1.2
    plate = box(-60, 60, -32, t.y0 - 0.4, z0, z1)
    rib = box(-60, 60, t.y0 - 2.4, t.y0 - 0.4, z0, ZF + 3.0)
    tabs = [box(t.hw + 0.3, t.hw + 2.3, t.y0 - 2.4, t.y0 + 9, z0, ZF + 3.0),
            box(-t.hw - 2.3, -t.hw - 0.3, t.y0 - 2.4, t.y0 + 9, z0, ZF + 3.0)]
    part = union([plate, rib] + tabs)
    cut = [box(-9.75, 9.75, -33, LINE_START_Y, z0 - 1, z1 + 1),
           box(-T_HALF, T_HALF, T_Y[0], T_Y[1], z0 - 1, z1 + 1)]
    for s in (-1, 1):                                                                      # stansiyaya göstərən oxlar
        cut.append(ext_xy([(s * 30 - 5, 20), (s * 30 + 5, 20), (s * 30, 30)], z1 - 0.5, z1 + 1))
    return part - union(cut)


# ================================================================== hardware (yalnız görüntü / yoxlama üçün)
def plunger_hw(s, compression):
    """s=+1 → F +x divarı. compression: başlığın sərbəst vəziyyətdən içəri basılması (mm)."""
    t = TP
    head_face = s * (t.th - t.head_free_protrusion + compression)       # kontakt səthi
    hx = lambda a, b: sorted((a, b))                                     # noqa: E731
    head = cyl_x(*hx(head_face, head_face + s * 2.1), t.py, t.pz, 2.75)
    shank = cyl_x(*hx(head_face + s * 2.1, head_face + s * 22.1), t.py, t.pz, 1.5)
    under = head_face + s * 2.1
    bore_bottom = s * (t.th + t.bore_depth)
    spring = cyl_x(*hx(under, bore_bottom), t.py, t.pz, 2.9) - cyl_x(*hx(under - s, bore_bottom + s), t.py, t.pz, 2.3)
    wall_out = s * (t.th + t.bore_depth + t.bore_wall)
    n1 = wall_out + s * compression
    nut1 = hex_x(*hx(n1, n1 + s * 2.4), t.py, t.pz)
    lug = cyl_x(*hx(n1 + s * 2.4, n1 + s * 3.2), t.py, t.pz, 3.0)
    nut2 = hex_x(*hx(n1 + s * 3.2, n1 + s * 5.6), t.py, t.pz)
    return dict(metal=head + shank + nut1 + nut2, spring=spring, lug=lug)


def switch_hw(pressed):
    sw = station_switch_box()
    body = box(sw["x0"], sw["x1"], sw["y0"], sw["y1"], sw["z0"], sw["z1"])
    hinge_x, hinge_y = sw["x1"] - 1.5, sw["y0"]
    L = 18.0
    # sərbəst: ~10° (uc boğaza ~2.3 mm çıxır); basılı: uc quyruğun ucu ilə eyni müstəvidə (y = 84.0)
    ang = np.arcsin((hinge_y - SP.ty1) / L) if pressed else np.radians(10.0)
    tip = (hinge_x - L * np.cos(ang), hinge_y - L * np.sin(ang))
    lever = hull([(hinge_x, hinge_y, sw["z0"] + 1.6), (hinge_x, hinge_y, sw["z1"] - 1.6),
                  (tip[0], tip[1], sw["z0"] + 1.6), (tip[0], tip[1], sw["z1"] - 1.6),
                  (hinge_x, hinge_y - 0.4, sw["z0"] + 1.6), (tip[0], tip[1] - 0.4, sw["z1"] - 1.6)])
    pins = union([box(sw["x0"] + 2 + i * 7, sw["x0"] + 2.6 + i * 7, sw["y1"], sw["y1"] + 3.5, sw["z0"] + 1.5, sw["z1"] - 1.5) for i in range(3)])
    return dict(body=body, lever=lever, pins=pins, tip_y=tip[1])


def leds_hw():
    t = TP
    out = []
    for lx, col in ((-12.0, "#ff2a2a"), (12.0, "#29e05a")):
        m = vcyl(lx, 110.0, 2.5, t.roof_under - 4, t.roof_under + t.roof_t + 2.5, seg=24) + \
            mf.Manifold.sphere(2.5, 24).translate([lx, 110.0, t.roof_under + t.roof_t + 2.5])
        out.append((m, col))
    return out


def tape_hw():
    t = SP
    out = []
    for s in (-1, 1):
        x0, x1 = sorted((s * (t.tw2 - 0.2), s * (t.tw2 - 0.12)))
        face = box(x0, x1, t.tape_y0, t.tape_y1, t.tape_z - 5, t.tape_z + 5)
        xi0, xi1 = sorted((s * (t.tw2 - 0.2), s * (t.tw2 - t.wall - 0.2)))
        through = box(xi0, xi1, t.slot_y[0] + 0.3, t.slot_y[0] + 0.4, t.tape_z - 5, t.tape_z + 5)
        xb0, xb1 = sorted((s * (t.tw2 - t.wall - 0.2), s * (t.tw2 - t.wall - 6)))
        inside = box(xb0, xb1, t.slot_y[0] + 0.3, t.slot_y[0] + 0.4, t.tape_z - 5, t.tape_z + 5)
        out.append(face + through + inside)
    return union(out)


def module_hw():
    """IP2326 2S şarj modulu (~30×20 mm, satıcıya görə fərqlənir) — bölmənin döşəməsində."""
    t = SP
    z = t.tz0 + t.wall
    pcb = box(-15, 15, 48, 68, z + 1.0, z + 2.6)
    usb = box(-4.5, 4.5, 66, 69.5, z + 2.6, z + 5.8)
    coil = box(2, 9, 51, 58, z + 2.6, z + 6.0)
    ic = box(-8, -2, 54, 60, z + 2.6, z + 3.4)
    leds = box(-12, -9, 50, 52, z + 2.6, z + 3.4)
    return dict(pcb=pcb, parts=usb + coil + ic, leds=leds)


def wires_hw(points, r=0.7):
    segs = []
    for a, b in zip(points[:-1], points[1:]):
        segs.append(hull([tuple(np.add(a, d)) for d in [(r, 0, 0), (-r, 0, 0), (0, r, 0), (0, -r, 0), (0, 0, r), (0, 0, -r)]] +
                         [tuple(np.add(b, d)) for d in [(r, 0, 0), (-r, 0, 0), (0, r, 0), (0, -r, 0), (0, 0, r), (0, 0, -r)]]))
    return union(segs)


def lid_screws_hw():
    t = SP
    z = t.tz1 + t.lid_t
    return union([vcyl(s * bx, by, 2.75, z, z + 2.1, seg=20) for bx, by in t.boss_xy for s in (-1, 1)])


def battery_hw():
    """2S LiPo (≤ 50×31×20) + 2S BMS — şassinin mərkəzi bölməsində (chassis çərçivəsi)."""
    pack = box(-24.5, 24.5, -15, 15, -13.3, 4.7)
    bms = box(-23, 23, -10, 10, 4.8, 8.3)
    return pack, bms


# ================================================================== robot (chassis çərçivəsi)
def robot_parts():
    from check_assembly import build_running_gear
    from make_combo3 import params3
    wheels, links = build_running_gear()
    p = params3(9.0)
    mods = [build_module(p, xc) for xc in p.pocket_centres()]
    out3 = ROOT / "out_kombo3"
    return dict(
        chassis=from_trimesh(trimesh.load(ROOT / "_parts" / "chassis_SL.stl")),
        wheels=union(from_trimesh(w) for w in wheels),
        tracks=union(from_trimesh(l) for l in links),
        front=union([from_trimesh(trimesh.load(out3 / n)) for n in ("base_P9.0.stl", "blade.stl", "u2.stl")]),
        front_u1=from_trimesh(trimesh.load(out3 / "u1.stl")),
        tcrt=union(union(m[k] for k in ("pcb", "sensor", "pot", "header", "dupont")) for m in mods),
    )


# ================================================================== main: build, check, export
def main(outdir: Path):
    import time
    t0 = time.time()
    out_stl = outdir / "STL"
    (out_stl / "robot").mkdir(parents=True, exist_ok=True)
    (out_stl / "stansiya").mkdir(parents=True, exist_ok=True)
    asm_dir = outdir / "_asm"; asm_dir.mkdir(parents=True, exist_ok=True)

    clean = lambda m: m.simplify(0.001)                                    # noqa: E731  (CSG sliver-ləri)
    shoe, lid = clean(build_shoe()), clean(build_lid())
    body, cover, templ = clean(build_station_body()), clean(build_station_cover()), clean(build_template())
    print("parts built", round(time.time() - t0, 1), "s")

    rep = {}
    # ---------- print exports (çap vəziyyəti)
    def export_print(m, path, flip=False):
        tm = to_trimesh(m)
        if flip:
            tm.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
        lo, hi = tm.bounds
        tm.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
        tm.export(path)
        return tm

    prints = {
        "robot/SMARS_sarj_quyrugu.stl": (shoe, False),
        "robot/SMARS_sarj_quyrugu_qapaq.stl": (lid, False),
        "stansiya/SMARS_stansiya_govde.stl": (body, True),          # başıaşağı: dam masada
        "stansiya/SMARS_stansiya_alt_qapaq.stl": (cover, False),
        "stansiya/SMARS_stansiya_lent_sablonu.stl": (templ, False),
    }
    for name, (m, flip) in prints.items():
        tm = export_print(m, out_stl / name, flip)
        rep[name] = {"olcu_mm": np.round(tm.extents, 1).tolist(), "hecm_cm3": round(tm.volume / 1000, 2),
                     "watertight": bool(tm.is_watertight), "hisse_sayi": len(tm.split())}

    # ---------- robot assembly + checks (chassis çərçivəsi: quyruq ARXADA)
    R = robot_parts()
    shoe_c, lid_c = rot180(shoe), rot180(lid)
    gear = R["wheels"] + R["tracks"]
    vol = lambda a, b: round((a ^ b).volume(), 4)                      # noqa: E731
    gap = lambda a, b, s=20: round(a.min_gap(b, s), 3)                  # noqa: E731
    rep["quyruq^gövdə_mm3"] = vol(shoe_c, R["chassis"])
    rep["quyruq_0.1mm_aşağı^gövdə_mm3 (>0 = yuvada oturur)"] = vol(shoe_c.translate([0, 0, -0.1]), R["chassis"])
    rep["quyruq^təkər/tırtıl_mm3"] = vol(shoe_c + lid_c, gear)
    rep["quyruq↔təkər/tırtıl_min_boşluq_mm"] = gap(shoe_c + lid_c, gear)
    rep["qapaq^quyruq_mm3"] = vol(lid_c, shoe_c)

    # ---------- docked: stansiya (F → chassis) + robot (aşağı / yuxarı vəziyyət, yan sürüşmə ±c, yaw ±1°)
    body_c, cover_c = rot180(body), rot180(cover)
    station_c = body_c + cover_c
    robot_rigid = union([R["chassis"], gear, R["front"], R["front_u1"], R["tcrt"], shoe_c, lid_c, rot180(lid_screws_hw())])
    worst = []
    for dz in (0.0, BOB):                                   # təkər yırğalanması: ən aşağı / ən yuxarı
        for dx in (-TP.c + 0.05, 0.0, TP.c - 0.05):          # boğaz boşluğu daxilində yan sürüşmə
            rb = robot_rigid.translate([dx, 0, dz])
            worst.append(((dz, dx), vol(rb, station_c)))
    rep["dok: robot^stansiya_mm3 (6 hal: hündürlük × yan sürüşmə)"] = max(v for _, v in worst)
    ymax = 0.0
    for yaw in np.arange(0.1, 5.0, 0.1):
        if vol(shoe_c.rotate([0, 0, yaw]), station_c) > 1e-3:
            break
        ymax = yaw
    rep["dok: boğazda icazə verilən yaw ±°  (divarlar robotu bu dəqiqliyə düzəldir)"] = round(float(ymax), 1)
    rep["dok: tırtıl ↔ stansiya ön üzü min boşluq_mm"] = gap(gear, station_c, 10)
    rep["dok: quyruq+qapaq ↔ boğaz/dam min boşluq_mm"] = gap(shoe_c + lid_c + rot180(lid_screws_hw()), station_c, 10)

    # plunjer sıxılması: quyruğun yan üzü |x|=22, sərbəst başlıq |x| = th − protrusion
    free_face = TP.th - TP.head_free_protrusion
    comp = [round(SP.tw2 + d - free_face, 2) for d in (-TP.c, 0, TP.c)]
    rep["plunjer_sıxılması_mm [min, mərkəz, maks] (iş aralığı ~5 mm)"] = comp
    rep["huni_tutma_aralığı_yan_±mm"] = round(TP.mouth - SP.tw2, 2)
    rep["kontakt_lent_üzərində_plunjer: ucdan_mm / içəri_mm"] = [round(SP.tape_y1 - TP.py, 1), round(TP.py - SP.tape_y0, 1)]
    rep["kontakt hündürlüyü: lent z aralığı vs plunjer (yırğalanma daxil)"] = \
        [SP.tape_z - SP.tape_w / 2, SP.tape_z + SP.tape_w / 2, TP.pz, TP.pz - BOB]

    # mikrovaç: ləçəyin ucu dayaq divarı üzünün qarşısında olmalıdır
    sw_free, sw_press = switch_hw(False), switch_hw(True)
    rep["mikrovaç: ləçək ucu (sərbəst) y, dayaq üzü y, quyruq ucu y"] = [round(sw_free["tip_y"], 2), TP.back_y, SP.ty1]

    # 180° fırlanma (T-də) — robotun süpürdüyü sahə stansiyaya dəyməməlidir
    spin_min = 1e9
    rob_spin = union([R["chassis"], gear, R["front"], R["front_u1"], shoe_c, lid_c])
    for ang in range(0, 360, 6):
        rb = rob_spin.rotate([0, 0, ang]).translate([0, -SPIN_Y, 0])        # chassis: mərkəz (0, +80)
        spin_min = min(spin_min, rb.min_gap(station_c, 60))
    rep["T-də 180° fırlanma: robot ↔ stansiya min boşluq_mm"] = round(spin_min, 2)
    print("checks done", round(time.time() - t0, 1), "s")

    # ---------- assembly model (rəngli)
    hw_c = {}
    for side in (-1, 1):
        h = plunger_hw(side, SP.tw2 - free_face)
        for k, v in h.items():
            hw_c[k] = hw_c.get(k, mf.Manifold()) + rot180(v)
    sw_c = {k: rot180(v) for k, v in switch_hw(True).items() if k != "tip_y"}
    mod = {k: rot180(v) for k, v in module_hw().items()}
    pack, bms = battery_hw()
    t = SP
    wire_red = rot180(wires_hw([(t.tw2 - 6, 46.9, -6), (8, 50, -12), (6, 60, -12)]))
    wire_blk = rot180(wires_hw([(-t.tw2 + 6, 46.9, -6), (-8, 50, -12), (-6, 60, -12)]))
    wire_out = rot180(wires_hw([(0, 62, -14), (0, 40.5, -10), (0, 40.5, 7.5), (0, 33, 8), (0, 20, 6), (0, 12, 6)], r=0.9))
    usb = rot180(cyl_y(TP.y1 - 3, TP.y1 + 60, 25.0, ZF + 14.0, 2.0))
    tape = rot180(tape_hw())
    line_c = []                                                          # qara lent (yer) — chassis çərçivəsi
    Lw = 9.5
    line_c.append(rot180(box(-Lw, Lw, -230, LINE_START_Y, ZF - 0.3, ZF - 0.05)))
    line_c.append(rot180(box(-50, 50, T_Y[0] + 0.25, T_Y[1] - 0.25, ZF - 0.3, ZF - 0.05)))
    floor = box(-140, 140, -150, 260, ZF - 1.3, ZF - 0.3)
    leds = [(rot180(m), c) for m, c in leds_hw()]

    assembly = [
        ("gövdə (chassis_SL)", R["chassis"], "#e9edf0"), ("təkərlər", R["wheels"], "#7cdc3a"), ("tırtıl", R["tracks"], "#8a929a"),
        ("ön modul (kaset+kürək+korpus)", R["front"], "#f2b705"), ("ultrasəs korpusu", R["front_u1"], "#f4f6f8"),
        ("TCRT5000", R["tcrt"], "#1d5fbf"),
        ("2S batareya", pack, "#3b7dd8"), ("2S BMS", bms, "#1a8f3c"),
        ("şarj quyruğu", shoe_c, "#ff7a1a"), ("quyruq qapağı", lid_c, "#ffb066"), ("qapaq vintləri", rot180(lid_screws_hw()), "#b8bec4"),
        ("mis lent kontaktlar", tape, "#d9822b"), ("IP2326 modul", mod["pcb"], "#1d5fbf"), ("modul hissələri", mod["parts"], "#9aa3ab"),
        ("modul LED", mod["leds"], "#29e05a"),
        ("qırmızı naqil", wire_red, "#e02020"), ("qara naqil", wire_blk, "#202020"), ("çıxış naqilləri", wire_out, "#e0a000"),
        ("stansiya gövdəsi", body_c, "#2b3440"), ("stansiya alt qapağı", cover_c, "#3a4652"),
        ("plunjer vint/qayka", hw_c["metal"], "#c9ced3"), ("yay", hw_c["spring"], "#9da5ad"), ("kabel ucluğu", hw_c["lug"], "#d4a017"),
        ("mikrovaç", sw_c["body"], "#1c1c1c"), ("mikrovaç ləçəyi", sw_c["lever"], "#c9ced3"), ("mikrovaç ayaqları", sw_c["pins"], "#d4a017"),
        ("USB kabel", usb, "#111111"),
    ] + [(f"LED {i}", m, c) for i, (m, c) in enumerate(leds)] + [
        ("qara xətt + T", union(line_c), "#0b0b0b"), ("döşəmə", floor, "#f1f1ec"),
    ]
    scene = trimesh.Scene()
    for name, m, col in assembly:
        tm = to_trimesh(m)
        if len(tm.faces) == 0:
            continue
        rgba = np.array([int(col[i:i + 2], 16) for i in (1, 3, 5)] + [255], dtype=np.uint8)
        tm.visual.face_colors = np.tile(rgba, (len(tm.faces), 1))
        scene.add_geometry(tm, node_name=name, geom_name=name)
        tm.export(asm_dir / (name.replace(" ", "_").replace("/", "_").replace("(", "").replace(")", "").replace("+", "_") + ".stl"))
    (outdir / "montaj").mkdir(exist_ok=True)
    scene.export(outdir / "montaj" / "SMARS_sarj_montaj.glb")

    # yalnız çap hissələri — dok vəziyyətində (Bambu Studio / Prusa üçün 3MF)
    sc2 = trimesh.Scene()
    for name, m in [("sarj_quyrugu", shoe_c), ("quyruq_qapaq", lid_c), ("stansiya_govde", body_c), ("stansiya_alt_qapaq", cover_c)]:
        sc2.add_geometry(to_trimesh(m), node_name=name, geom_name=name)
    sc2.export(outdir / "montaj" / "SMARS_sarj_hisseler_dok_veziyyetinde.3mf")

    rep["_qeyd"] = "chassis çərçivəsi: robot +Y irəli; quyruq və stansiya arxada (−Y)."
    (outdir / "_report.json").write_text(json.dumps(rep, indent=2, ensure_ascii=False))
    print(json.dumps(rep, indent=2, ensure_ascii=False))
    # yerləşmə/simulyasiya üçün parametrlər
    params = dict(shoe=asdict(SP), station=asdict(TP), line_start_y=LINE_START_Y, t_y=T_Y, t_half=T_HALF, spin_y=SPIN_Y,
                  zf=ZF, bob=BOB, throat_half=TP.th, mouth_half=TP.mouth, roof_under=TP.roof_under,
                  switch_tip_free_y=sw_free["tip_y"])
    (outdir / "_params.json").write_text(json.dumps(params, indent=2))
    print("total", round(time.time() - t0, 1), "s")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "smars-charger")
