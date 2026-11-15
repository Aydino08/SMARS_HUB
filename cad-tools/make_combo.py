"""
SMARS  -  front extension "kombo":
    * 3 x TCRT5000 line sensors (same cassette as make_mount.py, sensors front / jumpers back)
    * HC-SR04 ultrasonic sensor holder on top (sensor mounted pins-up, slides in from above)
    * bulldozer blade ("kürək") - separate part, slides down onto two dovetail rails on the
      front cross-beam of the cassette

All coordinates are SMARS chassis_SL coordinates (mm): +Y forward, +Z up, origin = chassis centre.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import manifold3d as mf
import numpy as np
import trimesh
from shapely.geometry import Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import (Params, build_mount, box, hull, yz_prism, vcyl, to_trimesh,  # noqa: E402
                        GROUND_Z_LOW, GROUND_Z_HIGH, AXLE_Y, WALL_Y1, FR_Z1)

ROOT = Path(__file__).resolve().parent.parent
ADD = mf.OpType.Add


@dataclass
class Combo:
    # ---------------- HC-SR04 (45 x 20 PCB, 2 x D16 transducers, 26 mm apart) ----------------
    hc_w: float = 45.0
    hc_h: float = 20.0
    hc_t: float = 1.6
    hc_tr_d: float = 16.0
    hc_tr_h: float = 12.0
    hc_tr_dx: float = 13.0
    hc_back: float = 2.5           # components on the back of the PCB
    hc_y_back: float = 40.5        # PCB back face (wires of the TCRTs rise behind it)
    hc_z0: float = 16.0            # PCB lower edge  -> 7 mm cable tunnel above the arm plate (z=9)
    # holder posts
    post_x_in: float = 21.6        # inner face of the posts (transducer edge is at 21.0)
    post_x_out: float = 25.15
    post_y0_low: float = WALL_Y1 + 0.25   # on the arm plate top
    post_y0_high: float = 37.3     # posts are trimmed above the tunnel (Arduino clearance)
    groove_x: float = 22.85        # groove bottom: PCB edge 22.5 + 0.35
    groove_w: float = 2.0          # PCB 1.6 + 0.4
    lip: float = 2.0
    # ---------------- dovetail joint for the blade ----------------
    beam_depth: float = 4.0        # front cross-beam of the cassette
    rail_x: tuple = (-15.0, 15.0)
    rail_root: float = 6.0
    rail_tip: float = 8.4
    rail_depth: float = 2.5
    rail_top: float = -13.0        # blade hangs on the rail tops (same absolute z for every P)
    rail_min_len: float = 5.5
    dt_clear: float = 0.2
    mount_gap: float = 0.15        # blade block <-> beam front face
    # ---------------- blade ----------------
    blade_plan_r: float = 120.0    # plan radius (U-blade: wings forward)
    blade_half_w: float = 50.0
    blade_t: float = 2.4
    blade_h: float = 30.0
    blade_a_bot: float = -35.0     # profile angles; |a| <= 45 deg -> prints without support
    blade_a_top: float = 40.0
    blade_clear: float = 2.5       # cutting edge above the floor (chassis lowest, at rest)
    end_plate: float = 6.0
    end_t: float = 2.0
    rib_depth: float = 4.0
    rib_t: float = 2.0
    rib_x: tuple = (-36.0, 36.0)
    block_half_w: float = 20.0
    block_front: float = 2.0       # material in front of the dovetail channels


# =================================================================================
# base: TCRT cassette + HC-SR04 posts + front beam with dovetail rails
# =================================================================================
def combo_params(P: float) -> Params:
    return Params(protrusion=P, front_bar=Combo().beam_depth, front_chamfer=0.0)


def dovetail_poly(xc, y_root, root, tip, depth, grow=0.0):
    """Male rail profile in XY: narrow (root) at the beam face, wide (tip) at y_root+depth.
    grow > 0 returns the same profile offset outwards by `grow` (the female channel)."""
    pts = [(xc - root / 2, y_root - 0.4), (xc + root / 2, y_root - 0.4),
           (xc + tip / 2, y_root + depth), (xc - tip / 2, y_root + depth)]
    if grow <= 0:
        return pts
    off = Polygon(pts).buffer(grow, join_style=2, mitre_limit=3.0)
    return [tuple(q) for q in np.array(off.exterior.coords)[:-1]]


def prism_xy(poly, z0, z1):
    return hull([(x, y, z) for (x, y) in poly for z in (z0, z1)])


def build_base(p: Params, c: Combo, posts: bool = True) -> mf.Manifold:
    base = build_mount(p)
    hw = p.half_w
    add, cut = [], []

    # ---- front cross-beam (closes the cassette front, carries the rails) ----
    yb0 = p.y_back + p.window_y[1]
    yb1 = p.y_front
    zb1 = max(p.wall_z1, c.rail_top + 0.6)
    add.append(box(-hw, hw, yb0, yb1, p.floor_z0, zb1))
    rail_z0 = p.floor_z0 + 0.3                          # keeps elephant-foot off the rail
    assert c.rail_top - rail_z0 >= c.rail_min_len, "rail too short for this P"
    for xr in c.rail_x:
        add.append(prism_xy(dovetail_poly(xr, yb1, c.rail_root, c.rail_tip, c.rail_depth),
                            rail_z0, c.rail_top))

    # ---- HC-SR04 posts ----
    yf = c.hc_y_back + c.hc_t + 0.2 + c.lip          # front of the posts
    for s in ((-1, 1) if posts else ()):
        xi, xo = sorted((s * c.post_x_in, s * c.post_x_out))
        xw0, xw1 = sorted((s * (p.pocket_w * 1.5 + p.wall_in), s * p.half_w))   # outer pocket wall
        top = c.hc_z0 + c.hc_h + 1.0
        add.append(box(xi, xo, c.post_y0_low, yf, FR_Z1, c.hc_z0))            # lower part
        add.append(box(xi, xo, c.post_y0_high, yf, c.hc_z0 - 0.01, top))      # upper part
        add.append(box(xw0, xw1, p.arm_y1 - 0.5, yf, p.wall_z1 - 0.01, FR_Z1 + 0.01))  # on outer wall
        # 45 deg chamfer under the part that overhangs the outer pocket (in front of the arm plate)
        xa = s * (p.pocket_w * 1.5 + p.wall_in)          # = +-23.35 (pocket outer face)
        xb = s * (c.post_x_in - 0.2)
        h = abs(xa - xb)
        tri = [(xa, FR_Z1 - 0.01), (xb, FR_Z1 - 0.01), (xb, FR_Z1 + h)]
        cut.append(hull([(x, y, z) for (x, z) in tri for y in (p.arm_y1, yf + 0.5)]))
        # groove for the PCB edge
        g0, g1 = sorted((s * (c.post_x_in - 0.3), s * c.groove_x))
        cut.append(box(g0, g1, c.hc_y_back - 0.2, c.hc_y_back + c.hc_t + 0.2, c.hc_z0, top + 1))
        # lead-in chamfer at the groove top
        cut.append(hull([(x, y, z) for x in (g0, g1)
                         for (y, z) in ((c.hc_y_back - 0.2, top - 1.0), (c.hc_y_back + c.hc_t + 0.2, top - 1.0),
                                        (c.hc_y_back - 0.9, top + 0.01), (c.hc_y_back + c.hc_t + 0.9, top + 0.01))]))

    part = mf.Manifold.batch_boolean([base] + add, ADD)
    part = part - mf.Manifold.batch_boolean(cut, ADD)
    # crush bumps in the grooves (0.1 mm squeeze on the PCB back face)
    bumps = []
    if not posts:
        return part
    for s in (-1, 1):
        bumps.append(vcyl(s * 22.25, c.hc_y_back - 0.2 - 0.15, 0.45, c.hc_z0 + 8, c.hc_z0 + c.hc_h - 1.5, seg=20))
    part = part + mf.Manifold.batch_boolean(bumps, ADD)
    return part


# =================================================================================
# blade
# =================================================================================
def _clip_poly(pts, z_min):
    poly = Polygon(pts).buffer(0)
    poly = poly.intersection(sbox(-1, z_min, 1000, 1000))
    if poly.geom_type != "Polygon":
        poly = max(poly.geoms, key=lambda g: g.area)
    return [list(map(float, q)) for q in np.array(poly.exterior.coords)[:-1]]


def _arc_band(rc, zc, r_in, r_out, a0, a1, z_min, n=64):
    a = np.radians(np.linspace(a0, a1, n))
    outer = [(rc + r_out * np.cos(t), zc + r_out * np.sin(t)) for t in a]
    inner = [(rc + r_in * np.cos(t), zc + r_in * np.sin(t)) for t in a[::-1]]
    return _clip_poly(outer + inner, z_min)


def _revolve(poly_rz, deg, seg=96):
    arr = np.asarray(poly_rz, dtype=np.float64)
    x, y = arr[:, 0], arr[:, 1]
    if 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) < 0:     # manifold needs CCW contours
        arr = arr[::-1].copy()
    cs = mf.CrossSection([arr])
    m = mf.Manifold.revolve(cs, seg, deg)
    assert m.num_vert() > 0, "empty revolve"
    return m


def _centre_on(m: mf.Manifold, angle_deg: float, y_axis: float) -> mf.Manifold:
    """rotate a revolved piece about Z so its mean angle is angle_deg, then move the axis to (0, y_axis)."""
    t = to_trimesh(m)
    cx, cy = t.vertices[:, 0].mean(), t.vertices[:, 1].mean()
    phi = np.degrees(np.arctan2(cy, cx))
    return m.rotate([0, 0, angle_deg - phi]).translate([0, y_axis, 0])


@dataclass
class BladeGeom:
    y_axis: float
    y_mid0: float
    r_s: float
    rc: float
    zc: float
    z_bot: float
    z_top: float
    span_deg: float
    droop: float


def blade_geom(p: Params, c: Combo) -> BladeGeom:
    y_mount = p.y_front + c.mount_gap
    y_back_surf = p.y_front + c.rail_depth + c.dt_clear + c.block_front     # blade back @ centre
    y_mid0 = y_back_surf + c.blade_t / 2
    r_s = c.blade_h / (np.sin(np.radians(c.blade_a_top)) - np.sin(np.radians(c.blade_a_bot)))
    y_edge = y_mid0 + r_s * (1 - np.cos(np.radians(c.blade_a_bot)))
    droop = (y_edge - WALL_Y1) * np.sin(p.droop_angle)
    z_bot = GROUND_Z_LOW + c.blade_clear + droop                              # as built
    zc = z_bot - r_s * np.sin(np.radians(c.blade_a_bot))
    span = 2 * np.degrees(np.arcsin(c.blade_half_w / c.blade_plan_r))
    return BladeGeom(y_axis=y_mid0 + c.blade_plan_r, y_mid0=y_mid0, r_s=r_s,
                     rc=c.blade_plan_r - r_s, zc=zc, z_bot=z_bot, z_top=z_bot + c.blade_h,
                     span_deg=span, droop=droop)


def build_blade(p: Params, c: Combo) -> mf.Manifold:
    g = blade_geom(p, c)
    t = c.blade_t
    a_lo = c.blade_a_bot - 20
    parts = []
    # main moldboard (doubly curved: profile arc revolved around a vertical axis in front)
    band = _arc_band(g.rc, g.zc, g.r_s - t / 2, g.r_s + t / 2, a_lo, c.blade_a_top, g.z_bot)
    parts.append(_centre_on(_revolve(band, g.span_deg), -90, g.y_axis))
    # end plates (curved fins at both wing tips, sticking out in front of the blade)
    end_deg = np.degrees(c.end_t / c.blade_plan_r)
    fin = _arc_band(g.rc, g.zc, g.r_s - t / 2 - c.end_plate, g.r_s + t / 2 - 0.4, a_lo, c.blade_a_top - 0.5,
                    g.z_bot)
    for s in (-1, 1):
        a = -90 + s * (g.span_deg / 2 - end_deg / 2 - 0.05)     # tiny inset: no coincident end faces
        parts.append(_centre_on(_revolve(fin, end_deg, 8), a, g.y_axis))
    # back ribs
    rib_deg = np.degrees(c.rib_t / c.blade_plan_r)
    rib = _arc_band(g.rc, g.zc, g.r_s + t / 2 - 0.3, g.r_s + t / 2 + c.rib_depth, a_lo,
                    c.blade_a_top - 12, g.z_bot)
    for xr in c.rib_x:
        a = -90 + np.degrees(np.arcsin(xr / c.blade_plan_r))
        parts.append(_centre_on(_revolve(rib, rib_deg, 8), a, g.y_axis))
    blade = mf.Manifold.batch_boolean(parts, ADD)

    # mounting block behind the centre, fused to the back of the moldboard
    a = np.radians(np.linspace(a_lo, c.blade_a_top, 50))
    r_front = g.r_s - t / 2 + 0.4               # stays inside the moldboard thickness
    behind = [(g.rc + r_front * np.cos(q), g.zc + r_front * np.sin(q)) for q in a]
    behind = behind + [(400.0, behind[-1][1]), (400.0, behind[0][1])]
    behind = _clip_poly(behind, g.z_bot)
    region = _centre_on(_revolve(behind, g.span_deg + 2), -90, g.y_axis)
    y_mount = p.y_front + c.mount_gap
    blk = box(-c.block_half_w, c.block_half_w, y_mount, g.y_mid0 + 2.0, g.z_bot, c.rail_top + 1.8)
    blade = blade + (blk ^ region)

    # dovetail channels (open at the bottom, closed at rail_top -> the blade hangs on the rails)
    ch = []
    for xr in c.rail_x:
        poly = dovetail_poly(xr, p.y_front, c.rail_root, c.rail_tip, c.rail_depth, grow=c.dt_clear)
        ch.append(prism_xy(poly, g.z_bot - 1, c.rail_top))
    blade = blade - mf.Manifold.batch_boolean(ch, ADD)
    return blade.simplify(0.002)                 # drop CSG slivers


# =================================================================================
# simplified HC-SR04 (pins up) for checks / pictures
# =================================================================================
def build_hcsr04(c: Combo) -> dict:
    yb, z0 = c.hc_y_back, c.hc_z0
    yf = yb + c.hc_t
    hw = c.hc_w / 2
    d = {}
    d["pcb"] = box(-hw, hw, yb, yf, z0, z0 + c.hc_h)
    tr = []
    for s in (-1, 1):
        cyl = mf.Manifold.cylinder(c.hc_tr_h, c.hc_tr_d / 2, c.hc_tr_d / 2, 64).rotate([-90, 0, 0])
        tr.append(cyl.translate([s * c.hc_tr_dx, yf, z0 + c.hc_h / 2]))
    d["transducers"] = mf.Manifold.batch_boolean(tr, ADD)
    d["back"] = box(-14, 14, yb - c.hc_back, yb, z0 + 3, z0 + 17) + \
        box(-5.08, 5.08, yb - c.hc_back, yb, z0 + c.hc_h - 2.5, z0 + c.hc_h)
    pins = [box(x - 0.32, x + 0.32, yb + 0.48, yb + 1.12, z0 + c.hc_h - 2, z0 + c.hc_h + 6)
            for x in (-3.81, -1.27, 1.27, 3.81)]
    d["pins"] = mf.Manifold.batch_boolean(pins, ADD)
    d["dupont"] = mf.Manifold.batch_boolean(
        [box(x - 1.25, x + 1.25, yb + 0.8 - 1.25, yb + 0.8 + 1.25, z0 + c.hc_h + 0.5, z0 + c.hc_h + 14.5)
         for x in (-3.81, -1.27, 1.27, 3.81)], ADD)
    return d


# =================================================================================
def export_print(m: mf.Manifold, path: Path) -> trimesh.Trimesh:
    t = to_trimesh(m)
    lo, hi = t.bounds
    t.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
    t.export(path)
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "kombo" / "STL"))
    ap.add_argument("--protrusion", type=float, nargs="+", default=[7.5, 9.0, 10.5])
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    asm = ROOT / "out_kombo"; asm.mkdir(exist_ok=True)
    c = Combo()
    report = {}
    for P in a.protrusion:
        p = combo_params(P)
        base = build_base(p, c)
        to_trimesh(base).export(asm / f"_asm_base_P{P:.1f}.stl")
        t = export_print(base, out / f"SMARS_kombo_baza_P{P:.1f}.stl")
        report[f"baza_P{P:.1f}"] = {"bbox": np.round(t.extents, 2).tolist(), "volume_cm3": round(t.volume / 1000, 2),
                                    "watertight": bool(t.is_watertight), "bodies": len(t.split())}
    p = combo_params(9.0)
    blade = build_blade(p, c)
    to_trimesh(blade).export(asm / "_asm_blade.stl")
    tb = export_print(blade, out / "SMARS_kombo_kurek.stl")
    g = blade_geom(p, c)
    report["kurek"] = {"bbox": np.round(tb.extents, 2).tolist(), "volume_cm3": round(tb.volume / 1000, 2),
                       "watertight": bool(tb.is_watertight), "bodies": len(tb.split()),
                       "profile_radius": round(g.r_s, 2), "z_bottom_as_built": round(g.z_bot, 3),
                       "droop_comp_mm": round(g.droop, 3)}
    for k in (7.5, 10.5):        # the blade must be identical for every P (rails at the same absolute z)
        assert abs(blade_geom(combo_params(k), c).z_bot - g.z_bot) < 1e-9
    hc = build_hcsr04(c)
    trimesh.util.concatenate([to_trimesh(v) for v in hc.values()]).export(asm / "_asm_hcsr04.stl")
    for k, v in report.items():
        print(k, json.dumps(v))
    (asm / "_report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
