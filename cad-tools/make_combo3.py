"""
Kombo v3 (compact):  the blade is pulled back right in front of the TCRT5000 sensors, its rear
deck covers the sensors, and the ORIGINAL SMARS ultrasonic enclosure sits on that deck.
The blade hangs on two vertical dovetail rails on the OUTER side walls of the cassette.
Chassis coordinates (mm): +Y forward, +Z up.
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import manifold3d as mf
import numpy as np
import trimesh
from shapely.geometry import Polygon

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import (Params, build_mount, build_module, box, hull, yz_prism, to_trimesh,  # noqa: E402
                        from_trimesh, GROUND_Z_LOW, WALL_Y1)
from make_combo import _clip_poly, _revolve, _centre_on, _arc_band, export_print  # noqa: E402
from make_combo2 import enclosure, U1_BACK, U1_ZC  # noqa: E402
from check_assembly import build_running_gear  # noqa: E402
from check_combo import beam_hits_from, union, vol, gap, PIVOT  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
PARTS = ROOT / "_parts"
ADD = mf.OpType.Add


@dataclass
class C3:
    plan_r: float = 200.0          # gentle U in plan (wings ~6 mm forward)
    half_w: float = 50.0
    t: float = 2.4
    h: float = 30.0
    a_bot: float = -35.0
    a_top: float = 40.0
    clear: float = 2.5             # cutting edge above floor at rest
    back_min_gap: float = 0.6      # moldboard back <-> cassette front
    end_plate: float = 5.0
    end_t: float = 2.0
    rib_x: tuple = (-38.0, 38.0)
    # side dovetails (rails on the outer cassette walls, channels in the blade cheeks)
    rail_yc: float = 66.0
    rail_root: float = 6.0
    rail_tip: float = 8.4
    rail_depth: float = 2.5
    rail_top: float = -10.0        # blade hangs here (same for every P)
    dt_clear: float = 0.2
    cheek_gap: float = 0.15
    cheek_t: float = 4.0
    cheek_y0: float = 60.5
    # deck under the enclosure (50 deg underside, clear of the cassette)
    under_y0: float = 74.5
    under_z0: float = -10.0
    under_slope: float = 1.2
    deck_top: float = 3.0
    # copy of the chassis slot for the original ultrasonic enclosure
    dy: float = 28.5
    dz: float = 16.5
    back: float = 3.0


def params3(P: float) -> Params:
    return Params(protrusion=P)


def geom(p: Params, c: C3):
    r_s = c.h / (np.sin(np.radians(c.a_top)) - np.sin(np.radians(c.a_bot)))
    y_mid0 = p.y_front + c.back_min_gap + c.t / 2
    y_edge = y_mid0 + r_s * (1 - np.cos(np.radians(c.a_bot)))
    droop = (y_edge - WALL_Y1) * np.sin(p.droop_angle)
    z_bot = GROUND_Z_LOW + c.clear + droop
    zc = z_bot - r_s * np.sin(np.radians(c.a_bot))
    return dict(r_s=r_s, y_mid0=y_mid0, y_axis=y_mid0 + c.plan_r, rc=c.plan_r - r_s, zc=zc,
                z_bot=z_bot, span=2 * np.degrees(np.arcsin(c.half_w / c.plan_r)), droop=droop)


def dt_poly(s, x0, yc, root, tip, depth, grow=0.0):
    """dovetail on a side face at |x| = x0, protruding outwards (s = +-1)."""
    pts = [(s * (x0 - 0.4), yc - root / 2), (s * (x0 - 0.4), yc + root / 2),
           (s * (x0 + depth), yc + tip / 2), (s * (x0 + depth), yc - tip / 2)]
    if grow > 0:
        pts = [tuple(q) for q in np.array(Polygon(pts).buffer(grow, join_style=2, mitre_limit=3).exterior.coords)[:-1]]
    return pts


def prism(poly, z0, z1):
    return hull([(x, y, z) for (x, y) in poly for z in (z0, z1)])


def build_base3(p: Params, c: C3) -> mf.Manifold:
    base = build_mount(p)
    hw = p.half_w
    xin = p.pocket_w * 1.5 + p.wall_in              # 23.35
    add = []
    rz0 = p.floor_z0 + 0.3
    for s in (-1, 1):
        y0, y1 = c.rail_yc - c.rail_tip / 2 - 0.5, c.rail_yc + c.rail_tip / 2 + 0.5
        xa, xb = sorted((s * xin, s * hw))
        add.append(box(xa, xb, y0, y1, p.floor_z0, c.rail_top))          # wall raised to the rail top
        add.append(prism(dt_poly(s, hw, c.rail_yc, c.rail_root, c.rail_tip, c.rail_depth), rz0, c.rail_top))
    return mf.Manifold.batch_boolean([base] + add, ADD)


def build_blade3(p: Params, c: C3) -> mf.Manifold:
    g = geom(p, c)
    a_lo = c.a_bot - 20
    parts = []
    band = _arc_band(g["rc"], g["zc"], g["r_s"] - c.t / 2, g["r_s"] + c.t / 2, a_lo, c.a_top, g["z_bot"])
    parts.append(_centre_on(_revolve(band, g["span"]), -90, g["y_axis"]))
    end_deg = np.degrees(c.end_t / c.plan_r)
    fin = _arc_band(g["rc"], g["zc"], g["r_s"] - c.t / 2 - c.end_plate, g["r_s"] + c.t / 2 - 0.4,
                    a_lo, c.a_top - 0.5, g["z_bot"])
    for s in (-1, 1):
        parts.append(_centre_on(_revolve(fin, end_deg, 8), -90 + s * (g["span"] / 2 - end_deg / 2 - 0.05), g["y_axis"]))
    rib_deg = np.degrees(2.0 / c.plan_r)
    rib = _arc_band(g["rc"], g["zc"], g["r_s"] + c.t / 2 - 0.3, g["r_s"] + c.t / 2 + 4, a_lo, c.a_top - 12, g["z_bot"])
    for xr in c.rib_x:
        parts.append(_centre_on(_revolve(rib, rib_deg, 8), -90 + np.degrees(np.arcsin(xr / c.plan_r)), g["y_axis"]))

    a = np.radians(np.linspace(a_lo, c.a_top, 50))
    rf = g["r_s"] - c.t / 2 + 0.4
    behind = [(g["rc"] + rf * np.cos(q), g["zc"] + rf * np.sin(q)) for q in a]
    behind = _clip_poly(behind + [(400.0, behind[-1][1]), (400.0, behind[0][1])], g["z_bot"])
    region = _centre_on(_revolve(behind, g["span"] + 2), -90, g["y_axis"])

    def under(y):
        return c.under_z0 + c.under_slope * (c.under_y0 - y)

    wy1 = WALL_Y1 + c.dy
    wy0 = wy1 - 2.0
    wt = -1.5 + c.dz
    yb = wy0 - c.back
    y_far = g["y_mid0"] + 15
    hw = 26.0
    keep = [
        yz_prism([(yb, wt), (wy1, wt), (wy1, under(wy1)), (yb, under(yb))], -hw, hw),          # socket block
        box(-hw, -21.25, wy0, wy1, wt - 0.01, wt + 10.5), box(21.25, hw, wy0, wy1, wt - 0.01, wt + 10.5),
    ]
    back = [
        yz_prism([(wy1 - 0.01, c.deck_top), (y_far, c.deck_top), (y_far, under(y_far)),
                  (wy1 - 0.01, under(wy1))], -hw, hw),                                        # deck/wedge
    ]
    xc0 = p.half_w + c.cheek_gap
    for s in (-1, 1):
        xa, xb = sorted((s * xc0, s * (xc0 + c.cheek_t)))
        keep.append(box(xa, xb, c.cheek_y0, wy1, g["z_bot"], wt))
        back.append(box(xa, xb, wy1 - 0.01, y_far, g["z_bot"], c.deck_top))
    blade = (mf.Manifold.batch_boolean(parts + keep, ADD) +
             (mf.Manifold.batch_boolean(back, ADD) ^ region))
    ch = [prism(dt_poly(s, p.half_w, c.rail_yc, c.rail_root, c.rail_tip, c.rail_depth, grow=c.dt_clear),
                g["z_bot"] - 1, c.rail_top) for s in (-1, 1)]
    blade = blade - mf.Manifold.batch_boolean(ch, ADD)
    return blade.simplify(0.01)


def main():
    c = C3()
    chassis_t = trimesh.load(PARTS / "chassis_SL.stl"); chassis = from_trimesh(chassis_t)
    wheels, links = build_running_gear()
    gear = union(from_trimesh(m) for m in wheels + links)
    out = ROOT / "kombo3" / "STL"; out.mkdir(parents=True, exist_ok=True)
    asm = ROOT / "out_kombo3"; asm.mkdir(exist_ok=True)

    p9 = params3(9.0)
    blade = build_blade3(p9, c)
    tb = export_print(blade, out / "SMARS_kombo3_kurek_yuvali.stl"); to_trimesh(blade).export(asm / "blade.stl")
    u1, u2 = enclosure(c.dy, c.dz); u1.export(asm / "u1.stl"); u2.export(asm / "u2.stl")
    U1, U2 = from_trimesh(u1), from_trimesh(u2); enc = U1 + U2
    r = {"blade": {"size": np.round(tb.extents, 1).tolist(), "cm3": round(tb.volume / 1000, 2),
                   "watertight": bool(tb.is_watertight), "bodies": len(tb.split())},
         "blade_front_max_y": round(float(to_trimesh(blade).bounds[1][1]), 1),
         "enclosure_front_y": round(float(u1.bounds[1][1]), 1),
         "enclosure^blade": vol(enc, blade), "u1~blade_gap": gap(U1, blade, 5),
         "u2_lowered_0.1^blade(>0 = rests on wall)": vol(U2.translate([0, 0, -0.1]), blade),
         "enclosure^chassis": vol(enc, chassis), "enclosure^gear": vol(enc, gear),
         "blade^chassis": vol(blade, chassis), "blade^gear": vol(blade, gear), "blade~gear_gap": gap(blade, gear, 20)}
    for P in (7.5, 9.0, 10.5):
        p = params3(P)
        base = build_base3(p, c)
        tp = export_print(base, out / f"SMARS_kombo3_baza_P{P:.1f}.stl"); to_trimesh(base).export(asm / f"base_P{P:.1f}.stl")
        mods = [build_module(p, xc) for xc in p.pocket_centres()]
        mod = union(union(m.values()) for m in mods)
        sens = union(m["sensor"] for m in mods)
        th = float(np.degrees(p.droop_angle))

        def droop(m):
            return m.translate(-PIVOT).rotate([-th, 0, 0]).translate(PIVOT)

        own = trimesh.util.concatenate([to_trimesh(droop(x)) for x in (blade, base, mod)] + [chassis_t, to_trimesh(gear)])
        R = trimesh.transformations.rotation_matrix(np.radians(-th), [1, 0, 0], PIVOT)
        hits, tot = beam_hits_from(np.array([[sx * 13.0, U1_BACK + c.dy + 16.1 + 0.05, U1_ZC + c.dz] for sx in (-1, 1)]), R, own)
        r[f"P{P}"] = {
            "base": {"size": np.round(tp.extents, 1).tolist(), "watertight": bool(tp.is_watertight), "bodies": len(tp.split())},
            "base^chassis": vol(base, chassis), "base^gear": vol(base, gear),
            "blade^base": vol(blade, base), "blade~base_gap": gap(blade, base, 3),
            "blade^modules": vol(blade, mod), "blade~modules_gap": gap(blade, mod, 10),
            "enclosure^base": vol(enc, base), "enclosure^modules": vol(enc, mod),
            "rest_tcrt_gap_mm": round(float(to_trimesh(droop(sens)).bounds[0][2]) - GROUND_Z_LOW, 2),
            "rest_blade_gap_mm": round(float(to_trimesh(droop(blade)).bounds[0][2]) - GROUND_Z_LOW, 2),
            "us_axis_above_floor_mm": round(U1_ZC + c.dz - GROUND_Z_LOW, 1),
            "us_beam_hits_15deg": f"{hits}/{tot}",
        }
    print(json.dumps(r, indent=2, ensure_ascii=False))
    (asm / "report.json").write_text(json.dumps(r, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
