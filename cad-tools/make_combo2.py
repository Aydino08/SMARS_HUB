"""
Kombo v2:  TCRT5000 x3 base (no HC-SR04 posts)  +  blade that carries a copy of the SMARS
chassis extension slot on its top, so the ORIGINAL SMARS ultrasonic enclosure
(ultrasonic_2b_V2 + ultrasonic_1 with the HC-SR04 inside) drops onto the blade exactly as it
drops onto the chassis.

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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import (build_module, box, yz_prism, to_trimesh, from_trimesh,  # noqa: E402
                        GROUND_Z_LOW, GROUND_Z_HIGH, AXLE_Y)
from make_combo import (Combo, combo_params, build_base, build_blade, blade_geom,  # noqa: E402
                        _clip_poly, _revolve, _centre_on, export_print)
from check_assembly import build_running_gear  # noqa: E402
from check_combo import beam_hits_from, union, vol, gap, PIVOT  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
PARTS = ROOT / "_parts"
ADD = mf.OpType.Add

# pose of the original enclosure parts when they sit in the CHASSIS front slot
U2_T = np.array([0.0, 30.625 + 11.07, -1.5 + 5.25])      # ultrasonic_2b_V2 (verified: 0 mm^3 with chassis)
U1_BACK = 30.625 + 6.75                                   # ultrasonic_1 back face = front of the slot frame
U1_ZC = U2_T[2]                                           # arm slots of the shell are centred on the arms


def u1_matrix(dy=0.0, dz=0.0):
    """local (x, y, z) of ultrasonic_1  ->  chassis: X = x, Y = y0 - z, Z = zc + y."""
    M = np.eye(4)
    M[:3, :3] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    M[1, 3] = U1_BACK + 8.05 + dy
    M[2, 3] = U1_ZC + dz
    return M


@dataclass
class Socket:
    dy: float = 35.5        # slot copy = chassis slot moved forward ...
    dz: float = 15.5        # ... and up (enclosure bottom 1 mm above the blade top lip)
    back: float = 3.0       # solid block behind the 2 mm wall (frame of ultrasonic_2b rests on it)
    deck_top: float = 3.0
    half_w: float = 26.0


def enclosure(dy=0.0, dz=0.0):
    u2 = trimesh.load(PARTS / "ultrasonic_2b_V2.stl"); u2.apply_translation(U2_T + [0, dy, dz])
    u1 = trimesh.load(PARTS / "ultrasonic_1.stl"); u1.apply_transform(u1_matrix(dy, dz))
    return u1, u2


def build_blade_v2(p, c: Combo, s: Socket) -> mf.Manifold:
    blade = build_blade(p, c)
    g = blade_geom(p, c)
    wy1 = 35.0 + s.dy                    # outer face of the copied wall
    wy0 = wy1 - 2.0
    wt = -1.5 + s.dz                     # wall top (enclosure frame rests here)
    yb = wy0 - s.back
    blk_top = c.rail_top + 1.8           # top of the dovetail block
    y_blk = p.y_front + c.mount_gap      # back face of the dovetail block

    def under(y):                        # ~50 deg underside, rising backwards from the block
        return blk_top + 1.2 * (y_blk - y)

    hw = s.half_w
    parts = [
        # block behind / under the wall (top = wall top)
        yz_prism([(yb, wt), (wy1, wt), (wy1, under(wy1)), (yb, under(yb))], -hw, hw),
        # shoulders, 2 mm thick, 10.5 mm tall - identical to the chassis
        box(-hw, -21.25, wy0, wy1, wt - 0.01, wt + 10.5),
        box(21.25, hw, wy0, wy1, wt - 0.01, wt + 10.5),
    ]
    # deck + 45 deg wedge joining the block to the moldboard (kept below the enclosure)
    y_far = g.y_mid0 + 14
    bw = c.block_half_w                  # centre: sits on the dovetail block
    wedge = yz_prism([(wy1 - 0.01, s.deck_top), (y_far, s.deck_top), (y_far, blk_top),
                      (y_blk, blk_top), (wy1 - 0.01, under(wy1))], -bw, bw)
    for x0, x1 in ((-hw, -bw + 0.01), (bw - 0.01, hw)):   # sides: solid down to the bed
        wedge = wedge + yz_prism([(y_blk, s.deck_top), (y_far, s.deck_top), (y_far, g.z_bot),
                                  (y_blk, g.z_bot)], x0, x1)
        wedge = wedge + yz_prism([(wy1 - 0.01, s.deck_top), (y_blk + 0.01, s.deck_top),
                                  (y_blk + 0.01, blk_top), (wy1 - 0.01, under(wy1))], x0, x1)
    t = c.blade_t
    a = np.radians(np.linspace(c.blade_a_bot - 20, c.blade_a_top, 50))
    r_front = g.r_s - t / 2 + 0.4
    behind = [(g.rc + r_front * np.cos(q), g.zc + r_front * np.sin(q)) for q in a]
    behind = _clip_poly(behind + [(400.0, behind[-1][1]), (400.0, behind[0][1])], g.z_bot)
    region = _centre_on(_revolve(behind, g.span_deg + 2), -90, g.y_axis)
    parts.append(wedge ^ region)
    out = blade + mf.Manifold.batch_boolean(parts, ADD)
    return out.simplify(0.002)


def main():
    c, s = Combo(), Socket()
    chassis_t = trimesh.load(PARTS / "chassis_SL.stl")
    chassis = from_trimesh(chassis_t)

    # 1) verify the inferred enclosure pose on the chassis itself
    u1c, u2c = enclosure()
    r = {"chassis_slot: u2^chassis": vol(from_trimesh(u2c), chassis),
         "chassis_slot: u1^chassis": vol(from_trimesh(u1c), chassis),
         "chassis_slot: u1^u2": vol(from_trimesh(u1c), from_trimesh(u2c))}

    out = ROOT / "kombo2" / "STL"; out.mkdir(parents=True, exist_ok=True)
    asm = ROOT / "out_kombo2"; asm.mkdir(exist_ok=True)
    p9 = combo_params(9.0)
    blade = build_blade_v2(p9, c, s)
    tb = export_print(blade, out / "SMARS_kombo2_kurek_yuvali.stl")
    to_trimesh(blade).export(asm / "blade.stl")
    r["blade"] = {"size": np.round(tb.extents, 1).tolist(), "cm3": round(tb.volume / 1000, 2),
                  "watertight": bool(tb.is_watertight), "bodies": len(tb.split())}

    u1, u2 = enclosure(s.dy, s.dz)
    U1, U2 = from_trimesh(u1), from_trimesh(u2)
    enc = U1 + U2
    u1.export(asm / "u1.stl"); u2.export(asm / "u2.stl")
    wheels, links = build_running_gear()
    gear = union(from_trimesh(m) for m in wheels + links)
    r["u2^blade"] = vol(U2, blade); r["u1^blade"] = vol(U1, blade)
    r["enclosure~blade_gap"] = gap(enc, blade, 5)
    r["u1~blade_gap"] = gap(U1, blade, 5)
    r["u1^u2"] = vol(U1, U2)
    r["enclosure^gear"] = vol(enc, gear); r["enclosure^chassis"] = vol(enc, chassis)
    for dz in (-0.1,):
        r["u2 lowered 0.1 -> ^blade (must be >0: it rests on the wall top)"] = vol(U2.translate([0, 0, dz]), blade)

    for P in (7.5, 9.0, 10.5):
        p = combo_params(P)
        base = build_base(p, c, posts=False)
        tp = export_print(base, out / f"SMARS_kombo2_baza_P{P:.1f}.stl")
        to_trimesh(base).export(asm / f"base_P{P:.1f}.stl")
        mods = [build_module(p, xc) for xc in p.pocket_centres()]
        mod = union(union(m.values()) for m in mods)
        sens = union(m["sensor"] for m in mods)
        th = float(np.degrees(p.droop_angle))

        def droop(m):
            return m.translate(-PIVOT).rotate([-th, 0, 0]).translate(PIVOT)

        own = trimesh.util.concatenate([to_trimesh(droop(x)) for x in (blade, base, mod)] +
                                       [chassis_t, to_trimesh(gear)])
        R = trimesh.transformations.rotation_matrix(np.radians(-th), [1, 0, 0], PIVOT)
        hits, tot = beam_hits_from(np.array([[sx * c.hc_tr_dx, U1_BACK + s.dy + 16.1 + 0.05, U1_ZC + s.dz]
                                             for sx in (-1, 1)]), R, own)
        r[f"P{P}"] = {
            "base": {"size": np.round(tp.extents, 1).tolist(), "cm3": round(tp.volume / 1000, 2),
                     "watertight": bool(tp.is_watertight), "bodies": len(tp.split())},
            "base^chassis": vol(base, chassis), "base^gear": vol(base, gear),
            "blade^base": vol(blade, base), "blade~base_gap": gap(blade, base, 3),
            "blade^modules": vol(blade, mod), "blade~modules_gap": gap(blade, mod, 10),
            "enclosure^base": vol(enc, base), "enclosure^modules": vol(enc, mod),
            "enclosure~modules_gap": gap(enc, mod, 20),
            "blade^gear": vol(blade, gear),
            "rest_tcrt_gap_mm": round(float(to_trimesh(droop(sens)).bounds[0][2]) - GROUND_Z_LOW, 2),
            "rest_blade_gap_mm": round(float(to_trimesh(droop(blade)).bounds[0][2]) - GROUND_Z_LOW, 2),
            "us_axis_above_floor_mm": round(U1_ZC + s.dz - GROUND_Z_LOW, 1),
            "us_beam_rays_hitting_robot_15deg": f"{hits}/{tot}",
        }
    print(json.dumps(r, indent=2, ensure_ascii=False))
    (asm / "report.json").write_text(json.dumps(r, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
