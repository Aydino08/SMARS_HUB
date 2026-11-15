"""
v4: blade only + HC-SR04 in its ORIGINAL SMARS enclosure (ultrasonic_1 + ultrasonic_2b_V2).
One printed part:  chassis slot frame -> arm plate -> curved blade,  plus a copy of the chassis
slot on top so the original enclosure drops onto it unchanged.
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
from make_mount import (box, yz_prism, to_trimesh, from_trimesh, GROUND_Z_LOW, GROUND_Z_HIGH,  # noqa: E402
                        WALL_Y0, WALL_Y1, WALL_TOP_Z, FR_X, FR_Y0, FR_Z0, FR_Z1, NOTCH_X, NOTCH_Y0, NOTCH_Y1)
from make_combo import _clip_poly, _revolve, _centre_on, _arc_band, export_print  # noqa: E402
from make_combo2 import enclosure, U1_BACK, U1_ZC  # noqa: E402
from check_assembly import build_running_gear  # noqa: E402
from check_combo import beam_hits_from, union, vol, gap, PIVOT  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
PARTS = ROOT / "_parts"
ADD = mf.OpType.Add


@dataclass
class K4:
    # bracket (same chassis interface as the TCRT mount)
    half_w: float = 25.15
    arm_y0: float = WALL_Y1 + 0.25
    arm_t: float = 3.35
    seat_y: float = 33.8
    contact_rib: float = 0.20
    contact_rib_z: tuple = (-10.0, -8.0)
    # blade
    plan_r: float = 120.0
    half_w_blade: float = 50.0
    t: float = 2.4
    h: float = 30.0
    a_bot: float = -35.0
    a_top: float = 40.0
    clear: float = 2.5
    back_y: float = 46.5         # moldboard back surface at the centre
    end_plate: float = 5.0
    end_t: float = 2.0
    # support between arm plate and blade
    web_x: tuple = (-23.0, -8.0, 8.0, 23.0)
    web_t: float = 2.4
    deck_z: tuple = (0.0, 3.0)
    # copy of the chassis slot for the enclosure
    dy: float = 4.7               # enclosure frame back just in front of the chassis plane
    dz: float = 15.5              # enclosure bottom above the blade top lip

    @property
    def arm_y1(self):
        return self.arm_y0 + self.arm_t

    @property
    def droop_angle(self):        # same contact-rib limit as the TCRT mount
        free = self.arm_y0 - WALL_Y1
        lever = WALL_TOP_Z - 0.5 * sum(self.contact_rib_z)
        return min(max(free - self.contact_rib, 0) / lever, (WALL_Y0 - NOTCH_Y0) / (FR_Z1 - WALL_TOP_Z))


def blade_geom(c: K4):
    r_s = c.h / (np.sin(np.radians(c.a_top)) - np.sin(np.radians(c.a_bot)))
    y_mid0 = c.back_y + c.t / 2
    y_edge = y_mid0 + r_s * (1 - np.cos(np.radians(c.a_bot)))
    z_bot = GROUND_Z_LOW + c.clear + (y_edge - WALL_Y1) * np.sin(c.droop_angle)
    return dict(r_s=r_s, y_mid0=y_mid0, y_axis=y_mid0 + c.plan_r, rc=c.plan_r - r_s,
                zc=z_bot - r_s * np.sin(np.radians(c.a_bot)), z_bot=z_bot,
                span=2 * np.degrees(np.arcsin(c.half_w_blade / c.plan_r)))


def build(c: K4) -> mf.Manifold:
    g = blade_geom(c)
    hw, zb = c.half_w, g["z_bot"]
    # ---- frame + arm plate (chassis interface) ----
    solid = [box(-FR_X, FR_X, FR_Y0, FR_Y1_ := 37.375, FR_Z0, FR_Z1),
             box(-hw, hw, c.arm_y0, c.arm_y1, zb, FR_Z1),
             box(-20.0, 20.0, c.arm_y0 - c.contact_rib, c.arm_y0 + 0.5, *c.contact_rib_z)]
    # ---- copy of the chassis slot on top of the arm plate ----
    wy1 = WALL_Y1 + c.dy; wy0 = wy1 - 2.0; wt = WALL_TOP_Z + c.dz
    solid += [box(-26, 26, c.arm_y0, wy1, FR_Z1 - 0.01, wt),
              box(-26, -21.25, wy0, wy1, wt - 0.01, wt + 10.5), box(21.25, 26, wy0, wy1, wt - 0.01, wt + 10.5)]
    # ---- moldboard ----
    a_lo = c.a_bot - 20
    band = _arc_band(g["rc"], g["zc"], g["r_s"] - c.t / 2, g["r_s"] + c.t / 2, a_lo, c.a_top, zb)
    solid.append(_centre_on(_revolve(band, g["span"]), -90, g["y_axis"]))
    end_deg = np.degrees(c.end_t / c.plan_r)
    fin = _arc_band(g["rc"], g["zc"], g["r_s"] - c.t / 2 - c.end_plate, g["r_s"] + c.t / 2 - 0.4,
                    a_lo, c.a_top - 0.5, zb)
    for s in (-1, 1):
        solid.append(_centre_on(_revolve(fin, end_deg, 8), -90 + s * (g["span"] / 2 - end_deg / 2 - 0.05), g["y_axis"]))
    # ---- webs + deck between arm plate and blade, trimmed to behind the moldboard front ----
    a = np.radians(np.linspace(a_lo, c.a_top, 50))
    rf = g["r_s"] - c.t / 2 + 0.4
    behind = [(g["rc"] + rf * np.cos(q), g["zc"] + rf * np.sin(q)) for q in a]
    behind = _clip_poly(behind + [(400.0, behind[-1][1]), (400.0, behind[0][1])], zb)
    region = _centre_on(_revolve(behind, g["span"] + 2), -90, g["y_axis"])
    sup = [box(x - c.web_t / 2, x + c.web_t / 2, c.arm_y1 - 0.5, g["y_mid0"] + 15, zb, c.deck_z[1]) for x in c.web_x]
    sup.append(box(-hw, hw, c.arm_y1 - 0.5, g["y_mid0"] + 15, *c.deck_z))
    part = mf.Manifold.batch_boolean(solid, ADD) + (mf.Manifold.batch_boolean(sup, ADD) ^ region)
    # ---- cuts: notches + 45 deg chamfer under the inner half of the frame ----
    cut = [box(NOTCH_X, FR_X + 5, NOTCH_Y0, NOTCH_Y1, FR_Z0 - 5, FR_Z1 + 5),
           box(-FR_X - 5, -NOTCH_X, NOTCH_Y0, NOTCH_Y1, FR_Z0 - 5, FR_Z1 + 5),
           yz_prism([(c.seat_y, FR_Z0 - 0.01), (FR_Y0 - 2, FR_Z0 - 0.01), (FR_Y0 - 2, FR_Z0 + c.seat_y - FR_Y0 + 2)],
                    -FR_X - 6, FR_X + 6)]
    part = part - mf.Manifold.batch_boolean(cut, ADD)
    return part.simplify(0.01)


def main():
    c = K4()
    out = ROOT / "kurek4" / "STL"; out.mkdir(parents=True, exist_ok=True)
    asm = ROOT / "out_kurek4"; asm.mkdir(exist_ok=True)
    part = build(c)
    tp = export_print(part, out / "SMARS_kurek_HCSR04.stl"); to_trimesh(part).export(asm / "part.stl")
    chassis_t = trimesh.load(PARTS / "chassis_SL.stl"); chassis = from_trimesh(chassis_t)
    wheels, links = build_running_gear()
    gear = union(from_trimesh(m) for m in wheels + links)
    u1, u2 = enclosure(c.dy, c.dz); u1.export(asm / "u1.stl"); u2.export(asm / "u2.stl")
    U1, U2 = from_trimesh(u1), from_trimesh(u2); enc = U1 + U2
    th = float(np.degrees(c.droop_angle))

    def droop(m):
        return m.translate(-PIVOT).rotate([-th, 0, 0]).translate(PIVOT)

    own = trimesh.util.concatenate([to_trimesh(droop(part)), chassis_t, to_trimesh(gear)])
    R = trimesh.transformations.rotation_matrix(np.radians(-th), [1, 0, 0], PIVOT)
    hits, tot = beam_hits_from(np.array([[sx * 13.0, U1_BACK + c.dy + 16.1 + 0.05, U1_ZC + c.dz] for sx in (-1, 1)]), R, own)
    v = to_trimesh(part).vertices
    tm = tp
    r = {"part": {"size": np.round(tm.extents, 1).tolist(), "cm3": round(tm.volume / 1000, 2),
                  "watertight": bool(tm.is_watertight), "bodies": len(tm.split())},
         "part^chassis": vol(part, chassis), "part~chassis_gap": gap(part, chassis, 3),
         "part_lowered_0.1^chassis(>0 = seated)": vol(part.translate([0, 0, -0.1]), chassis),
         "part^tracks_wheels": vol(part, gear), "part~tracks_wheels_gap": gap(part, gear, 20),
         "enclosure^part": vol(enc, part), "u1~part_gap": gap(U1, part, 5),
         "u2_lowered_0.1^part(>0 = rests on wall)": vol(U2.translate([0, 0, -0.1]), part),
         "enclosure^chassis": vol(enc, chassis), "enclosure~chassis_gap": gap(enc, chassis, 5),
         "enclosure^tracks": vol(enc, gear),
         "enclosure_min_y_above_z16.5": round(float(u2.vertices[u2.vertices[:, 2] > 16.5][:, 1].min()), 2),
         "rest_blade_floor_gap_mm": round(float(to_trimesh(droop(part)).bounds[0][2]) - GROUND_Z_LOW, 2),
         "rest_blade_floor_gap_max_mm": round(float(to_trimesh(droop(part)).bounds[0][2]) - GROUND_Z_HIGH, 2),
         "us_axis_above_floor_mm": round(U1_ZC + c.dz - GROUND_Z_LOW, 1),
         "us_beam_hits_15deg": f"{hits}/{tot}",
         "front_max_y": round(float(v[:, 1].max()), 1),
         "front_centre_y": round(float(v[np.abs(v[:, 0]) < 2][:, 1].max()), 1),
         "enclosure_front_y": round(float(u1.bounds[1][1]), 1)}
    print(json.dumps(r, indent=2, ensure_ascii=False))
    (asm / "report.json").write_text(json.dumps(r, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
