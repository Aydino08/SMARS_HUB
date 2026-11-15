"""Full assembly check for the kombo (TCRT5000 x3 + HC-SR04 + blade) on the SMARS chassis."""
import json
import sys
from pathlib import Path

import manifold3d as mf
import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import build_module, to_trimesh, from_trimesh, GROUND_Z_LOW, GROUND_Z_HIGH  # noqa: E402
from make_combo import Combo, combo_params, build_base, build_blade, build_hcsr04, blade_geom  # noqa: E402
from check_assembly import build_running_gear  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
ADD = mf.OpType.Add
PIVOT = np.array([0.0, 35.0, -1.5])


def union(ms):
    return mf.Manifold.batch_boolean(list(ms), ADD)


def vol(a, b):
    return round((a ^ b).volume(), 4)


def gap(a, b, search=20.0):
    return round(a.min_gap(b, search), 3)


def beam_hits(c: Combo, rest_deg, targets: trimesh.Trimesh, half_angle=15.0, n=900, reach=400.0):
    """Cast rays from both transducer faces inside a cone; count rays that hit our own robot."""
    rng = np.random.default_rng(1)
    origins, dirs = [], []
    yf = c.hc_y_back + c.hc_t + c.hc_tr_h + 0.05
    zc = c.hc_z0 + c.hc_h / 2
    for s in (-1, 1):
        for _ in range(n):
            r = (c.hc_tr_d / 2) * np.sqrt(rng.random()); a = 2 * np.pi * rng.random()
            o = np.array([s * c.hc_tr_dx + r * np.cos(a), yf, zc + r * np.sin(a)])
            th = np.radians(half_angle) * np.sqrt(rng.random()); ph = 2 * np.pi * rng.random()
            d = np.array([np.sin(th) * np.cos(ph), np.cos(th), np.sin(th) * np.sin(ph)])
            origins.append(o); dirs.append(d)
    origins, dirs = np.array(origins), np.array(dirs)
    # apply the at-rest droop to the sensor too
    R = trimesh.transformations.rotation_matrix(np.radians(-rest_deg), [1, 0, 0], PIVOT)
    origins = trimesh.transform_points(origins, R)
    dirs = dirs @ R[:3, :3].T
    loc, idx_ray, _ = targets.ray.intersects_location(origins, dirs, multiple_hits=False)
    if len(idx_ray) == 0:
        return 0, len(origins)
    dist = np.linalg.norm(loc - origins[idx_ray], axis=1)
    return int((dist < reach).sum()), len(origins)


def beam_hits_from(centres, R, targets, radius=8.0, half_angle=15.0, n=900, reach=400.0):
    """Rays from transducer faces (centres = Nx3 points on the face, facing +Y) inside a cone."""
    rng = np.random.default_rng(1)
    origins, dirs = [], []
    for cc in centres:
        for _ in range(n):
            r = radius * np.sqrt(rng.random()); a = 2 * np.pi * rng.random()
            origins.append(cc + [r * np.cos(a), 0.0, r * np.sin(a)])
            th = np.radians(half_angle) * np.sqrt(rng.random()); ph = 2 * np.pi * rng.random()
            dirs.append([np.sin(th) * np.cos(ph), np.cos(th), np.sin(th) * np.sin(ph)])
    origins = trimesh.transform_points(np.array(origins), R)
    dirs = np.array(dirs) @ R[:3, :3].T
    loc, idx, _ = targets.ray.intersects_location(origins, dirs, multiple_hits=False)
    if len(idx) == 0:
        return 0, len(origins)
    return int((np.linalg.norm(loc - origins[idx], axis=1) < reach).sum()), len(origins)


def main(P=9.0, write_preview=True):
    c = Combo()
    p = combo_params(P)
    base = build_base(p, c)
    blade = build_blade(p, c)
    hc = build_hcsr04(c)
    hc_all = union(hc.values())
    chassis = from_trimesh(trimesh.load(ROOT / "_parts" / "chassis_SL.stl"))
    wheels, links = build_running_gear()
    gear = union(from_trimesh(m) for m in wheels + links)
    mods = [build_module(p, xc) for xc in p.pocket_centres()]
    mod = {k: union(m[k] for m in mods) for k in mods[0]}
    mod_all = union(mod.values())

    r = {"P": P}
    r["base^chassis"] = vol(base, chassis); r["base~chassis_gap"] = gap(base, chassis, 3)
    r["base^gear"] = vol(base, gear); r["base~gear_gap"] = gap(base, gear)
    r["base^modules"] = {k: vol(base, v) for k, v in mod.items()}
    r["base^hcsr04"] = {k: vol(base, v) for k, v in hc.items()}
    r["hcsr04^chassis"] = vol(hc_all, chassis); r["hcsr04~chassis_gap"] = gap(hc_all, chassis)
    r["hcsr04^modules"] = vol(hc_all, mod_all); r["hcsr04~modules_gap"] = gap(hc_all, mod_all)
    r["hcsr04^gear"] = vol(hc_all, gear)
    r["blade^base"] = vol(blade, base); r["blade~base_gap"] = gap(blade, base, 3)
    r["blade^chassis"] = vol(blade, chassis)
    r["blade^gear"] = vol(blade, gear); r["blade~gear_gap"] = gap(blade, gear, 40)
    r["blade^modules"] = vol(blade, mod_all); r["blade~modules_gap"] = gap(blade, mod_all)
    r["blade^hcsr04"] = vol(blade, hc_all)

    # clearances of the HC-SR04 region
    hb = to_trimesh(hc["back"]).bounds
    r["hcsr04_back_components_min_y"] = round(float(hb[0][1]), 2)
    r["wire_gap_behind_hcsr04_to_chassis_plane_mm"] = round(float(hb[0][1]) - 35.0, 2)
    r["cable_tunnel_height_mm"] = round(c.hc_z0 - 9.0, 2)
    bt = to_trimesh(base)
    above = bt.vertices[bt.vertices[:, 2] > 16.5]
    r["base_min_y_above_chassis_top"] = round(float(above[:, 1].min()), 2)

    # at rest (gravity droop)
    th = float(np.degrees(p.droop_angle))

    def droop(m):
        return m.translate(-PIVOT).rotate([-th, 0, 0]).translate(PIVOT)

    blade_r, sens_r, base_r = droop(blade), droop(mod["sensor"]), droop(base)
    r["rest_droop_deg"] = round(th, 3)
    r["rest_base^chassis"] = vol(base_r, chassis)
    zb = float(to_trimesh(blade_r).bounds[0][2])
    r["rest_blade_floor_gap_min_mm"] = round(zb - GROUND_Z_LOW, 3)
    r["rest_blade_floor_gap_max_mm"] = round(zb - GROUND_Z_HIGH, 3)
    zs = float(to_trimesh(sens_r).bounds[0][2])
    r["rest_tcrt_floor_gap_min_mm"] = round(zs - GROUND_Z_LOW, 3)
    r["rest_tcrt_floor_gap_max_mm"] = round(zs - GROUND_Z_HIGH, 3)
    zc = float(to_trimesh(base_r).bounds[0][2])
    r["rest_cassette_floor_gap_mm"] = round(zc - GROUND_Z_LOW, 3)
    g = blade_geom(p, c)
    r["blade_top_above_floor_mm"] = round(g.z_top - GROUND_Z_LOW, 2)
    r["hcsr04_axis_above_floor_mm"] = round(c.hc_z0 + c.hc_h / 2 - GROUND_Z_LOW, 2)

    # ultrasonic beam: does a 15 deg (half angle) cone hit our own robot?
    own = trimesh.util.concatenate([to_trimesh(x) for x in (blade_r, base_r, droop(mod_all))] +
                                   [to_trimesh(chassis), to_trimesh(gear)])
    hits, total = beam_hits(c, th, own)
    r["us_beam_rays_hitting_robot"] = f"{hits}/{total}"
    hits20, total20 = beam_hits(c, th, own, half_angle=20.0)
    r["us_beam_rays_hitting_robot_20deg"] = f"{hits20}/{total20}"
    print(json.dumps(r, indent=2, ensure_ascii=False))

    if write_preview:
        out = ROOT / "_preview_kombo"; out.mkdir(exist_ok=True)
        to_trimesh(base).export(out / "base.stl")
        to_trimesh(blade).export(out / "blade.stl")
        to_trimesh(hc_all).export(out / "hcsr04.stl")
        to_trimesh(mod["pcb"]).export(out / "tcrt_pcb.stl")
        to_trimesh(union([mod["sensor"], mod["pot"], mod["ic"], mod["header"]])).export(out / "tcrt_parts.stl")
        to_trimesh(mod["dupont"]).export(out / "tcrt_dupont.stl")
    return r


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 9.0)
