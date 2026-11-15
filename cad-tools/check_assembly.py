"""Build the full SMARS assembly (chassis, wheels, 32 track links, new mount, 3 TCRT5000
modules), run interference / clearance checks and write preview meshes."""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
import manifold3d as mf

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import (Params, build_mount, build_module, to_trimesh, from_trimesh,  # noqa: E402
                        AXLE_Y, AXLE_Z, WHEEL_APOTHEM, GROUND_Z_LOW, GROUND_Z_HIGH)

ROOT = Path(__file__).resolve().parent.parent
PARTS = ROOT / "_parts"
WHEEL_X = 40.2                      # wheel / track centre plane (outside the 29 mm side walls)


def link_pose(theta_deg, centre_yz, r):
    t = np.radians(theta_deg)
    s, c = np.sin(t), np.cos(t)
    M = np.eye(4)
    M[:3, :3] = [[1, 0, 0], [0, s, c], [0, -c, s]]          # local y -> tangent, local z -> outward normal
    M[1, 3] = centre_yz[0] + r * c
    M[2, 3] = centre_yz[1] + r * s
    return M


def build_running_gear():
    link = trimesh.load(PARTS / "mechanical_track.stl")
    mw = trimesh.load(PARTS / "master_wheel.stl")
    sw = trimesh.load(PARTS / "slave_wheel_SL.stl")
    wheels, links = [], []
    r_link = WHEEL_APOTHEM + 3.0                               # link centre = 3 mm above the octagon face
    for sx in (-1, 1):
        for sy in (-1, 1):
            w = (mw if sx * sy < 0 else sw).copy()
            w.apply_translation([sx * WHEEL_X, sy * AXLE_Y, AXLE_Z])
            wheels.append(w)
        poses = []
        for th in (-90, -45, 0, 45, 90):
            poses.append(link_pose(th, (AXLE_Y, AXLE_Z), r_link))
        for th in (90, 135, 180, 225, 270):
            poses.append(link_pose(th, (-AXLE_Y, AXLE_Z), r_link))
        for y in (-AXLE_Y / 2, 0.0, AXLE_Y / 2):
            for th, z in ((-90, AXLE_Z - r_link), (90, AXLE_Z + r_link)):
                M = link_pose(th, (0, 0), 0)
                M[1, 3], M[2, 3] = y, z
                poses.append(M)
        for M in poses:
            l = link.copy(); l.apply_transform(M); l.apply_translation([sx * WHEEL_X, 0, 0])
            links.append(l)
    return wheels, links


def main(P=9.0):
    p = Params(protrusion=P)
    mount = build_mount(p)
    chassis_t = trimesh.load(PARTS / "chassis_SL.stl")
    chassis = from_trimesh(chassis_t)
    wheels, links = build_running_gear()
    gear = mf.Manifold.batch_boolean([from_trimesh(m) for m in wheels + links], mf.OpType.Add)

    res = {}
    res["mount_x_chassis_overlap_mm3"] = round((mount ^ chassis).volume(), 4)
    res["mount_chassis_min_gap_mm"] = round(mount.min_gap(chassis, 5.0), 3)
    res["mount_x_wheels_tracks_overlap_mm3"] = round((mount ^ gear).volume(), 4)
    res["mount_wheels_tracks_min_gap_mm"] = round(mount.min_gap(gear, 20.0), 3)
    res["track_lowest_z"] = round(float(min(l.bounds[0][2] for l in links)), 3)
    res["ground_z_low"] = GROUND_Z_LOW

    mods = [build_module(p, xc) for xc in p.pocket_centres()]
    overlaps = {}
    for k in mods[0]:
        allk = mf.Manifold.batch_boolean([m[k] for m in mods], mf.OpType.Add)
        overlaps[k] = round((mount ^ allk).volume(), 4)
    res["mount_x_module_overlap_mm3"] = overlaps
    sens = mf.Manifold.batch_boolean([m["sensor"] for m in mods], mf.OpType.Add)
    res["sensor_mount_min_gap_mm"] = round(mount.min_gap(sens, 5.0), 3)
    sb = to_trimesh(sens).bounds
    res["sensor_tip_z"] = round(float(sb[0][2]), 3)
    res["sensor_gap_floor_min_mm"] = round(float(sb[0][2] - GROUND_Z_LOW), 3)
    res["sensor_gap_floor_max_mm"] = round(float(sb[0][2] - GROUND_Z_HIGH), 3)
    mb = to_trimesh(mount).bounds
    res["mount_lowest_z"] = round(float(mb[0][2]), 3)
    res["mount_floor_gap_min_mm"] = round(float(mb[0][2] - GROUND_Z_LOW), 3)

    # at rest: gravity tips the part nose-down about the outer top edge of the front wall
    th = float(np.degrees(p.droop_angle))
    pivot = np.array([0.0, 35.0, -1.5])

    def droop(m):
        return m.translate(-pivot).rotate([-th, 0, 0]).translate(pivot)

    mount_r = droop(mount)
    sens_r = droop(sens)
    res["rest_droop_deg"] = round(th, 3)
    res["rest_mount_x_chassis_overlap_mm3"] = round((mount_r ^ chassis).volume(), 4)
    res["rest_rib_to_wall_gap_mm"] = round(mount_r.min_gap(chassis, 2.0), 3)
    tip_rest = float(to_trimesh(sens_r).bounds[0][2])
    res["rest_sensor_gap_floor_min_mm"] = round(tip_rest - GROUND_Z_LOW, 3)
    res["rest_sensor_gap_floor_max_mm"] = round(tip_rest - GROUND_Z_HIGH, 3)
    print(json.dumps(res, indent=2))

    # preview meshes
    out = ROOT / "_preview"; out.mkdir(exist_ok=True)
    chassis_t.export(out / "chassis.stl")
    trimesh.util.concatenate(wheels).export(out / "wheels.stl")
    trimesh.util.concatenate(links).export(out / "tracks.stl")
    to_trimesh(mount).export(out / f"mount_P{P:.1f}.stl")
    pcb = trimesh.util.concatenate([to_trimesh(m["pcb"]) for m in mods])
    pcb.export(out / "modules_pcb.stl")
    comp = trimesh.util.concatenate([to_trimesh(m[k]) for m in mods for k in ("sensor", "pot", "ic", "header")])
    comp.export(out / "modules_parts.stl")
    g = trimesh.creation.box([150, 190, 0.4]); g.apply_translation([0, 10, GROUND_Z_LOW - 0.2])
    g.export(out / "ground.stl")
    return res


if __name__ == "__main__":
    P = float(sys.argv[1]) if len(sys.argv) > 1 else 9.0
    main(P)
