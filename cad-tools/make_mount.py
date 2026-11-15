"""
SMARS  -  3 x TCRT5000 line-sensor mount for the front extension slot of chassis_SL.

Every coordinate below is in the SMARS chassis_SL frame (mm), exactly as the chassis
is stored in SMARS.3mf:   +Y = forward,  +Z = up,  X = left/right,  origin = chassis centre.

All SMARS numbers were measured from the meshes inside SMARS.3mf (see README):
  * front wall          y = 33.0 .. 35.0, centre part top at z = -1.5
  * shoulders           |x| >= 21.25, top at z = 9.0
  * axles               y = +-26.75, z = -8.5
  * wheels              octagon, apothem 14.25 (flat), 15.42 (corner)
  * track link          6.0 thick (inner face -> tread top)
  * extension frame     46.2 x 6.75 x 10.5, end notches 2.5 wide  (connector_a / ultrasonic_2b)
TCRT5000 data: Vishay datasheet (body 10.2 x 5.8, 7.0 high, peak distance 2.5 mm) and the
common 4-pin LM393 module (PCB 32-32.5 x 14-14.2 x 1.6, sensor centre 3 mm from PCB end).
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass, asdict, field
from pathlib import Path

import manifold3d as mf
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parent.parent

# ----------------------------------------------------------------------------------
# SMARS geometry (measured from SMARS.3mf)
# ----------------------------------------------------------------------------------
AXLE_Z = -8.5
AXLE_Y = 26.75
WHEEL_APOTHEM = 14.25          # axle -> flat face of the octagonal wheel
WHEEL_CORNER = 15.42           # axle -> corner of the octagon
TRACK_T = 6.0                  # link inner face -> tread top
GROUND_Z_LOW = AXLE_Z - WHEEL_APOTHEM - TRACK_T     # -28.75 : floor when chassis is lowest
GROUND_Z_HIGH = AXLE_Z - WHEEL_CORNER - TRACK_T     # -29.92 : floor when chassis is highest

WALL_Y0, WALL_Y1 = 33.0, 35.0  # front wall (inner / outer face)
WALL_TOP_Z = -1.5              # top of the centre part of the front wall
SHOULDER_X = 21.25             # inner edge of the two shoulders
SHOULDER_TOP_Z = 9.0

# extension frame, identical to connector_a / ultrasonic_2b_V2 (same pose, verified 0 mm^3 overlap)
FR_X = 23.09
FR_Y0, FR_Y1 = 30.625, 37.375
FR_Z0, FR_Z1 = WALL_TOP_Z, SHOULDER_TOP_Z
NOTCH_X = 20.85
NOTCH_Y0, NOTCH_Y1 = 32.75, 35.25


# ----------------------------------------------------------------------------------
# parameters
# ----------------------------------------------------------------------------------
@dataclass
class Params:
    # optics
    gap: float = 3.5               # sensor lens tip -> floor, with the chassis at its LOWEST point
    protrusion: float = 9.0        # P: PCB bottom face -> tip of TCRT5000 lenses (measure yours!)

    # TCRT5000 module (4-pin LM393 board)
    pcb_l: float = 32.5
    pcb_w: float = 14.2
    pcb_t: float = 1.6
    sensor_x: float = 10.2         # TCRT5000 body across the PCB
    sensor_y: float = 5.8          # TCRT5000 body along the PCB
    sensor_h: float = 7.0
    sensor_from_end: float = 3.0   # sensor centre -> PCB end
    # tolerance band the sensor window must accept (different module makers)
    pcb_l_range: tuple = (31.8, 32.8)
    sensor_from_end_range: tuple = (2.6, 3.4)

    # module orientation: True  -> TCRT5000 at the FRONT, jumper pins at the back (towards chassis)
    #                     False -> TCRT5000 at the back, pins at the front
    sensor_front: bool = True

    # pockets
    pocket_w: float = 14.5         # PCB 14.2 + 0.15 per side (ribs make it a press fit)
    pocket_len: float = 33.5       # used when sensor_front=False (open front)
    front_bar: float = 2.0         # solid floor strip in front of the sensor windows (sensor_front=True)
    wall_in: float = 1.6           # wall between two pockets
    wall_out: float = 1.8
    rail_w: float = 1.2            # ledge under each long PCB edge
    under_gap: float = 3.0         # PCB bottom -> floor top (solder joints, header pin tails)
    top_wall: float = 1.6          # side walls rise this much above the PCB top
    floor_t: float = 1.6
    win_x: float = 11.2            # sensor window (10.2 body + 0.5 per side)
    win_y: float = 8.0             # window length when sensor_front=False
    win_margin: float = 0.3        # extra window length around the tolerance band (sensor_front=True)
    rib_r: float = 0.45            # crush-rib radius
    rib_protrude: float = 0.35     # how far a rib sticks into the pocket (14.2 PCB -> 0.2 mm squeeze/side)
    rib_pos: tuple = (8.0, 22.0)   # rib positions along the pocket (from the back wall)
    gusset: float = 6.0            # triangular stiffeners on the walls next to the arm plate

    # bracket
    arm_y0: float = WALL_Y1 + 0.25  # 0.25 mm clearance to the outer face of the front wall
    arm_t: float = 3.35
    seat_y: float = 33.8           # frame rests flat on the wall top for y >= seat_y, 45 deg chamfer behind
    front_chamfer: float = 1.2
    # contact rib on the back of the arm plate: leans on the flat outer face of the front wall
    # so the cantilevered cassette cannot droop more than ~0.4 deg
    contact_rib: float = 0.20      # protrusion -> 0.05 mm nominal gap to the wall
    contact_rib_z: tuple = (-10.0, -8.0)
    contact_rib_x: float = 20.0

    @property
    def n(self):
        return 3

    @property
    def pitch(self):
        return self.pocket_w + self.wall_in

    @property
    def half_w(self):
        return 1.5 * self.pocket_w + self.wall_in + self.wall_out

    # --- at-rest droop of the cantilevered cassette ------------------------------------
    # Gravity rotates the part nose-down about the outer top edge of the front wall
    # (y=35, z=-1.5) until the contact rib (or, without it, the back jaw of the notch)
    # touches the wall.  The sensor is raised by the resulting nominal drop.
    @property
    def droop_angle(self):
        free = self.arm_y0 - WALL_Y1
        lever_jaw = FR_Z1 - WALL_TOP_Z                       # back jaw touches shoulder inner face
        th_jaw = (WALL_Y0 - NOTCH_Y0) / lever_jaw            # 0.25 / 10.5
        if self.contact_rib > 0:
            lever = WALL_TOP_Z - 0.5 * (self.contact_rib_z[0] + self.contact_rib_z[1])
            th = max(free - self.contact_rib, 0.0) / lever
        else:
            lever = WALL_TOP_Z - (-10.5)
            th = free / lever
        return min(th, th_jaw)

    @property
    def sensor_y_nominal(self):
        if self.sensor_front:
            return self.y_back + 0.05 + self.pcb_l - self.sensor_from_end
        return self.y_back + 0.05 + self.sensor_from_end

    @property
    def droop_at_sensor(self):
        return (self.sensor_y_nominal - WALL_Y1) * np.sin(self.droop_angle)

    @property
    def tip_z(self):            # as built (un-drooped); at rest it sits droop_at_sensor lower
        return GROUND_Z_LOW + self.gap + self.droop_at_sensor

    @property
    def pcb_z(self):            # PCB bottom face
        return self.tip_z + self.protrusion

    @property
    def window_y(self):         # sensor window, relative to y_back
        if self.sensor_front:
            lo = self.pcb_l_range[0] - self.sensor_from_end_range[1] - self.sensor_y / 2 - self.win_margin
            hi = self.pcb_l_range[1] - self.sensor_from_end_range[0] + self.sensor_y / 2 + self.win_margin
            return lo + 0.05, hi + 0.05
        return 0.0, self.win_y

    @property
    def floor_z0(self):
        return self.pcb_z - self.under_gap - self.floor_t

    @property
    def floor_z1(self):
        return self.pcb_z - self.under_gap

    @property
    def wall_z1(self):
        return self.pcb_z + self.pcb_t + self.top_wall

    @property
    def arm_y1(self):
        return self.arm_y0 + self.arm_t

    @property
    def y_back(self):           # back stop of the PCB (front face of the arm plate)
        return self.arm_y1

    @property
    def y_front(self):
        if self.sensor_front:
            return self.y_back + self.window_y[1] + self.front_bar
        return self.arm_y1 + self.pocket_len

    def pocket_centres(self):
        return [(-1 + i) * self.pitch for i in range(self.n)]


# ----------------------------------------------------------------------------------
# CSG helpers
# ----------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1) -> mf.Manifold:
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def hull(points) -> mf.Manifold:
    return mf.Manifold.hull_points(np.asarray(points, dtype=np.float64))


def yz_prism(poly_yz, x0, x1) -> mf.Manifold:
    pts = [(x, y, z) for (y, z) in poly_yz for x in (x0, x1)]
    return hull(pts)


def vcyl(x, y, r, z0, z1, r_top=None, seg=24) -> mf.Manifold:
    c = mf.Manifold.cylinder(z1 - z0, r, r if r_top is None else r_top, seg)
    return c.translate([x, y, z0])


def to_trimesh(m: mf.Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)


def from_trimesh(t: trimesh.Trimesh) -> mf.Manifold:
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32),
                               tri_verts=np.asarray(t.faces, np.uint32)))


# ----------------------------------------------------------------------------------
# the printed part
# ----------------------------------------------------------------------------------
def build_mount(p: Params) -> mf.Manifold:
    hw = p.half_w
    solid = []

    # 1) extension frame (same envelope as connector_a), solid instead of slotted
    solid.append(box(-FR_X, FR_X, FR_Y0, FR_Y1, FR_Z0, FR_Z1))

    # 2) vertical arm plate in front of the chassis wall, frame top -> cassette floor
    solid.append(box(-hw, hw, p.arm_y0, p.arm_y1, p.floor_z0, FR_Z1))
    #    contact rib on its back face (leans on the flat outer face of the front wall)
    if p.contact_rib > 0:
        solid.append(box(-p.contact_rib_x, p.contact_rib_x, p.arm_y0 - p.contact_rib, p.arm_y0 + 0.5,
                         p.contact_rib_z[0], p.contact_rib_z[1]))

    # 3) cassette floor
    solid.append(box(-hw, hw, p.arm_y0, p.y_front, p.floor_z0, p.floor_z1))

    # 4) pocket walls + rails + crush ribs
    cs = p.pocket_centres()
    edges = []                  # x positions of pocket side faces
    for xc in cs:
        edges += [xc - p.pocket_w / 2, xc + p.pocket_w / 2]
    # outer walls
    wall_x = [(-hw, edges[0]), (edges[-1], hw)]
    # inner walls
    for i in range(p.n - 1):
        wall_x.append((edges[2 * i + 1], edges[2 * i + 2]))
    g = p.gusset
    for x0, x1 in wall_x:
        solid.append(box(x0, x1, p.arm_y0, p.y_front, p.floor_z0, p.wall_z1))
        # 45 deg gusset tying the wall to the arm plate
        solid.append(yz_prism([(p.arm_y1 - 0.5, p.wall_z1 - 0.01), (p.arm_y1 + g, p.wall_z1 - 0.01),
                               (p.arm_y1 - 0.5, p.wall_z1 + g + 0.5)], x0, x1))
    for xc in cs:
        a, b = xc - p.pocket_w / 2, xc + p.pocket_w / 2
        solid.append(box(a, a + p.rail_w, p.arm_y0, p.y_front, p.floor_z0, p.pcb_z))
        solid.append(box(b - p.rail_w, b, p.arm_y0, p.y_front, p.floor_z0, p.pcb_z))
        for side, xf in ((-1, a), (1, b)):
            cx = xf - side * (p.rib_protrude - p.rib_r)      # centre so that rib sticks in by rib_protrude
            for yy in p.rib_pos:
                y = p.y_back + yy
                z_top = p.wall_z1
                solid.append(vcyl(cx, y, p.rib_r, p.pcb_z, z_top - 1.0, seg=20))
                # tapered lead-in at the top of the rib
                solid.append(vcyl(cx, y, p.rib_r, z_top - 1.0, z_top,
                                  r_top=p.rib_r - p.rib_protrude + 0.02, seg=20))

    part = mf.Manifold.batch_boolean(solid, mf.OpType.Add)

    cut = []
    # 5) frame end notches (shoulders of the front wall slide in here)
    for s in (-1, 1):
        x0, x1 = (NOTCH_X, FR_X + 5) if s > 0 else (-FR_X - 5, -NOTCH_X)
        cut.append(box(x0, x1, NOTCH_Y0, NOTCH_Y1, FR_Z0 - 5, FR_Z1 + 5))
    # 6) 45 deg chamfer under the inner (chassis-side) half of the frame -> printable w/o supports
    cut.append(yz_prism([(p.seat_y, FR_Z0 - 0.01), (FR_Y0 - 2, FR_Z0 - 0.01),
                         (FR_Y0 - 2, FR_Z0 + (p.seat_y - FR_Y0 + 2))], -FR_X - 6, FR_X + 6))
    # 7) sensor windows through the floor
    w0, w1 = p.window_y
    for xc in cs:
        cut.append(box(xc - p.win_x / 2, xc + p.win_x / 2, p.y_back + w0, p.y_back + w1,
                       p.floor_z0 - 1, p.floor_z1 + 0.5))
    # 8) chamfer on the front-bottom edge (rides over small bumps)
    c = p.front_chamfer
    if c > 0:
        cut.append(yz_prism([(p.y_front + 0.01, p.floor_z0 - 0.01), (p.y_front - c, p.floor_z0 - 0.01),
                             (p.y_front + 0.01, p.floor_z0 + c + 0.01)], -hw - 1, hw + 1))

    part = part - mf.Manifold.batch_boolean(cut, mf.OpType.Add)
    return part


# ----------------------------------------------------------------------------------
# simplified TCRT5000 module model (for checking / pictures only)
# ----------------------------------------------------------------------------------
def build_module(p: Params, xc: float, header="vertical"):
    """Module with its PCB end against the back stop.  Positions are measured from the
    sensor end of the board ("u" = distance from the sensor end)."""
    y0 = p.y_back + 0.05                  # PCB end touches the back stop
    z0 = p.pcb_z
    L = p.pcb_l

    def Y(u):                             # board coordinate (from sensor end) -> chassis y
        return y0 + (L - u) if p.sensor_front else y0 + u

    def span(u0, u1):
        a, b = Y(u0), Y(u1)
        return min(a, b), max(a, b)

    parts = {}
    parts["pcb"] = box(xc - p.pcb_w / 2, xc + p.pcb_w / 2, y0, y0 + L, z0, z0 + p.pcb_t) - \
        vcyl(xc, Y(24.7), 1.6, z0 - 1, z0 + 3)
    sy = Y(p.sensor_from_end)
    tip = z0 - p.protrusion
    body = box(xc - p.sensor_x / 2, xc + p.sensor_x / 2, sy - p.sensor_y / 2, sy + p.sensor_y / 2,
               tip + 0.7, tip + p.sensor_h)
    lens = (vcyl(xc - 2.75, sy, 2.1, tip, tip + 0.8) + vcyl(xc + 2.75, sy, 2.1, tip, tip + 0.8))
    legs = []
    if p.protrusion > p.sensor_h:
        for dx in (-2.75, 2.75):
            for dy in (-1.27, 1.27):
                legs.append(box(xc + dx - 0.25, xc + dx + 0.25, sy + dy - 0.25, sy + dy + 0.25,
                                tip + p.sensor_h, z0))
    parts["sensor"] = mf.Manifold.batch_boolean([body, lens] + legs, mf.OpType.Add)
    top = z0 + p.pcb_t
    a, b = span(12.4, 19.2)
    parts["pot"] = box(xc - 3.4 + 1.0, xc + 3.4 + 1.0, a, b, top, top + 4.8)
    a, b = span(14.0, 19.0)
    parts["ic"] = box(xc - 5.5, xc - 1.5, a, b, top, top + 1.75)
    hy = Y(L - 2.3)                       # 4-pin header, 2.3 mm from the pin end of the board
    if header == "vertical":
        pins = [box(xc + dx - 0.32, xc + dx + 0.32, hy - 0.32, hy + 0.32, z0 - 1.5, top + 8.5)
                for dx in (-3.81, -1.27, 1.27, 3.81)]
        parts["header"] = mf.Manifold.batch_boolean(
            [box(xc - 5.08, xc + 5.08, hy - 1.27, hy + 1.27, top, top + 2.5)] + pins, mf.OpType.Add)
        # female Dupont jumper housings pushed onto the pins (2.54 x 2.54 x 14 mm)
        parts["dupont"] = mf.Manifold.batch_boolean(
            [box(xc + dx - 1.25, xc + dx + 1.25, hy - 1.25, hy + 1.25, top + 2.55, top + 16.5)
             for dx in (-3.81, -1.27, 1.27, 3.81)], mf.OpType.Add)
    parts["solder"] = mf.Manifold.batch_boolean(
        [vcyl(xc + dx, hy, 0.8, z0 - 1.8, z0) for dx in (-3.81, -1.27, 1.27, 3.81)] +
        [vcyl(xc + 1.0 + dx, Y(15.8), 0.8, z0 - 1.5, z0) for dx in (-2.5, 0, 2.5)], mf.OpType.Add)
    return parts


# ----------------------------------------------------------------------------------
def summary(p: Params, part: mf.Manifold) -> dict:
    t = to_trimesh(part)
    lo, hi = t.bounds
    rest = p.tip_z - p.droop_at_sensor
    return {
        "protrusion_P_mm": p.protrusion,
        "sensor_front": p.sensor_front,
        "droop_deg": round(float(np.degrees(p.droop_angle)), 3),
        "droop_at_sensor_mm": round(float(p.droop_at_sensor), 3),
        "sensor_tip_z_as_built": round(p.tip_z, 3),
        "gap_at_rest_min_mm": round(rest - GROUND_Z_LOW, 3),
        "gap_at_rest_max_mm": round(rest - GROUND_Z_HIGH, 3),
        "pcb_bottom_z": round(p.pcb_z, 3),
        "cassette_floor_clearance_mm": round(p.floor_z0 - GROUND_Z_LOW, 3),
        "sensor_below_cassette_mm": round(p.floor_z0 - p.tip_z, 3),
        "sensor_centres_x": [round(x, 2) for x in p.pocket_centres()],
        "sensor_centre_y": round(float(p.sensor_y_nominal), 2),
        "sensor_ahead_of_front_axle_mm": round(float(p.sensor_y_nominal - AXLE_Y), 2),
        "window_y_rel": [round(v, 2) for v in p.window_y],
        "bbox_mm": [round(v, 2) for v in (hi - lo)],
        "volume_cm3": round(t.volume / 1000, 2),
        "watertight": bool(t.is_watertight),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "out"))
    ap.add_argument("--protrusion", type=float, nargs="+", default=[7.5, 9.0, 10.5])
    ap.add_argument("--gap", type=float, default=3.5)
    ap.add_argument("--sensor-back", action="store_true",
                    help="old layout: TCRT5000 next to the chassis, jumper pins at the front")
    a = ap.parse_args()
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    report = {}
    for P in a.protrusion:
        p = Params(gap=a.gap, protrusion=P, sensor_front=not a.sensor_back)
        part = build_mount(p)
        t = to_trimesh(part)
        # assembly-pose copy (chassis coordinates) for checking
        t.export(out / f"_asm_mount_P{P:.1f}.stl")
        # print pose: as used (floor down), moved onto the bed and centred
        tp = t.copy()
        lo, hi = tp.bounds
        tp.apply_translation([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
        name = f"SMARS_TCRT5000x3_mount_P{P:.1f}.stl"
        tp.export(out / name)
        report[name] = summary(p, part)
        print(name, json.dumps(report[name]))
    (out / "_report.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
