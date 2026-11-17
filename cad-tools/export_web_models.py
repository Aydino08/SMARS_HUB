"""Export decimated SMARS meshes + layout for the browser simulator.

Output (in the web project):  public/models/smars.bin  (float32 positions + uint32 indices)
                              public/models/smars.json (part table + layout measured from the CAD)
All coordinates: SMARS chassis frame, mm, +Y forward, +Z up.
"""
import json
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from make_mount import build_module, to_trimesh, GROUND_Z_LOW, AXLE_Y, AXLE_Z, WHEEL_APOTHEM  # noqa: E402
from make_combo3 import params3, C3  # noqa: E402
from make_kurek4 import K4  # noqa: E402
from make_combo2 import U1_BACK, U1_ZC  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
WEB = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT.parent / "smars-web" / "smars-hub" / "public" / "models"
WEB.mkdir(parents=True, exist_ok=True)
P = ROOT / "_parts"


def load(p):
    return trimesh.load(p, force="mesh")


def deci(m, faces):
    if len(m.faces) <= faces:
        return m
    d = m.simplify_quadric_decimation(face_count=faces)
    d.remove_unreferenced_vertices()
    return d


def hcsr04(dy, dz):
    """Schematic HC-SR04 board + 2 transducers inside the original enclosure."""
    front = U1_BACK + dy + 16.1           # front face of ultrasonic_1 (measured)
    zc = U1_ZC + dz
    parts = []
    for sx in (-1, 1):
        c = trimesh.creation.cylinder(radius=7.9, height=12.0, sections=40)
        c.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
        c.apply_translation([sx * 13.0, front - 6.4, zc])
        parts.append(c)
    trans = trimesh.util.concatenate(parts)
    pcb = trimesh.creation.box([45.0, 1.6, 20.0])
    pcb.apply_translation([0, front - 13.4, zc])
    return pcb, trans, front, zc


parts = {}
parts["chassis"] = (deci(load(P / "chassis_SL.stl"), 9000), "#e9edf0")
parts["wheel_master"] = (deci(load(P / "master_wheel.stl"), 5000), "#7cdc3a")
parts["wheel_slave"] = (deci(load(P / "slave_wheel_SL.stl"), 4000), "#7cdc3a")
parts["link"] = (deci(load(P / "mechanical_track.stl"), 700), "#8a929a")

# --- config A: TCRT5000 x3 + blade + original ultrasonic enclosure (kombo v3) ---
pA = params3(9.0)
cA = C3()
parts["A_base"] = (deci(load(ROOT / "out_kombo3" / "base_P9.0.stl"), 3000), "#3a3f45")
parts["A_blade"] = (deci(load(ROOT / "out_kombo3" / "blade.stl"), 7000), "#f2b705")
parts["A_u1"] = (deci(load(ROOT / "out_kombo3" / "u1.stl"), 3500), "#f4f6f8")
parts["A_u2"] = (load(ROOT / "out_kombo3" / "u2.stl"), "#8a929a")
mods = [build_module(pA, xc) for xc in pA.pocket_centres()]
parts["A_tcrt_pcb"] = (trimesh.util.concatenate([to_trimesh(m["pcb"]) for m in mods]), "#1d5fbf")
parts["A_tcrt_sensor"] = (trimesh.util.concatenate([to_trimesh(m["sensor"]) for m in mods]), "#15181b")
parts["A_tcrt_pot"] = (trimesh.util.concatenate([to_trimesh(m["pot"]) for m in mods]), "#2b7de9")
parts["A_tcrt_header"] = (trimesh.util.concatenate([to_trimesh(m[k]) for m in mods for k in ("header", "dupont")]), "#d9a400")
pcbA, trA, frontA, zcA = hcsr04(cA.dy, cA.dz)
parts["A_hc_pcb"] = (pcbA, "#1d5fbf")
parts["A_hc_trans"] = (trA, "#c9ced3")

# --- config B: blade + original ultrasonic enclosure, no IR (kurek v4) ---
cB = K4()
parts["B_part"] = (deci(load(ROOT / "out_kurek4" / "part.stl"), 7000), "#f2b705")
parts["B_u1"] = (deci(load(ROOT / "out_kurek4" / "u1.stl"), 3500), "#f4f6f8")
parts["B_u2"] = (load(ROOT / "out_kurek4" / "u2.stl"), "#8a929a")
pcbB, trB, frontB, zcB = hcsr04(cB.dy, cB.dz)
parts["B_hc_pcb"] = (pcbB, "#1d5fbf")
parts["B_hc_trans"] = (trB, "#c9ced3")


# --- şarj sistemi (smars-charger/_asm — dok olunmuş robotun chassis çərçivəsində) ---
CH = ROOT.parent / "smars-charger" / "_asm"
HAS_CHARGER = CH.exists()
if HAS_CHARGER:
    def ld(n):
        return trimesh.load(CH / f"{n}.stl", force="mesh")
    parts["R_shoe"] = (ld("şarj_quyruğu"), "#ff7a1a")
    parts["R_lid"] = (ld("quyruq_qapağı"), "#ffb066")
    parts["R_tape"] = (ld("mis_lent_kontaktlar"), "#d9822b")
    parts["R_screws"] = (ld("qapaq_vintləri"), "#b8bec4")
    parts["S_body"] = (ld("stansiya_gövdəsi"), "#2b3440")
    parts["S_cover"] = (ld("stansiya_alt_qapağı"), "#3a4652")
    parts["S_plunger"] = (trimesh.util.concatenate([ld("plunjer_vint_qayka"), ld("yay"), ld("kabel_ucluğu")]), "#c9ced3")
    parts["S_switch"] = (trimesh.util.concatenate([ld("mikrovaç"), ld("mikrovaç_ayaqları"), ld("mikrovaç_ləçəyi")]), "#1c1c1c")
    parts["S_led_red"] = (ld("LED_0"), "#ff2a2a")
    parts["S_led_green"] = (ld("LED_1"), "#29e05a")
    parts["S_usb"] = (ld("USB_kabel"), "#111111")


def front_profile(meshes):
    v = np.vstack([m.vertices for m in meshes])
    centre = v[np.abs(v[:, 0]) < 6]
    return dict(front_centre=round(float(centre[:, 1].max()), 2), front_max=round(float(v[:, 1].max()), 2),
                half_width=round(float(np.abs(v[:, 0]).max()), 2), bottom=round(float(v[:, 2].min()), 2))


blob = bytearray()
table = {}
for name, (m, color) in parts.items():
    v = np.asarray(m.vertices, dtype=np.float32)
    f = np.asarray(m.faces, dtype=np.uint32)
    vo = len(blob); blob += v.tobytes()
    io = len(blob); blob += f.tobytes()
    table[name] = {"v": [vo, len(v)], "i": [io, len(f) * 3], "color": color}
    print(f"{name:16s} faces={len(f):6d}")

layout = {
    "units": "mm, chassis frame: +X right, +Y forward, +Z up",
    "ground_z": GROUND_Z_LOW,
    "wheels": {"x": 40.2, "axle_y": AXLE_Y, "axle_z": AXLE_Z, "apothem": WHEEL_APOTHEM,
               "master": [[-40.2, 26.75], [40.2, -26.75]], "slave": [[40.2, 26.75], [-40.2, -26.75]]},
    "track": {"x": 40.2, "link_radius": WHEEL_APOTHEM + 3.0, "links_per_side": 16, "half_width": 8.0},
    "chassis": {"rear_y": -(AXLE_Y + WHEEL_APOTHEM + 6.0), "top_z": 16.5},
    "configs": {
        "A": {
            "name": "TCRT5000 ×3 + kürək + HC-SR04",
            "parts": ["A_base", "A_blade", "A_u1", "A_u2", "A_tcrt_pcb", "A_tcrt_sensor", "A_tcrt_pot", "A_tcrt_header", "A_hc_pcb", "A_hc_trans"],
            "ir": [{"x": 0.0, "y": float(pA.sensor_y_nominal)}, {"x": -pA.pitch, "y": float(pA.sensor_y_nominal)},
                   {"x": pA.pitch, "y": float(pA.sensor_y_nominal)}],
            "us": {"y": round(frontA, 2), "z": round(zcA, 2), "dx": 13.0},
            **front_profile([parts["A_blade"][0]]),
        },
        "B": {
            "name": "Kürək + HC-SR04 (IR-sız)",
            "parts": ["B_part", "B_u1", "B_u2", "B_hc_pcb", "B_hc_trans"],
            "ir": [],
            "us": {"y": round(frontB, 2), "z": round(zcB, 2), "dx": 13.0},
            **front_profile([parts["B_part"][0]]),
        },
    },
    "parts": table,
}
if HAS_CHARGER:
    prm = json.loads((CH.parent / "_params.json").read_text())
    st, sh = prm["station"], prm["shoe"]
    th, mo = prm["throat_half"], prm["mouth_half"]
    y0, y1, hw = st["y0"], st["y1"], st["hw"]
    # dok çərçivəsi = dok olunmuş robotun chassis çərçivəsi (stansiya −Y tərəfdə). F → chassis: (x, y) → (−x, −y)
    left = [(hw, -y0), (mo, -y0), (th, -st["funnel_y"]), (th, -y1), (hw, -y1)]
    layout["dock"] = {
        "robot_parts": ["R_shoe", "R_lid", "R_tape", "R_screws"],
        "station_parts": ["S_body", "S_cover", "S_plunger", "S_switch", "S_led_red", "S_led_green", "S_usb"],
        "station_polys": [left, [(-x, y) for x, y in left],
                          [(-th, -st["back_y"]), (th, -st["back_y"]), (th, -y1), (-th, -y1)]],
        "station_h": prm["roof_under"] + st["roof_t"] - prm["zf"],
        "tail": [-sh["tw2"], sh["tw2"], -sh["ty1"], -sh["arm_y0"]],
        "tail_end": -sh["ty1"],
        "contact_tail_end": -(st["py"] + (sh["ty1"] - sh["tape_y1"])),     # plunjer lentə çatır
        "switch_tail_end": -(st["back_y"] - 1.3),                           # mikrovaç işə düşür
        "back_stop": -st["back_y"],
        "throat_half": th,
        "line_start": -prm["line_start_y"],
        "t_y": sorted([-prm["t_y"][0], -prm["t_y"][1]]),
        "t_half": prm["t_half"],
        "spin_y": -prm["spin_y"],
    }
(WEB / "smars.bin").write_bytes(bytes(blob))
(WEB / "smars.json").write_text(json.dumps(layout, indent=1, ensure_ascii=False))
print("bytes", len(blob), "->", WEB)
print(json.dumps(layout["configs"], indent=1, ensure_ascii=False)[:1500])
