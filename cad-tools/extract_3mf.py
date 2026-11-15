"""Extract the SMARS meshes from a Bambu Studio SMARS.3mf into separate STL files
(each part in its own local coordinates) and print their bounding boxes.

usage:  python extract_3mf.py path/to/SMARS.3mf
"""
import sys
import zipfile
from pathlib import Path

import numpy as np
import trimesh
from lxml import etree

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "_parts"
NS = {"m": "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"}

# 3D/Objects/<file> -> part name (from Metadata/model_settings.config of SMARS.3mf)
NAMES = {
    "object_1.model": "mechanical_track",
    "object_33.model": "chassis_SL",
    "object_34.model": "holding_board_9V",
    "object_35.model": "master_wheel",
    "object_36.model": "slave_wheel_SL",
    "object_71.model": "ultrasonic_1",
    "object_72.model": "ultrasonic_2b_V2",
    "object_73.model": "connector_a",
    "object_74.model": "ir_sensor_a",
    "object_110.model": "Remorque",
}


def load_mesh(data: bytes) -> trimesh.Trimesh:
    tree = etree.fromstring(data)
    verts = tree.xpath("//m:vertex", namespaces=NS)
    tris = tree.xpath("//m:triangle", namespaces=NS)
    v = np.array([[float(e.get("x")), float(e.get("y")), float(e.get("z"))] for e in verts])
    f = np.array([[int(e.get("v1")), int(e.get("v2")), int(e.get("v3"))] for e in tris])
    return trimesh.Trimesh(v, f, process=True)


def main(path_3mf: str):
    OUT.mkdir(exist_ok=True)
    with zipfile.ZipFile(path_3mf) as z:
        for fname, name in NAMES.items():
            m = load_mesh(z.read(f"3D/Objects/{fname}"))
            m.export(OUT / f"{name}.stl")
            lo, hi = m.bounds
            print(f"{name:20s} faces={len(m.faces):7d} watertight={m.is_watertight} "
                  f"size={np.round(hi - lo, 2)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else str(ROOT.parent / "uploads" / "SMARS.3mf"))
