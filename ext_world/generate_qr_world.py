#!/usr/bin/env python3
"""
generate_qr_world.py
--------------------
Generates 4 QR code PNGs and patches them as textures onto 4 existing box
obstacles inside ext_world/maze.world (tl_box, pillar_a, pillar_b,
diag_block_1).  The modified world is saved to ext_world/maze.world in-place
(a backup is written to ext_world/maze.world.bak first).

Requirements:
    pip install qrcode[pil] Pillow

Run:
    python generate_qr_world.py
"""

import os
import shutil
import xml.etree.ElementTree as ET

import qrcode
from PIL import Image

# ── configuration ─────────────────────────────────────────────────────────────

_HERE        = os.path.dirname(os.path.abspath(__file__))
MAZE_WORLD   = os.path.join(_HERE, "maze.world")
TEX_DIR      = os.path.join(_HERE, "qr_textures")

# Which boxes get a QR code, and what each QR encodes
QR_TARGETS = [
    ("tl_box",      "Zone-A: pink box (-7.5, 6)"),
    ("pillar_a",    "Zone-B: pillar A (0, 3)"),
    ("pillar_b",    "Zone-C: pillar B (0, -4)"),
    ("diag_block_1","Zone-D: diagonal block (6, 6)"),
]

QR_PNG_SIZE = 512   # pixels per side

# ── QR generation ─────────────────────────────────────────────────────────────

def make_qr_png(content: str, path: str):
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=10,
        border=4,
    )
    qr.add_data(content)
    qr.make(fit=True)
    img = qr.make_image(fill_color="black", back_color="white").convert("RGB")
    img = img.resize((QR_PNG_SIZE, QR_PNG_SIZE), Image.LANCZOS)
    img.save(path)
    print(f"  [QR] {path}  ← \"{content}\"")

# ── SDF material XML with PBR texture ─────────────────────────────────────────

def make_material_element(abs_tex_path: str) -> ET.Element:
    """
    Returns an XML <material> element that applies the given PNG as an
    albedo map via Ignition Gazebo's PBR metal workflow.
    """
    uri = "file://" + abs_tex_path.replace("\\", "/")

    mat = ET.Element("material")

    for tag, val in [("ambient", "1 1 1 1"), ("diffuse", "1 1 1 1"),
                     ("specular", "0.1 0.1 0.1 1")]:
        e = ET.SubElement(mat, tag)
        e.text = val

    pbr  = ET.SubElement(mat, "pbr")
    metal = ET.SubElement(pbr, "metal")
    alb  = ET.SubElement(metal, "albedo_map")
    alb.text = uri

    return mat

# ── XML patching ──────────────────────────────────────────────────────────────

def patch_world(tree: ET.ElementTree, targets: dict[str, str]):
    """
    targets: {model_name: absolute_texture_path}
    Replaces the <material> element inside each matching model's <visual>.
    """
    world = tree.getroot().find("world")
    if world is None:          # root IS the world element
        world = tree.getroot()

    patched = []
    for model in world.findall("model"):
        name = model.get("name", "")
        if name not in targets:
            continue

        link = model.find("link")
        if link is None:
            continue
        visual = link.find("visual")
        if visual is None:
            continue

        # Remove old <material> if present
        old_mat = visual.find("material")
        if old_mat is not None:
            visual.remove(old_mat)

        # Insert new textured material
        visual.append(make_material_element(targets[name]))
        patched.append(name)
        print(f"  [patch] applied QR texture to model '{name}'")

    missing = set(targets) - set(patched)
    if missing:
        print(f"  [warn]  models not found in world: {missing}")

# ── indent helper (Python < 3.9 has no ET.indent) ────────────────────────────

def _indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    # 1. Check input file exists
    if not os.path.isfile(MAZE_WORLD):
        raise FileNotFoundError(f"Cannot find {MAZE_WORLD} — run from the project root.")

    # 2. Create texture folder
    os.makedirs(TEX_DIR, exist_ok=True)

    # 3. Generate QR PNGs
    print("Generating QR codes …")
    tex_map: dict[str, str] = {}   # model_name → absolute PNG path
    for model_name, content in QR_TARGETS:
        png_name = f"qr_{model_name}.png"
        abs_path = os.path.abspath(os.path.join(TEX_DIR, png_name))
        make_qr_png(content, abs_path)
        tex_map[model_name] = abs_path

    # 4. Backup original world
    backup = MAZE_WORLD + ".bak"
    shutil.copy2(MAZE_WORLD, backup)
    print(f"\nBackup saved → {backup}")

    # 5. Parse, patch, write
    print("\nPatching maze.world …")
    ET.register_namespace("", "")
    tree = ET.parse(MAZE_WORLD)
    patch_world(tree, tex_map)

    # Pretty-print
    _indent(tree.getroot())
    tree.write(MAZE_WORLD, encoding="unicode", xml_declaration=True)

    # ET strips the <?xml?> version cleanly; prepend sdf declaration comment
    print(f"\nDone — modified world saved to {os.path.abspath(MAZE_WORLD)}")
    print("\nLaunch:")
    print(f'  ign gazebo "{os.path.abspath(MAZE_WORLD)}"')
    print("\nQR summary:")
    for model_name, content in QR_TARGETS:
        print(f"  {model_name:20s} → \"{content}\"")


if __name__ == "__main__":
    main()
