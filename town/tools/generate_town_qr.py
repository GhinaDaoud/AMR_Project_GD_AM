#!/usr/bin/env python3
"""
generate_town_qr.py
-------------------
1. Generates 9 QR code PNGs.
2. Creates a proper Gazebo model directory for each sign
   (model.config + model.sdf + Ogre .material script + QR texture).
   Ogre material scripts are the only reliable texture method with ogre1.
3. Injects <include> tags into mixed_town.world.

Panel: 0.52 x 0.52 m board (30% up from 0.4), board centre at Z=0.55 m.
       Wooden pole 0.06 x 0.06 x 0.45 m below it.

Requirements:  pip install "qrcode[pil]" Pillow
Run:           python tools/generate_town_qr.py
"""

import os, shutil, re
import qrcode
from PIL import Image

# ── paths ─────────────────────────────────────────────────────────────────────
_ROOT      = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_FILE = os.path.join(_ROOT, "mixed_town.world")
MODELS_DIR = os.path.join(_ROOT, "models")

# ── landmark table ─────────────────────────────────────────────────────────────
# (id, qr_content, sign_x, sign_y, yaw_rad)
#
# Building extents (from world comments + known model sizes ~6x6 m houses):
#   supermarket  : X[7.3,18.1]  Y[1.6,4.4]   → south face Y=1.6 → sign Y=0.8
#   restaurant   : X[10.5,17.5] Y[-8,-2]      → north face Y=-2  → sign Y=-1.2
#   pharmacy     : X[-15.5,-8.5] Y[2.5,9.5]   → east  face X=-8.5→ sign X=-7.5
#   fire_station : X[-15.5,-8.5] Y[-9.5,-2.5] → east  face X=-8.5→ sign X=-7.5
#   house_north_1: centre(-6,13) south face Y≈10 → sign Y=11
#   house_north_2: centre(-6,19) south face Y≈16 → sign Y=17
#   house_south_1: centre(1,18)  south face Y≈15 → sign Y=16.5
#   house_south_2: centre(-6,-14) north face Y≈-11→ sign Y=-12
#   house_south_3: centre(6,-14)  north face Y≈-11→ sign Y=-12
#
# yaw=0      → board face points ±Y  (robot sees it from N or S)
# yaw=1.5708 → board face points ±X  (robot sees it from E or W)

LANDMARKS = [
    # id              qr_content      x       y      yaw
    #
    # QR is on the sign's local -Y face.  The sign yaw rotates that face into world coords:
    #   yaw=0       → QR faces world -Y (south) — visible from south (robot approaching from S)
    #   yaw=π       → QR faces world +Y (north) — visible from north
    #   yaw=π/2     → QR faces world +X (east)  — visible from east
    #   yaw=-π/2    → QR faces world -X (west)  — visible from west
    #
    # Signs are placed BETWEEN each building and the docking station (origin).
    # North houses  → sign to east  (docking is SE): yaw=π/2, QR faces east
    # South houses  → sign to north (docking is NE): yaw=π,   QR faces north
    # West services → sign to south (docking is NE): yaw=0,   QR faces south
    # Supermarket   → sign to south (docking is SW): yaw=0,   QR faces south
    # Restaurant    → sign to north (docking is NW): yaw=π,   QR faces north

    ("house_1",      "house_1",      -2.5,  13.0,   1.5708 ),  # east of house_north_1 (unchanged)
    ("house_2",      "house_2",      -2.5,  19.0,   1.5708 ),  # east of house_north_2 (unchanged)
    ("house_3",      "house_3",       1.0,  15.0,  -1.5708 ),  # opposite 90°, 1.5 m closer to house_south_1
    ("house_4",      "house_4",      -2.5, -14.0,   1.5708 ),  # east face (front) of house_south_2
    ("house_5",      "house_5",       3.5, -14.0,  -1.5708 ),  # west face (front) of house_south_3
    ("pharmacy",     "pharmacy",    -12.0,   1.5,   0.0    ),  # south of pharmacy (unchanged)
    ("fire_station", "fire_station",-12.0,  -3.5,   3.14159),  # north of fire_station (unchanged)
    ("restaurant",   "restaurant",   14.0,  -1.5,   3.14159),  # north of restaurant (front faces north)
    ("supermarket",  "supermarket",  12.5,   2.0,   0.0    ),  # 1 m closer to supermarket south face
]

QR_PNG_SIZE = 512   # pixels

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

# ── Ogre material script ───────────────────────────────────────────────────────

def make_material_script(mat_name: str, tex_filename: str) -> str:
    return f"""material {mat_name}
{{
  technique
  {{
    pass
    {{
      ambient  1 1 1 1
      diffuse  1 1 1 1
      specular 0.1 0.1 0.1 1

      texture_unit
      {{
        texture {tex_filename}
        filtering trilinear
        max_anisotropy 4
      }}
    }}
  }}
}}
"""

# ── model.config ──────────────────────────────────────────────────────────────

def make_model_config(sign_id: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>sign_{sign_id}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>QR sign post for {sign_id}</description>
</model>
"""

# ── model.sdf ─────────────────────────────────────────────────────────────────

def make_model_sdf(sign_id: str, mat_name: str) -> str:
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="sign_{sign_id}">
    <static>true</static>
    <link name="link">

      <!-- wooden pole: 0.45 m tall, centre Z=0.225 -->
      <visual name="pole">
        <pose>0 0 0.225 0 0 0</pose>
        <geometry><box><size>0.06 0.06 0.45</size></box></geometry>
        <material>
          <ambient>0.35 0.22 0.10 1</ambient>
          <diffuse>0.35 0.22 0.10 1</diffuse>
        </material>
      </visual>
      <collision name="pole_col">
        <pose>0 0 0.225 0 0 0</pose>
        <geometry><box><size>0.06 0.06 0.45</size></box></geometry>
      </collision>

      <!-- QR board body: white backing (0.52 x 0.52 m), centre Z=0.58 -->
      <visual name="board_body">
        <pose>0 0 0.58 0 0 0</pose>
        <geometry><box><size>0.52 0.02 0.52</size></box></geometry>
        <material>
          <ambient>0.95 0.95 0.95 1</ambient>
          <diffuse>0.95 0.95 0.95 1</diffuse>
        </material>
      </visual>
      <!-- QR face: ultra-thin overlay on local -Y side only.
           The white board body hides the +Y (back) face of this overlay,
           so the QR is effectively one-sided (front face only). -->
      <visual name="board_qr">
        <pose>0 -0.011 0.58 0 0 0</pose>
        <geometry><box><size>0.50 0.001 0.50</size></box></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <pbr>
            <metal>
              <albedo_map>model://sign_{sign_id}/materials/textures/qr.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="board_col">
        <pose>0 0 0.58 0 0 0</pose>
        <geometry><box><size>0.52 0.02 0.52</size></box></geometry>
      </collision>

    </link>
  </model>
</sdf>
"""

# ── world patching ─────────────────────────────────────────────────────────────

def patch_world(includes: list[str]):
    with open(WORLD_FILE, "r", encoding="utf-8") as f:
        world = f.read()

    # Remove old inline <model> blocks injected by previous script versions
    # (matched by the <!-- QR sign: ... --> comment pattern)
    world = re.sub(
        r"\n    <!-- QR sign:.*?</model>", "", world, flags=re.DOTALL
    )

    # Remove old <include> blocks from previous runs of this script
    world = re.sub(
        r"\n    <!-- QR SIGNS -->.*?<!-- END QR SIGNS -->\n", "", world, flags=re.DOTALL
    )

    block = "\n    <!-- QR SIGNS -->\n" + "\n".join(includes) + "\n    <!-- END QR SIGNS -->\n"
    world = world.replace("  </world>", block + "\n  </world>")

    shutil.copy2(WORLD_FILE, WORLD_FILE + ".bak")
    with open(WORLD_FILE, "w", encoding="utf-8") as f:
        f.write(world)

# ── main ──────────────────────────────────────────────────────────────────────

def main():
    includes = []

    for (lid, content, x, y, yaw) in LANDMARKS:
        model_dir  = os.path.join(MODELS_DIR, f"sign_{lid}")
        tex_dir    = os.path.join(model_dir, "materials", "textures")
        script_dir = os.path.join(model_dir, "materials", "scripts")
        os.makedirs(tex_dir,    exist_ok=True)
        os.makedirs(script_dir, exist_ok=True)

        # QR PNG
        tex_file = "qr.png"
        tex_path = os.path.join(tex_dir, tex_file)
        make_qr_png(content, tex_path)
        print(f"  [QR] {lid} -> {tex_path}")

        # Ogre material script
        mat_name   = f"Sign/{lid}"
        script_path = os.path.join(script_dir, "board.material")
        with open(script_path, "w") as f:
            f.write(make_material_script(mat_name, tex_file))

        # model.config
        with open(os.path.join(model_dir, "model.config"), "w") as f:
            f.write(make_model_config(lid))

        # model.sdf
        with open(os.path.join(model_dir, "model.sdf"), "w") as f:
            f.write(make_model_sdf(lid, mat_name))

        # world include snippet
        includes.append(
            f"    <include>\n"
            f"      <name>sign_{lid}</name>\n"
            f"      <uri>model://sign_{lid}</uri>\n"
            f"      <pose>{x} {y} 0 0 0 {yaw}</pose>\n"
            f"    </include>"
        )

    print("\nPatching mixed_town.world ...")
    patch_world(includes)
    print("Done. Launch:  bash launch_gazebo.sh")

if __name__ == "__main__":
    main()
