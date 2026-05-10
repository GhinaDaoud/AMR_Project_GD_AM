"""
Convert pharmacy/meshes/model.glb from KHR_materials_pbrSpecularGlossiness
to standard pbrMetallicRoughness. Uses baseColorFactor boost for brightness
instead of emissiveFactor (emissive crashes Ogre's material cleanup on exit).

Run from the project root:
    python tools/fix_pharmacy_glb.py
"""

import json
import struct
from pathlib import Path

MAGIC = b"glTF"
GLB_HEADER_SIZE = 12
CHUNK_HEADER_SIZE = 8
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN  = 0x004E4942


def read_glb(path: Path):
    data = path.read_bytes()
    if data[:4] != MAGIC:
        raise ValueError(f"Not a GLB file: {path}")
    version, total_len = struct.unpack_from("<II", data, 4)
    offset = GLB_HEADER_SIZE
    chunks = []
    while offset < total_len:
        chunk_len, chunk_type = struct.unpack_from("<II", data, offset)
        chunk_data = data[offset + CHUNK_HEADER_SIZE: offset + CHUNK_HEADER_SIZE + chunk_len]
        chunks.append((chunk_type, chunk_data))
        offset += CHUNK_HEADER_SIZE + chunk_len
    return version, chunks


def write_glb(path: Path, version: int, chunks):
    parts = [MAGIC, struct.pack("<II", version, 0)]
    for chunk_type, chunk_data in chunks:
        pad = (4 - len(chunk_data) % 4) % 4
        pad_byte = b" " if chunk_type == CHUNK_JSON else b"\x00"
        chunk_data = chunk_data + pad_byte * pad
        parts.append(struct.pack("<II", len(chunk_data), chunk_type))
        parts.append(chunk_data)
    body = b"".join(parts)
    body = body[:8] + struct.pack("<I", len(body)) + body[12:]
    path.write_bytes(body)


def specular_gloss_to_metalrough(material: dict) -> dict:
    ext = material.get("extensions", {}).get("KHR_materials_pbrSpecularGlossiness", {})
    if not ext:
        return material

    pbr = {}
    if "diffuseFactor" in ext:
        # Boost factor to 1.3x for brightness without emissive (emissive crashes Ogre on exit)
        df = ext["diffuseFactor"]
        pbr["baseColorFactor"] = [min(df[0] * 1.3, 1.0), min(df[1] * 1.3, 1.0),
                                   min(df[2] * 1.3, 1.0), df[3]]
    else:
        pbr["baseColorFactor"] = [1.0, 1.0, 1.0, 1.0]

    if "diffuseTexture" in ext:
        pbr["baseColorTexture"] = ext["diffuseTexture"]

    pbr["roughnessFactor"] = 0.6
    pbr["metallicFactor"] = 0.0

    converted = {k: v for k, v in material.items() if k != "extensions"}
    converted["pbrMetallicRoughness"] = pbr

    remaining_exts = {k: v for k, v in material.get("extensions", {}).items()
                      if k != "KHR_materials_pbrSpecularGlossiness"}
    if remaining_exts:
        converted["extensions"] = remaining_exts

    return converted


def fix_glb(src: Path, dst: Path):
    version, chunks = read_glb(src)
    new_chunks = []
    converted_count = 0

    for chunk_type, chunk_data in chunks:
        if chunk_type != CHUNK_JSON:
            new_chunks.append((chunk_type, chunk_data))
            continue

        gltf = json.loads(chunk_data.decode("utf-8").rstrip("\x00"))

        for i, mat in enumerate(gltf.get("materials", [])):
            new_mat = specular_gloss_to_metalrough(mat)
            if new_mat is not mat:
                converted_count += 1
                print(f"  Material [{i}] '{mat.get('name', '')}': converted")
            gltf["materials"][i] = new_mat

        for key in ("extensionsUsed", "extensionsRequired"):
            if key in gltf:
                gltf[key] = [e for e in gltf[key]
                             if e != "KHR_materials_pbrSpecularGlossiness"]
                if not gltf[key]:
                    del gltf[key]

        new_json = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
        new_chunks.append((CHUNK_JSON, new_json))

    if converted_count == 0:
        print("No KHR_materials_pbrSpecularGlossiness materials found.")
        return

    write_glb(dst, version, new_chunks)
    print(f"Wrote {dst}  ({dst.stat().st_size:,} bytes)")


if __name__ == "__main__":
    root = Path(__file__).resolve().parent.parent
    src = root / "models" / "pharmacy" / "meshes" / "model.glb"
    dst = root / "models" / "pharmacy" / "meshes" / "model_fixed.glb"
    if not src.exists():
        raise FileNotFoundError(f"Source not found: {src}")
    print(f"Converting {src} ...")
    fix_glb(src, dst)
