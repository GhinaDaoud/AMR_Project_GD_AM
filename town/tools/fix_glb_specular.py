"""
Convert KHR_materials_pbrSpecularGlossiness → pbrMetallicRoughness in any GLB.
Usage:  python tools/fix_glb_specular.py <input.glb> <output.glb>
"""
import json, struct, sys
from pathlib import Path

MAGIC = b"glTF"
CHUNK_JSON = 0x4E4F534A
CHUNK_BIN  = 0x004E4942


def read_glb(path: Path):
    data = path.read_bytes()
    if data[:4] != MAGIC:
        raise ValueError(f"Not a GLB: {path}")
    version, total_len = struct.unpack_from("<II", data, 4)
    offset, chunks = 12, []
    while offset < total_len:
        chunk_len, chunk_type = struct.unpack_from("<II", data, offset)
        chunks.append((chunk_type, data[offset + 8: offset + 8 + chunk_len]))
        offset += 8 + chunk_len
    return version, chunks


def write_glb(path: Path, version: int, chunks):
    parts = [MAGIC, struct.pack("<II", version, 0)]
    for ct, cd in chunks:
        pad = (4 - len(cd) % 4) % 4
        cd += (b" " if ct == CHUNK_JSON else b"\x00") * pad
        parts += [struct.pack("<II", len(cd), ct), cd]
    body = b"".join(parts)
    body = body[:8] + struct.pack("<I", len(body)) + body[12:]
    path.write_bytes(body)


def convert_material(mat: dict) -> dict:
    ext = mat.get("extensions", {}).get("KHR_materials_pbrSpecularGlossiness", {})
    if not ext:
        return mat
    pbr = {}
    if "diffuseFactor" in ext:
        df = ext["diffuseFactor"]
        pbr["baseColorFactor"] = [min(df[0] * 1.3, 1.0), min(df[1] * 1.3, 1.0),
                                   min(df[2] * 1.3, 1.0), df[3]]
    else:
        pbr["baseColorFactor"] = [1.0, 1.0, 1.0, 1.0]
    if "diffuseTexture" in ext:
        pbr["baseColorTexture"] = ext["diffuseTexture"]
    pbr["roughnessFactor"] = 0.6
    pbr["metallicFactor"] = 0.0
    converted = {k: v for k, v in mat.items() if k != "extensions"}
    converted["pbrMetallicRoughness"] = pbr
    remaining = {k: v for k, v in mat.get("extensions", {}).items()
                 if k != "KHR_materials_pbrSpecularGlossiness"}
    if remaining:
        converted["extensions"] = remaining
    return converted


def fix_glb(src: Path, dst: Path):
    version, chunks = read_glb(src)
    new_chunks, count = [], 0
    for ct, cd in chunks:
        if ct != CHUNK_JSON:
            new_chunks.append((ct, cd))
            continue
        gltf = json.loads(cd.decode("utf-8").rstrip("\x00"))
        for i, mat in enumerate(gltf.get("materials", [])):
            new_mat = convert_material(mat)
            if new_mat is not mat:
                count += 1
                print(f"  Mat[{i}] '{mat.get('name', '')}': converted")
            gltf["materials"][i] = new_mat
        for key in ("extensionsUsed", "extensionsRequired"):
            if key in gltf:
                gltf[key] = [e for e in gltf[key]
                             if e != "KHR_materials_pbrSpecularGlossiness"]
                if not gltf[key]:
                    del gltf[key]
        new_chunks.append((CHUNK_JSON, json.dumps(gltf, separators=(",", ":")).encode("utf-8")))
    if count == 0:
        print("No KHR_materials_pbrSpecularGlossiness found — copy as-is")
        import shutil; shutil.copy2(src, dst)
    else:
        write_glb(dst, version, new_chunks)
        print(f"Wrote {dst}  ({dst.stat().st_size:,} bytes)")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python tools/fix_glb_specular.py <input.glb> <output.glb>")
        sys.exit(1)
    fix_glb(Path(sys.argv[1]), Path(sys.argv[2]))
