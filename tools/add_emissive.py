"""
Add emissiveFactor to all materials in a GLB to compensate for dark
metallic/specular textures that Gazebo's ogre renderer can't light correctly.

Usage:  python tools/add_emissive.py <path/to/model.glb> [emissive_value]
Default emissive_value = 0.25
"""
import json, struct, sys
from pathlib import Path

CHUNK_JSON = 0x4E4F534A

def read_glb(path):
    data = path.read_bytes()
    version, total_len = struct.unpack_from("<II", data, 4)
    offset, chunks = 12, []
    while offset < total_len:
        chunk_len, chunk_type = struct.unpack_from("<II", data, offset)
        chunks.append((chunk_type, data[offset+8: offset+8+chunk_len]))
        offset += 8 + chunk_len
    return version, chunks

def write_glb(path, version, chunks):
    parts = [b"glTF", struct.pack("<II", version, 0)]
    for ct, cd in chunks:
        pad = (4 - len(cd) % 4) % 4
        cd += (b" " if ct == CHUNK_JSON else b"\x00") * pad
        parts += [struct.pack("<II", len(cd), ct), cd]
    body = b"".join(parts)
    body = body[:8] + struct.pack("<I", len(body)) + body[12:]
    path.write_bytes(body)

def patch(glb_path: Path, emissive: float):
    version, chunks = read_glb(glb_path)
    new_chunks = []
    for ct, cd in chunks:
        if ct != CHUNK_JSON:
            new_chunks.append((ct, cd))
            continue
        gltf = json.loads(cd.decode("utf-8").rstrip("\x00"))
        for mat in gltf.get("materials", []):
            mat["emissiveFactor"] = [emissive, emissive, emissive]
        new_chunks.append((ct, json.dumps(gltf, separators=(",", ":")).encode("utf-8")))
    write_glb(glb_path, version, new_chunks)
    print(f"Patched {glb_path}  emissiveFactor=[{emissive},{emissive},{emissive}]")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python tools/add_emissive.py <model.glb> [emissive 0-1]")
        sys.exit(1)
    glb = Path(sys.argv[1])
    ev = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    patch(glb, ev)
