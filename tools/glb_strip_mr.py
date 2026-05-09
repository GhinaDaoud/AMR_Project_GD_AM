#!/usr/bin/env python3
"""Strip metallicRoughnessTexture from GLB materials to avoid channel errors.

Usage:
  python tools/glb_strip_mr.py in.glb out.glb
"""
import sys
import struct
import json

JSON_CHUNK = b"JSON"
BIN_CHUNK = b"BIN\x00"


def read_glb(path: str):
    with open(path, "rb") as f:
        header = f.read(12)
        if len(header) < 12:
            raise ValueError("Not a valid GLB (header too short)")
        magic, version, length = struct.unpack("<4sII", header)
        if magic != b"glTF":
            raise ValueError("Not a GLB (bad magic)")
        chunks = []
        while True:
            chunk_head = f.read(8)
            if not chunk_head or len(chunk_head) < 8:
                break
            chunk_len, chunk_type = struct.unpack("<I4s", chunk_head)
            chunk_data = f.read(chunk_len)
            chunks.append((chunk_type, chunk_data))
        return version, length, chunks


def write_glb(out_path: str, version: int, chunks):
    total_len = 12
    for t, d in chunks:
        total_len += 8 + len(d)
    with open(out_path, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", version, total_len))
        for t, d in chunks:
            f.write(struct.pack("<I4s", len(d), t))
            f.write(d)


def strip_mr(in_path: str, out_path: str) -> int:
    version, _length, chunks = read_glb(in_path)
    json_chunk = None
    other_chunks = []
    for t, d in chunks:
        if t == JSON_CHUNK:
            json_chunk = d
        else:
            other_chunks.append((t, d))
    if json_chunk is None:
        raise ValueError("No JSON chunk found")
    data = json.loads(json_chunk.decode("utf-8"))

    mats = data.get("materials", [])
    removed = 0
    for mat in mats:
        pbr = mat.get("pbrMetallicRoughness")
        if not isinstance(pbr, dict):
            continue
        if "metallicRoughnessTexture" in pbr:
            del pbr["metallicRoughnessTexture"]
            removed += 1

    new_json = json.dumps(data, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    pad = (4 - (len(new_json) % 4)) % 4
    if pad:
        new_json += b" " * pad

    out_chunks = [(JSON_CHUNK, new_json)] + other_chunks
    write_glb(out_path, version, out_chunks)
    return removed


def main():
    if len(sys.argv) != 3:
        print("Usage: python tools/glb_strip_mr.py in.glb out.glb")
        return 2
    in_path, out_path = sys.argv[1], sys.argv[2]
    removed = strip_mr(in_path, out_path)
    print(f"Stripped metallicRoughnessTexture from {removed} materials")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
