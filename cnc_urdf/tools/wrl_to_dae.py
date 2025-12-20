#!/usr/bin/env python3
"""
Very small WRL (VRML1 / VRML2-ish) -> COLLADA (.dae) converter for SolidWorks-like exports.

Goals:
- Convert VRML1 style files like:
  Coordinate3 { point [ ... ] } IndexedFaceSet { coordIndex [ ... ] }
  repeated many times (e.g., one Separator per part)
- Combine *all* chunks into a single DAE, so big assemblies render fully.

Notes / limitations:
- We export positions only (no normals/material/uv).
- Faces are triangulated with a simple fan.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


_FLOAT_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")
_INT_RE = re.compile(r"-?\d+")


def _parse_first_diffuse_color(s: str) -> Tuple[float, float, float]:
    """
    Try to find a VRML1 Material diffuseColor and return the first RGB triple.
    Falls back to a neutral gray.
    """
    try:
        block = _extract_bracket_block(s, "diffuseColor")
        nums = [float(x) for x in _FLOAT_RE.findall(block)]
        if len(nums) >= 3:
            r, g, b = nums[0], nums[1], nums[2]
            # clamp
            r = max(0.0, min(1.0, r))
            g = max(0.0, min(1.0, g))
            b = max(0.0, min(1.0, b))
            return (r, g, b)
    except Exception:
        pass
    return (0.35, 0.35, 0.35)


def _extract_bracket_block(s: str, token: str, start_at: int = 0) -> str:
    """
    Find 'token', then extract the content inside the first [...] that follows it,
    handling nested brackets.
    Returns the inside-content without the surrounding brackets.
    """
    i = s.find(token, start_at)
    if i < 0:
        raise ValueError(f"Token not found: {token!r}")
    j = s.find("[", i)
    if j < 0:
        raise ValueError(f"No '[' after token: {token!r}")
    depth = 0
    k = j
    while k < len(s):
        ch = s[k]
        if ch == "[":
            depth += 1
        elif ch == "]":
            depth -= 1
            if depth == 0:
                return s[j + 1 : k]
        k += 1
    raise ValueError(f"Unclosed bracket block after token: {token!r}")


def _parse_points(block: str) -> List[Tuple[float, float, float]]:
    nums = [float(x) for x in _FLOAT_RE.findall(block)]
    if len(nums) % 3 != 0:
        raise ValueError(f"Point list is not divisible by 3 floats (got {len(nums)} floats)")
    return [(nums[i], nums[i + 1], nums[i + 2]) for i in range(0, len(nums), 3)]


def _parse_faces(block: str) -> List[List[int]]:
    idxs = [int(x) for x in _INT_RE.findall(block)]
    faces: List[List[int]] = []
    cur: List[int] = []
    for v in idxs:
        if v == -1:
            if cur:
                faces.append(cur)
            cur = []
        else:
            cur.append(v)
    if cur:
        faces.append(cur)
    # drop degenerate
    return [f for f in faces if len(f) >= 3]


def _triangulate(faces: List[List[int]]) -> List[Tuple[int, int, int]]:
    tris: List[Tuple[int, int, int]] = []
    for f in faces:
        a = f[0]
        for i in range(1, len(f) - 1):
            tris.append((a, f[i], f[i + 1]))
    return tris


def _dae(mesh_name: str, points: List[Tuple[float, float, float]], tris: List[Tuple[int, int, int]], diffuse_rgb: Tuple[float, float, float]) -> str:
    geom_id = f"{mesh_name}-geom"
    pos_id = f"{mesh_name}-positions"
    vertices_id = f"{mesh_name}-vertices"
    fx_id = f"{mesh_name}-fx"
    mat_id = f"{mesh_name}-mat"

    # Flatten points (keep shared vertex list to keep files small)
    pos_flat = " ".join(f"{x:.9g} {y:.9g} {z:.9g}" for x, y, z in points)
    p_idx = " ".join(f"{a} {b} {c}" for a, b, c in tris)
    r, g, b = diffuse_rgb

    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor>
      <authoring_tool>wrl_to_dae.py</authoring_tool>
    </contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>

  <library_effects>
    <effect id="{fx_id}">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <diffuse>
              <color>{r:.6g} {g:.6g} {b:.6g} 1</color>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_materials>
    <material id="{mat_id}" name="{mat_id}">
      <instance_effect url="#{fx_id}"/>
    </material>
  </library_materials>

  <library_geometries>
    <geometry id="{geom_id}" name="{mesh_name}">
      <mesh>
        <source id="{pos_id}">
          <float_array id="{pos_id}-array" count="{len(points) * 3}">{pos_flat}</float_array>
          <technique_common>
            <accessor source="#{pos_id}-array" count="{len(points)}" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <vertices id="{vertices_id}">
          <input semantic="POSITION" source="#{pos_id}"/>
        </vertices>

        <triangles count="{len(tris)}" material="mat0">
          <input semantic="VERTEX" source="#{vertices_id}" offset="0"/>
          <p>{p_idx}</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{mesh_name}-node" name="{mesh_name}">
        <instance_geometry url="#{geom_id}">
          <bind_material>
            <technique_common>
              <instance_material symbol="mat0" target="#{mat_id}"/>
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
    </visual_scene>
  </library_visual_scenes>

  <scene>
    <instance_visual_scene url="#Scene"/>
  </scene>
</COLLADA>
"""


def convert_wrl_to_dae(src: Path, dst: Path, mesh_name: str) -> None:
    s = src.read_text(encoding="utf-8", errors="ignore")
    diffuse = _parse_first_diffuse_color(s)
    points_all: List[Tuple[float, float, float]] = []
    tris_all: List[Tuple[int, int, int]] = []

    # VRML1 pattern: many repeated Coordinate3 + IndexedFaceSet blocks.
    # We scan in order; each IndexedFaceSet uses the most recent Coordinate3.
    i = 0
    current_points: List[Tuple[float, float, float]] | None = None

    while True:
        next_coord3 = s.find("Coordinate3", i)
        next_coord = s.find("Coordinate", i)  # VRML2 style (rare in this repo)
        next_ifs = s.find("IndexedFaceSet", i)

        candidates = [(next_coord3, "Coordinate3"), (next_coord, "Coordinate"), (next_ifs, "IndexedFaceSet")]
        candidates = [(pos, kind) for pos, kind in candidates if pos != -1]
        if not candidates:
            break

        pos, kind = min(candidates, key=lambda x: x[0])
        if kind in ("Coordinate3", "Coordinate"):
            try:
                points_block = _extract_bracket_block(s, "point", start_at=pos)
                current_points = _parse_points(points_block)
            except Exception:
                current_points = None
            i = pos + 1
            continue

        # IndexedFaceSet
        if current_points:
            try:
                faces_block = _extract_bracket_block(s, "coordIndex", start_at=pos)
                faces = _parse_faces(faces_block)
                tris = _triangulate(faces)
                if tris:
                    base = len(points_all)
                    points_all.extend(current_points)
                    tris_all.extend((a + base, b + base, c + base) for a, b, c in tris)
            except Exception:
                pass
        i = pos + 1

    # Fallback: if scanning found nothing, try the old single-mesh approach.
    if not points_all or not tris_all:
        points_block = _extract_bracket_block(s, "point")
        faces_block = _extract_bracket_block(s, "coordIndex")
        points_all = _parse_points(points_block)
        faces = _parse_faces(faces_block)
        tris_all = _triangulate(faces)

    if not points_all or not tris_all:
        raise ValueError(f"No geometry parsed from {src}")

    dst.write_text(_dae(mesh_name, points_all, tris_all, diffuse), encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--name", default=None, help="mesh name inside DAE (defaults to dst stem)")
    args = ap.parse_args()

    mesh_name = args.name or args.dst.stem
    convert_wrl_to_dae(args.src, args.dst, mesh_name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


