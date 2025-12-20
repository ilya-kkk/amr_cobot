#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def make_pattern(tag_id: int, n: int = 6) -> list[list[int]]:
    # Deterministic pseudo-random pattern (not a real AprilTag code, just visual)
    x = (tag_id + 1) * 0x9E3779B1
    bits = []
    for i in range(n * n):
        x = (1103515245 * x + 12345) & 0xFFFFFFFF
        bits.append((x >> 31) & 1)
    return [bits[r * n : (r + 1) * n] for r in range(n)]


def write_ppm(path: Path, tag_id: int, size: int = 96) -> None:
    # P3 (ASCII) PPM, black/white with border and a 6x6 inner pattern.
    w = h = size
    border = max(6, size // 12)
    inner_border = max(4, size // 16)

    grid_n = 6
    grid = make_pattern(tag_id, grid_n)
    grid_size = size - 2 * (border + inner_border)
    cell = max(1, grid_size // grid_n)
    grid_px = cell * grid_n

    x0 = border + inner_border + (grid_size - grid_px) // 2
    y0 = border + inner_border + (grid_size - grid_px) // 2

    def pix(x: int, y: int) -> tuple[int, int, int]:
        # outer border: black
        if x < border or y < border or x >= w - border or y >= h - border:
            return (0, 0, 0)
        # inner border: white
        if x < border + inner_border or y < border + inner_border or x >= w - (border + inner_border) or y >= h - (border + inner_border):
            return (255, 255, 255)

        # inner grid area: pattern
        if x0 <= x < x0 + grid_px and y0 <= y < y0 + grid_px:
            cx = (x - x0) // cell
            cy = (y - y0) // cell
            v = grid[int(cy)][int(cx)]
            return (0, 0, 0) if v else (255, 255, 255)

        return (255, 255, 255)

    lines = ["P3", f"# fake apriltag texture id={tag_id}", f"{w} {h}", "255"]
    for y in range(h):
        row = []
        for x in range(w):
            r, g, b = pix(x, y)
            row.append(f"{r} {g} {b}")
        lines.append(" ".join(row))

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_textured_plane_dae(path: Path, image_filename: str, mesh_name: str, size_m: float = 0.2) -> None:
    # Simple 2-triangle plane in XY with UVs (0..1).
    s = size_m
    hs = s / 2.0
    dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>make_tag_assets.py</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>

  <library_images>
    <image id="{mesh_name}-img" name="{mesh_name}-img">
      <init_from>{image_filename}</init_from>
    </image>
  </library_images>

  <library_effects>
    <effect id="{mesh_name}-fx">
      <profile_COMMON>
        <newparam sid="surface">
          <surface type="2D">
            <init_from>{mesh_name}-img</init_from>
          </surface>
        </newparam>
        <newparam sid="sampler">
          <sampler2D>
            <source>surface</source>
          </sampler2D>
        </newparam>
        <technique sid="common">
          <lambert>
            <emission>
              <color>1 1 1 1</color>
            </emission>
            <diffuse>
              <texture texture="sampler" texcoord="TEXCOORD"/>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
  </library_effects>

  <library_materials>
    <material id="{mesh_name}-mat" name="{mesh_name}-mat">
      <instance_effect url="#{mesh_name}-fx"/>
    </material>
  </library_materials>

  <library_geometries>
    <geometry id="{mesh_name}-geom" name="{mesh_name}">
      <mesh>
        <source id="{mesh_name}-pos">
          <float_array id="{mesh_name}-pos-array" count="12">-{hs} -{hs} 0 {hs} -{hs} 0 {hs} {hs} 0 -{hs} {hs} 0</float_array>
          <technique_common>
            <accessor source="#{mesh_name}-pos-array" count="4" stride="3">
              <param name="X" type="float"/>
              <param name="Y" type="float"/>
              <param name="Z" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <source id="{mesh_name}-uv">
          <float_array id="{mesh_name}-uv-array" count="8">0 0 1 0 1 1 0 1</float_array>
          <technique_common>
            <accessor source="#{mesh_name}-uv-array" count="4" stride="2">
              <param name="S" type="float"/>
              <param name="T" type="float"/>
            </accessor>
          </technique_common>
        </source>

        <vertices id="{mesh_name}-vtx">
          <input semantic="POSITION" source="#{mesh_name}-pos"/>
        </vertices>

        <!-- Front face -->
        <triangles count="2" material="mat0">
          <input semantic="VERTEX" source="#{mesh_name}-vtx" offset="0"/>
          <input semantic="TEXCOORD" source="#{mesh_name}-uv" offset="1" set="0"/>
          <p>0 0 1 1 2 2 0 0 2 2 3 3</p>
        </triangles>

        <!-- Back face (duplicate with reversed winding so it's visible from behind) -->
        <triangles count="2" material="mat0">
          <input semantic="VERTEX" source="#{mesh_name}-vtx" offset="0"/>
          <input semantic="TEXCOORD" source="#{mesh_name}-uv" offset="1" set="0"/>
          <p>0 0 2 2 1 1 0 0 3 3 2 2</p>
        </triangles>
      </mesh>
    </geometry>
  </library_geometries>

  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="{mesh_name}-node" name="{mesh_name}">
        <instance_geometry url="#{mesh_name}-geom">
          <bind_material>
            <technique_common>
              <instance_material symbol="mat0" target="#{mesh_name}-mat">
                <bind_vertex_input semantic="TEXCOORD" input_semantic="TEXCOORD" input_set="0"/>
              </instance_material>
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
    path.write_text(dae, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--meshes-dir", type=Path, required=True)
    args = ap.parse_args()

    meshes = args.meshes_dir
    meshes.mkdir(parents=True, exist_ok=True)

    # Textures (ASCII PPM)
    write_ppm(meshes / "apriltag0.ppm", tag_id=0)
    write_ppm(meshes / "apriltag1.ppm", tag_id=1)

    # Textured visual planes
    write_textured_plane_dae(meshes / "april0_tag.dae", "apriltag0.ppm", "april0_tag", size_m=0.2)
    write_textured_plane_dae(meshes / "april1_tag.dae", "apriltag1.ppm", "april1_tag", size_m=0.2)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


