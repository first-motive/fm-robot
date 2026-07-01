#!/usr/bin/env python3
# Copyright 2026 First Motive
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Convert OpenArm visual COLLADA meshes to STL.

Why this exists: OpenArm's upstream visual meshes are COLLADA (.dae). Their
declared ``up_axis`` values are inconsistent (arm + pinch-gripper meshes are
``Y_UP``, the body and parallel-link meshes are ``Z_UP``), but the vertices
themselves are all authored in one shared Z-up world frame — the ``up_axis`` tags
are stale metadata, not a real per-file rotation. Foxglove Studio needs plain STL
(it fetches ``package://`` STL through foxglove_bridge; it cannot render the .dae).

The earlier assimp-based converter *honoured* each file's ``up_axis`` on export.
That rotated the ``Z_UP`` body mesh 90 degrees relative to the ``Y_UP`` arm meshes,
so the assembled robot rendered with its stand lying flat while the arms sat
upright. The correct conversion is the opposite: ignore ``up_axis`` and export the
raw authored geometry, which keeps every mesh in the shared Z-up frame the URDF
``<visual>`` origins already expect.

trimesh does exactly that — ``trimesh.load(..., force="mesh")`` applies the COLLADA
node transforms and concatenates geometry without re-rotating for ``up_axis``, so a
plain load-and-export per mesh yields the upright assembly. A canary assertion on
the body mesh guards against a future trimesh that starts applying ``up_axis``.

Output mirrors the source tree under ``--out`` so launch path rewrites map
``package://openarm_description/<rel>.dae`` to
``package://fm_description/openarm_meshes/<rel>.stl`` by a plain substitution.
"""

import argparse
import pathlib
import sys

import trimesh

# The body mesh declares Z_UP and stands ~0.77 m tall along its own Z (the arms
# mount 0.698 m up it). If a converted body's longest axis is not Z, trimesh has
# started applying the COLLADA up_axis — the assimp bug this converter replaced —
# and the assembled robot will render mis-rotated. Fail the build loudly instead.
_CANARY_STEM = "body_link0"

# Reconvert when the source .dae OR this script is newer than the output, so an
# edit to the conversion rule regenerates existing meshes even when the .dae is
# unchanged. CMake declares the same script as a DEPENDS; this mirrors it so a
# direct run stays consistent with the build.
_SCRIPT_MTIME = pathlib.Path(__file__).stat().st_mtime


def _convert_one(dae, out_stl):
    out_stl.parent.mkdir(parents=True, exist_ok=True)
    # Skip if the output is newer than both the source and this script (fast rebuilds).
    if out_stl.exists():
        out_mtime = out_stl.stat().st_mtime
        if out_mtime >= dae.stat().st_mtime and out_mtime >= _SCRIPT_MTIME:
            return False
    mesh = trimesh.load(str(dae), force="mesh")
    if mesh.is_empty or len(mesh.vertices) == 0:
        raise RuntimeError(f"trimesh loaded no geometry from {dae}")
    if dae.stem == _CANARY_STEM:
        long_axis = int(mesh.extents.argmax())
        if long_axis != 2:
            raise RuntimeError(
                f"{dae.name}: converted long axis is {'XYZ'[long_axis]}, expected Z. "
                "trimesh applied the COLLADA up_axis, so the assembled robot will be "
                "mis-rotated. Pin trimesh or add explicit axis normalisation."
            )
    mesh.export(str(out_stl))
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src", required=True, help="openarm_description source root")
    parser.add_argument("--out", required=True, help="output root for the STL meshes")
    args = parser.parse_args()

    src, out = pathlib.Path(args.src), pathlib.Path(args.out)
    # Match a visual/ dir at any depth (incl. directly under src), mirroring the
    # CMakeLists glob that filters *.dae on the "/visual/" substring. The prior
    # "*/visual/*.dae" required a dir before visual/, so a root-level visual/ would
    # be declared by CMake but skipped here, breaking the build.
    daes = sorted(src.rglob("visual/*.dae"))
    if not daes:
        print(f"convert_openarm_visual_meshes: no visual DAE under {src}", file=sys.stderr)
        return 1

    converted = 0
    for dae in daes:
        rel = dae.relative_to(src).with_suffix(".stl")
        if _convert_one(dae, out / rel):
            converted += 1
            print(f"  stl: {rel}")
    print(
        f"convert_openarm_visual_meshes: {converted} converted, "
        f"{len(daes) - converted} up to date ({len(daes)} total)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
