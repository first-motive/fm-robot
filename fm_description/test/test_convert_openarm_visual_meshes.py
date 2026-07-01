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
Unit-test the OpenArm visual-mesh converter (``_convert_one``).

The converter replaced an assimp CLI that honoured each COLLADA ``up_axis`` and so
rotated the lone ``Z_UP`` body mesh 90 degrees (stand flat, arms upright). trimesh
ignores ``up_axis`` and exports the raw geometry, so the body must stay Z-tall. A
canary assertion enforces that; these tests cover the canary (both directions), the
export path, and the staleness skip — with synthetic boxes so no ``.dae`` fixtures
or pycollada are needed (``_convert_one`` is mesh-format agnostic via trimesh).
"""

import importlib.util
import os
import pathlib

import pytest

trimesh = pytest.importorskip("trimesh")

# Load the converter as a module directly from scripts/ (not on the Python path).
_SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "convert_openarm_visual_meshes.py"
)
_spec = importlib.util.spec_from_file_location("convert_openarm_visual_meshes", _SCRIPT)
conv = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(conv)


def _box(path, extents):
    """Write a box mesh with the given (x, y, z) extents to ``path``."""
    trimesh.creation.box(extents=extents).export(path)
    return path


def test_export_produces_nonempty_stl(tmp_path):
    src = _box(tmp_path / "arm_link.obj", (0.10, 0.06, 0.12))
    out = tmp_path / "arm_link.stl"
    assert conv._convert_one(src, out) is True
    assert out.exists() and out.stat().st_size > 0
    # Round-trips as a real mesh with the same longest axis.
    assert int(trimesh.load(str(out), force="mesh").extents.argmax()) == 2


def test_canary_passes_when_body_long_axis_is_z(tmp_path):
    # Body stand: tall along Z, as authored (~0.77 m in the real mesh).
    src = _box(tmp_path / "body_link0.obj", (0.25, 0.19, 0.77))
    assert conv._convert_one(src, tmp_path / "body_link0.stl") is True


def test_canary_fails_when_body_long_axis_is_not_z(tmp_path):
    # Long axis on Y = the assimp mis-rotation the canary must catch.
    src = _box(tmp_path / "body_link0.obj", (0.25, 0.77, 0.19))
    with pytest.raises(RuntimeError, match="up_axis"):
        conv._convert_one(src, tmp_path / "body_link0.stl")


def test_canary_only_applies_to_body_mesh(tmp_path):
    # A non-body mesh with a non-Z long axis must NOT trip the canary.
    src = _box(tmp_path / "link1.obj", (0.10, 0.77, 0.10))
    assert conv._convert_one(src, tmp_path / "link1.stl") is True


def test_up_to_date_output_is_skipped(tmp_path):
    src = _box(tmp_path / "link.obj", (0.10, 0.10, 0.20))
    out = tmp_path / "link.stl"
    # Age the source so the fresh output is unambiguously newer than src + script.
    old = _SCRIPT.stat().st_mtime - 100
    os.utime(src, (old, old))
    assert conv._convert_one(src, out) is True
    assert conv._convert_one(src, out) is False


def test_stale_output_is_regenerated(tmp_path):
    src = _box(tmp_path / "link.obj", (0.10, 0.10, 0.20))
    out = tmp_path / "link.stl"
    assert conv._convert_one(src, out) is True
    # Source newer than the existing output → reconvert.
    future = _SCRIPT.stat().st_mtime + 100
    os.utime(src, (future, future))
    assert conv._convert_one(src, out) is True
