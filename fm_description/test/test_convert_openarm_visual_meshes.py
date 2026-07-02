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

trimesh is an optional dependency (dropped from rosdep as unresolvable), so its tests
skip when it is absent. The skip is scoped to the ``conv`` fixture, not the whole
module, so collection still yields one always-run test (``test_converter_script_exists``)
— otherwise a trimesh-less CI run collects zero tests and pytest exits 5, which colcon
reports as a failure.
"""

import importlib.util
import os
import pathlib

import pytest

# Path-only (no trimesh) so the module always imports and collects.
_SCRIPT = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "convert_openarm_visual_meshes.py"
)


@pytest.fixture
def conv():
    """Load the converter module, skipping the test when trimesh is unavailable."""
    # The converter imports trimesh at module scope, so skip here rather than at
    # import — that keeps this test file collectable without trimesh installed.
    pytest.importorskip("trimesh")
    spec = importlib.util.spec_from_file_location(
        "convert_openarm_visual_meshes", _SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _box(path, extents):
    """Write a box mesh with the given (x, y, z) extents to ``path``."""
    import trimesh

    trimesh.creation.box(extents=extents).export(path)
    return path


def test_converter_script_exists():
    """The converter script ships even when trimesh is not installed."""
    # Always-run (no trimesh): keeps the module from collecting zero tests — and
    # pytest exiting 5 — when trimesh is absent and every gated test below skips.
    assert _SCRIPT.is_file()


def test_export_produces_nonempty_stl(conv, tmp_path):
    import trimesh

    src = _box(tmp_path / "arm_link.obj", (0.10, 0.06, 0.12))
    out = tmp_path / "arm_link.stl"
    assert conv._convert_one(src, out) is True
    assert out.exists() and out.stat().st_size > 0
    # Round-trips as a real mesh with the same longest axis.
    assert int(trimesh.load(str(out), force="mesh").extents.argmax()) == 2


def test_canary_passes_when_body_long_axis_is_z(conv, tmp_path):
    # Body stand: tall along Z, as authored (~0.77 m in the real mesh).
    src = _box(tmp_path / "body_link0.obj", (0.25, 0.19, 0.77))
    assert conv._convert_one(src, tmp_path / "body_link0.stl") is True


def test_canary_fails_when_body_long_axis_is_not_z(conv, tmp_path):
    # Long axis on Y = the assimp mis-rotation the canary must catch.
    src = _box(tmp_path / "body_link0.obj", (0.25, 0.77, 0.19))
    with pytest.raises(RuntimeError, match="up_axis"):
        conv._convert_one(src, tmp_path / "body_link0.stl")


def test_canary_only_applies_to_body_mesh(conv, tmp_path):
    # A non-body mesh with a non-Z long axis must NOT trip the canary.
    src = _box(tmp_path / "link1.obj", (0.10, 0.77, 0.10))
    assert conv._convert_one(src, tmp_path / "link1.stl") is True


def test_up_to_date_output_is_skipped(conv, tmp_path):
    src = _box(tmp_path / "link.obj", (0.10, 0.10, 0.20))
    out = tmp_path / "link.stl"
    # Age the source so the fresh output is unambiguously newer than src + script.
    old = _SCRIPT.stat().st_mtime - 100
    os.utime(src, (old, old))
    assert conv._convert_one(src, out) is True
    assert conv._convert_one(src, out) is False


def test_stale_output_is_regenerated(conv, tmp_path):
    src = _box(tmp_path / "link.obj", (0.10, 0.10, 0.20))
    out = tmp_path / "link.stl"
    assert conv._convert_one(src, out) is True
    # Source newer than the existing output → reconvert. Anchor to the output's own
    # mtime, not the script's: on a slow build the output is written well over 100 s
    # after checkout, so a script-relative offset can land before it and wrongly skip.
    future = out.stat().st_mtime + 100
    os.utime(src, (future, future))
    assert conv._convert_one(src, out) is True
