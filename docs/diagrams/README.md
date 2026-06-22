# Diagrams

Architecture diagrams for the robot layer, authored in [d2](https://d2lang.com).
Each `.d2` file is the source of truth; the matching `.svg` is a generated
artifact referenced by the docs. Edit the `.d2`, then re-render.

## Render

```bash
./render.sh          # renders every *.d2 to *.svg with the brand font
```

Needs `d2` on `PATH`. The font ships in [`fonts/`](fonts/), so rendering is
self-contained — no font install, no personal tooling. The script passes the
font explicitly:

```bash
d2 --layout elk --font-regular fonts/GeistMono-VF.ttf \
   --font-bold fonts/GeistMono-VF.ttf --font-italic fonts/GeistMono-VF.ttf in.d2 out.svg
```

## Font

**Geist Mono** — First Motive's brand monospace ([Vercel](https://github.com/vercel/geist-font),
OFL). Ships as `fonts/GeistMono-VF.ttf`. Mono suits the technical tokens the
diagrams carry (`fm_*`, `*.launch.py`, `ros2_control`).

## Palette

Mirrors firstmotive.ai. Defined once in [`styles.d2`](styles.d2), imported with
`...@styles`.

| Token | Hex | Use |
|-------|-----|-----|
| plum | `#3B3443` | role band, borders, edges |
| lavender | `#B6A5C6` | package band |
| cream | `#E7DDC8` | artifact / node band |
| light text | `#ECE2CF` | text on plum |
| deep | `#342E3B` | text on lavender / cream |

## Block Grammar

Every component is a stacked block built as a `grid-rows` container:

```
┌─────────────────┐  role  — human label (plum)
├─────────────────┤  pkg   — package name (lavender), one colour for all packages
├─────────────────┤  art   — artifact / node (cream)
└─────────────────┘
```

- Blocks without a package (hardware plugins) drop the `pkg` band.
- Node/topic graphs use `node` (plum box) + `topic` (cream pill) instead.
- Layout is ELK (straight orthogonal edges); `direction: right` for fan-in.

## Diagrams

The robot layer is two stacked concerns — description and control — over one
hardware abstraction. Each diagram views one slice, narrowing from model to
hardware.

```
packages               how the four packages connect — ament deps; fm_robot aggregates, fm_control layers on fm_description
robot_state_publisher  where /robot_description comes from — URDF/xacro → xacro → robot_state_publisher
view_robot             robot state publishing — joint_state_publisher → robot_state_publisher → /tf · /robot_description
control_robot          simulation — /robot_description + /cmd_vel → Sim control plugin → /joint_states · /tf
control                ros2_control graph — controller_manager ↔ resource_manager → hardware interfaces
hardware               sim_backend → {mock · mujoco · gazebo · isaac · real} → one ros2_control system interface
```

`hardware` is the architectural crux: everything above the `ros2_control` system
interface is identical across every backend, and the dashed `hardware` block in
`control` expands into it. See [ARCHITECTURE.md](../ARCHITECTURE.md). The backend
plugins themselves live in [`fm-sim`](https://github.com/first-motive/fm-sim);
this diagram shows the interface they bind to.
