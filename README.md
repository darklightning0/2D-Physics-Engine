# PhysicsEngine

Custom 2D rigid‑body sandbox built with C++, SFML, and a simple SAT-based solver.  You can drop circles, rectangles, or arbitrary convex polygons, tweak material/physics properties, add distance or spring joints, and inspect every object through an in-app editor UI.

## Features

- **Rigid bodies**: circles, axis-aligned rectangles, or user-defined convex polygons (spawned as regular n‑gons with adjustable radius/sides).
- **Physics**: gravity, collision detection via SAT, dynamic/static friction, rolling resistance, and configurable restitution per material.
- **Joints**: distance and spring joints with draggable anchors, frequency/damping controls, and selection cycling.
- **Inspector UI**: left panel for simulation controls, right panel for object/joint properties (mass, restitution, material, geometry, colors, static flag, etc.).
- **Scene tools**: spawn/delete, save/load scenes, clear world, debug overlays (contact points, hitboxes, velocity vectors), dynamic timescale, and keyboard nudges for velocity/angular velocity.
- **Sample scenes**: `scene.txt`, `fluid1.txt`, `fluid2.txt` (contains a container plus 500 low-mass fluid particles).

## Quick Start

### 1. Install Dependencies

| Requirement | macOS (Homebrew) | Ubuntu/Debian | Windows |
| --- | --- | --- | --- |
| Compiler | `xcode-select --install` | `sudo apt install build-essential` | Visual Studio 2019+ with C++ |
| CMake ≥ 3.16 | `brew install cmake` | `sudo apt install cmake` | Included with Visual Studio installer or `choco install cmake` |
| SFML 2.6 | `brew install sfml` | `sudo apt install libsfml-dev` | Download from [sfml-dev.org](https://www.sfml-dev.org/download.php) and unpack; add `SFML_DIR` to CMake cache or system environment |

### 2. Clone

```bash
git clone https://github.com/<your-user>/PhysicsEngine.git
cd PhysicsEngine
```

### 3. Configure & Build

```bash
mkdir -p build
cmake -S . -B build -DSFML_DIR=/path/to/SFML/lib/cmake/SFML  # omit if SFML is in a standard location
cmake --build build
```

### 4. Run

```bash
cmake .
make
./PhysicsEngine
```

Assets (fonts) and sample scenes are bundled, so you can run immediately after building.  Use `F9` inside the app to load `scene.txt`, `fluid1.txt`, or `fluid2.txt` for ready-made demos.

## Controls

### Simulation

| Key | Action |
| --- | --- |
| `Space` | Pause / resume |
| `N` | Step once while paused |
| `Up` / `Down` | Increase / decrease time scale |
| `F3` | Toggle debug overlays |
| `Tab` | Toggle UI panels |
| `C` | Clear scene |
| `F5` / `F9` | Save / load scene (filename prompted) |
| `Delete` / `Backspace` | Remove selected object |

### Spawning & Editing

| Key | Action |
| --- | --- |
| `Mouse Left` | Select body or joint handle |
| `Z` / `X` | Spawn dynamic circle / rectangle at mouse |
| `P` | Spawn polygon (uses current sides & radius) |
| `G` / `H` | Decrease / increase polygon sides |
| `I` | Type exact polygon sides (3‑48) |
| `Y` / `U` | Decrease / increase polygon radius |
| `O` | Type exact polygon radius (meters) |
| Number keys `1-0` | Edit selected body properties (mass, restitution, material, width/height/radius, static toggle, RGB channels) or joint parameters |

For selected dynamic bodies:

- `W/A/S/D` apply linear velocity nudges.
- `Q/E` adjust angular velocity.
- `R` zero out linear & angular velocity.

### Joints

- `J` start creating a distance joint from the selected body (click another body to finish).
- `K` start creating a spring joint.
- `F6` / `F7` cycle through existing distance / spring joints.
- Joint anchors can be clicked and dragged directly in the scene when selected.

## Scene Files

Scenes are plain text with sections `OBJECTS`, `DISTANCE`, and `SPRINGS`.  You can load any file (e.g., `fluid2.txt`) via `F9`, or save the current scene via `F5`.  Use these as templates for scripted setups or “fluid” particle packs.

## Project Structure

```
Codes/
  main.cpp            // entry point, UI & controls
  World.cpp/.hpp      // world management, serialization, joint creation
  physics.cpp         // dynamics, gravity, collision resolution
  utility.cpp/.hpp    // SAT helpers, math utilities
  Object.hpp          // rigid body representation
  Joint.hpp           // distance & spring joints
  ...                 // math, config, and material helpers
Assets/               // fonts
scene.txt, fluid*.txt // sample scenes
```

## Contributing / Next Steps

Ideas for future improvements:

- Undo/redo history hook-ups for every edit action.
- Polygon editing after spawn (vertex drag handles, scaling UI).
- Additional joint types (hinges, sliders), or constraint limits.
- AI-assisted scene assembly / scripting hooks.

Feel free to fork and experiment!
