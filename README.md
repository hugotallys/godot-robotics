# godot-robotics
Godot for robotics simulation on complex environments

## Prerequisites

This project uses MuJoCo for robotics simulation. Download and install from the official releases page: [MuJoCo Releases](https://github.com/google-deepmind/mujoco/releases).

## Project Setup

### 1. Clone the repository
```bash
git clone https://github.com/hugotallys/godot-robotics.git
cd godot-robotics
```

### 2. Create and activate a virtual environment
```bash
# Create a virtual environment
python -m venv .venv

# Activate the virtual environment
# On macOS/Linux:
source .venv/bin/activate

# On Windows:
.venv\Scripts\activate
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Run the simulation
```bash
mjpython simulate_arm.py
```

---

## BVH Viewer

[BVHView](https://theorangeduck.com/page/bvhview) is a `.bvh` animation file viewer built with [raylib](https://www.raylib.com/) and included as a submodule under `BVHView/`.

### Setup and run (macOS)

A setup script handles all dependencies, builds the viewer, and launches it in one step:

```bash
./setup_bvhview.sh [path/to/file.bvh]
```

The script will:
1. Check for Xcode Command Line Tools (`make` and `clang`) — if missing, it will prompt you to install them and exit. Re-run after installation.
2. Clone [raylib](https://github.com/raysan5/raylib) and [raygui](https://github.com/raysan5/raygui) into `~/raylib/` (skipped if already present).
3. Build raylib for the desktop platform (skipped if already built).
4. Initialise the `BVHView` submodule if needed, then build `BVHView/bvhview`.
5. Launch the viewer, forwarding any arguments (e.g. a `.bvh` file path) directly to the binary.

Any argument passed to the script is forwarded to the `bvhview` binary, so you can open a file directly:

```bash
./setup_bvhview.sh motion_capture/walk.bvh
```

### Running the pre-built binary

Once the binary is built, you can run it directly without the setup script:

```bash
./BVHView/bvhview [path/to/file.bvh]
```