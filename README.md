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