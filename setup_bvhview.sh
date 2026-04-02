#!/usr/bin/env bash
set -e

RAYLIB_DIR="$HOME/raylib"

# ── 1. Dependencies ──────────────────────────────────────────────────────────

if ! command -v make &>/dev/null || ! command -v clang &>/dev/null; then
    echo "[setup] Installing Xcode Command Line Tools..."
    xcode-select --install
    echo "[setup] Re-run this script after the installation finishes."
    exit 1
fi

# ── 2. raylib ────────────────────────────────────────────────────────────────

if [ ! -d "$RAYLIB_DIR/raylib" ]; then
    echo "[setup] Cloning raylib..."
    mkdir -p "$RAYLIB_DIR"
    git clone https://github.com/raysan5/raylib.git "$RAYLIB_DIR/raylib"
fi

if [ ! -d "$RAYLIB_DIR/raygui" ]; then
    echo "[setup] Cloning raygui..."
    git clone https://github.com/raysan5/raygui.git "$RAYLIB_DIR/raygui"
fi

RAYLIB_LIB="$RAYLIB_DIR/raylib/src/libraylib.a"
if [ ! -f "$RAYLIB_LIB" ]; then
    echo "[setup] Building raylib..."
    make -C "$RAYLIB_DIR/raylib/src" PLATFORM=PLATFORM_DESKTOP
else
    echo "[setup] raylib already built, skipping."
fi

# ── 3. BVHView ───────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BVHVIEW_DIR="$SCRIPT_DIR/BVHView"

if [ ! -f "$BVHVIEW_DIR/bvhview.c" ]; then
    echo "[setup] Initialising BVHView submodule..."
    git -C "$SCRIPT_DIR" submodule update --init --recursive
fi

echo "[setup] Building BVHView..."
make -C "$BVHVIEW_DIR" PLATFORM_OS=Darwin

# ── 4. Run ───────────────────────────────────────────────────────────────────

echo "[setup] Launching BVHView..."
exec "$BVHVIEW_DIR/bvhview" "$@"
