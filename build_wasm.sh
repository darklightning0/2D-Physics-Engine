#!/bin/bash
set -e

# 1. Create target UI folder if not present
mkdir -p PhysicsEngineUI

# 2. Compile the C++ core and bindings into Wasm using system em++
echo "Compiling C++ to WebAssembly..."
em++ -O3 -std=c++17 --bind \
    -IPhysicsEngineCore/Codes \
    PhysicsEngineCore/Codes/Bindings.cpp \
    PhysicsEngineCore/Codes/World.cpp \
    PhysicsEngineCore/Codes/physics.cpp \
    PhysicsEngineCore/Codes/utility.cpp \
    PhysicsEngineCore/Codes/SpatialHashGrid.cpp \
    -o PhysicsEngineUI/physics.js \
    -s WASM=1 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -s NO_EXIT_RUNTIME=1

echo "WebAssembly build completed! Output files are in PhysicsEngineUI/"
