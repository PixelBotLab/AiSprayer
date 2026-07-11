#!/bin/bash

# Exit on any error
set -e

# Automatically resolve directories regardless of where the script is called from
PLANNER_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "$PLANNER_DIR/../../../.." &> /dev/null && pwd )"
BUILD_DIR="$PLANNER_DIR/build"
DEPS_LIB_DIR="$PLANNER_DIR/deps/install/lib"

echo "======================================"
echo " Building Trajectory Planner..."
echo "======================================"

# Create build directory if it doesn't exist
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run cmake and make
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

echo ""
echo "======================================"
echo " Running Trajectory Planner..."
echo "======================================"

# Set library path for runtime
export LD_LIBRARY_PATH="$DEPS_LIB_DIR:$LD_LIBRARY_PATH"

# Set default arguments if none are provided
if [ "$#" -eq 0 ]; then
    echo "No arguments provided. Running with default test arguments."
    
    # Check if the run directory exists to provide a valid test path
    TEST_DIR="$PROJECT_ROOT/data/runs/0/output"
    MESH_ARGS=""
    if [ -f "$TEST_DIR/1.obj" ] && [ -f "$TEST_DIR/2.obj" ]; then
        MESH_ARGS="--mesh $TEST_DIR/1.obj,$TEST_DIR/2.obj"
    else
        # Fallback if specific run output doesn't exist
        MESH_ARGS="--mesh $PROJECT_ROOT/data/1.obj"
    fi
    
    "$BUILD_DIR/planner" \
        --urdf "$PROJECT_ROOT/configs/m530_r6.urdf.xml" \
        --srdf "$PROJECT_ROOT/configs/m530_r6.srdf.xml" \
        $MESH_ARGS \
        --outdir "$TEST_DIR" \
        --group manipulator \
        --tcp spray_nozzle_link \
        --distance 0.20 \
        --row_spacing 0.04 \
        --point_spacing 0.01 \
        --kdl-only
else
    # Run with user-provided arguments
    echo "Running with custom arguments: $@"
    "$BUILD_DIR/planner" "$@"
fi
