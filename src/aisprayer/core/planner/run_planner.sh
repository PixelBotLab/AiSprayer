#!/bin/bash

# Exit on any error
set -e

# Automatically resolve directories regardless of where the script is called from
PLANNER_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "$PLANNER_DIR/../../../.." &> /dev/null && pwd )"
BUILD_DIR="$PLANNER_DIR/build"
DEPS_LIB_DIR="$PLANNER_DIR/deps/install/lib"

echo "======================================"
echo " Building Process and Motion Planners..."
echo "======================================"

# Create build directory if it doesn't exist
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run cmake and make
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

echo ""
echo "======================================"
echo " Running Refactored Planner Pipeline..."
echo "======================================"

# Set library path for runtime
export LD_LIBRARY_PATH="$DEPS_LIB_DIR:$LD_LIBRARY_PATH"

if [ "$#" -eq 0 ]; then
    TEST_DIR="$PROJECT_ROOT/data/runs/0/output"
    if [ -f "$TEST_DIR/1.obj" ] && [ -f "$TEST_DIR/2.obj" ]; then
        MESH_PATH="$TEST_DIR/1.obj,$TEST_DIR/2.obj"
    else
        MESH_PATH="$PROJECT_ROOT/data/1.obj"
    fi

    "$BUILD_DIR/process_planner" \
        --mesh "$MESH_PATH" \
        --outdir "$TEST_DIR" \
        --distance 0.20 \
        --row-spacing 0.04 \
        --point-spacing 0.01 \
        --calibration "$PROJECT_ROOT/configs/calib/calibration_result.yaml"
    "$BUILD_DIR/motion_planner" \
        --input "$TEST_DIR/tcp_targets.json" \
        --urdf "$PROJECT_ROOT/configs/m530_r6.urdf.xml" \
        --srdf "$PROJECT_ROOT/configs/m530_r6.srdf.xml" \
        --outdir "$TEST_DIR" \
        --group manipulator \
        --tcp spray_nozzle_link \
        --threads 6 
    exit 0
fi


# --ik-only

if [ "$1" = "--process-only" ]; then
    shift
    exec "$BUILD_DIR/process_planner" "$@"
fi

if [ "$1" = "--motion-only" ]; then
    shift
    exec "$BUILD_DIR/motion_planner" "$@"
fi

# Preserve the previous unified CLI for supported full-pipeline arguments while
# translating the old underscore spellings to the two new programs.
PROCESS_ARGS=()
MOTION_ARGS=()
OUTDIR=""
PROCESS_ONLY=false
RASTER_ORDER_SPECIFIED=false

require_value() {
    if [ "$#" -lt 2 ]; then
        echo "Missing value for $1" >&2
        exit 2
    fi
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --mesh|-m|--distance|--direction|--image-horizontal|--calibration|--seam-dedup-distance|--straight-lines)
            if [ "$1" = "--straight-lines" ]; then
                PROCESS_ARGS+=("$1")
                shift
            else
                require_value "$@"
                PROCESS_ARGS+=("$1" "$2")
                if [ "$1" = "--image-horizontal" ] || [ "$1" = "--calibration" ]; then
                    RASTER_ORDER_SPECIFIED=true
                fi
                shift 2
            fi
            ;;
        --row_spacing)
            require_value "$@"
            PROCESS_ARGS+=("--row-spacing" "$2")
            shift 2
            ;;
        --row-spacing|--point-spacing|--point_spacing|-r|-p)
            require_value "$@"
            if [ "$1" = "--point_spacing" ]; then
                PROCESS_ARGS+=("--point-spacing" "$2")
            else
                PROCESS_ARGS+=("$1" "$2")
            fi
            shift 2
            ;;
        --outdir|-o)
            require_value "$@"
            OUTDIR="$2"
            PROCESS_ARGS+=("--outdir" "$2")
            MOTION_ARGS+=("--outdir" "$2")
            shift 2
            ;;
        --urdf|--srdf|--group|--tcp|--base-link|--position-tolerance|--orientation-tolerance|--angle-unit|--threads)
            require_value "$@"
            MOTION_ARGS+=("$1" "$2")
            shift 2
            ;;
        --angle_unit)
            require_value "$@"
            MOTION_ARGS+=("--angle-unit" "$2")
            shift 2
            ;;
        --noether-only)
            PROCESS_ONLY=true
            shift
            ;;
        --kdl-only)
            echo "--kdl-only is mapped to motion_planner --ik-only with KDL." >&2
            MOTION_ARGS+=("--ik-only")
            shift
            ;;
        *)
            echo "Unsupported legacy argument: $1" >&2
            exit 2
            ;;
    esac
done

if [ -z "$OUTDIR" ]; then
    echo "Legacy full-pipeline mode requires --outdir." >&2
    exit 2
fi

if [ "$RASTER_ORDER_SPECIFIED" = false ]; then
    PROCESS_ARGS+=("--calibration" "$PROJECT_ROOT/configs/calib/calibration_result.yaml")
fi

"$BUILD_DIR/process_planner" "${PROCESS_ARGS[@]}"
if [ "$PROCESS_ONLY" = true ]; then
    exit 0
fi

exec "$BUILD_DIR/motion_planner" --input "$OUTDIR/tcp_targets.json" "${MOTION_ARGS[@]}"
