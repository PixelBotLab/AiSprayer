#!/bin/bash
set -e

CLEAN_BUILD=false
while getopts "c" opt; do
  case ${opt} in
    c )
      CLEAN_BUILD=true
      ;;
    \? )
      echo "Usage: $0 [-c]"
      exit 1
      ;;
  esac
done

# Build directory
DEPS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/deps"

# Dependencies to restore
# Format: "name|version|url"
DEPS=(
    "ros_industrial_cmake_boilerplate|0.7.4|https://github.com/ros-industrial/ros_industrial_cmake_boilerplate.git"
    "boost_plugin_loader|0.4.3|https://github.com/tesseract-robotics/boost_plugin_loader.git"
    "cereal|1.3.2|https://github.com/USCiLab/cereal.git"
    "descartes_light|0.4.10|https://github.com/swri-robotics/descartes_light.git"
    "noether|0.16.0|https://github.com/ros-industrial/noether.git"
    "noether_ros2|0.2.0|https://github.com/ros-industrial/noether_ros2.git"
    "opw_kinematics|0.5.5|https://github.com/Jmeyer1292/opw_kinematics.git"
    "ruckig|0.9.2|https://github.com/pantor/ruckig.git"
    "tesseract|0.35.0|https://github.com/tesseract-robotics/tesseract.git"
    "tesseract_ros2|0.35.0|https://github.com/tesseract-robotics/tesseract_ros2.git"
    "tesseract_qt|0.35.0|https://github.com/tesseract-robotics/tesseract_qt.git"
    "tesseract_planning|0.35.0|https://github.com/tesseract-robotics/tesseract_planning.git"
    "trajopt|0.35.0|https://github.com/tesseract-robotics/trajopt.git"
    "qdldl|master|https://github.com/osqp/qdldl.git"
)

mkdir -p "$DEPS_DIR/src"

for dep in "${DEPS[@]}"; do
    IFS="|" read -r name version url <<< "$dep"
    
    target_dep_dir="$DEPS_DIR/src/$name"
    if [ -d "$target_dep_dir" ]; then
        echo "====================================="
        echo "Target directory $target_dep_dir already exists. Skipping download."
        echo "====================================="
        continue
    fi

    echo "====================================="
    echo "Downloading $name (version $version)..."
    echo "====================================="

    # Try different tags/branches
    success=false
    for tag in "$version" "v$version" "main" "master"; do
        echo "Trying to clone tag/branch: $tag"
        if git clone --depth 1 --branch "$tag" "$url" "$target_dep_dir" &> /dev/null; then
            success=true
            break
        else
            rm -rf "$target_dep_dir"
        fi
    done

    if [ "$success" = false ]; then
        # Fallback to cloning default branch
        echo "Cloning default branch without specifying tag..."
        if ! git clone --depth 1 "$url" "$target_dep_dir" &> /dev/null; then
            echo "Failed to clone $name"
            rm -rf "$target_dep_dir"
            continue
        fi
    fi
done

echo "Done downloading all dependencies!"

echo "======================================"
echo " Building all dependencies in deps/src..."
echo "======================================"

cd "$DEPS_DIR"

if [ "$CLEAN_BUILD" = true ]; then
    echo "======================================"
    echo " Cleaning previous build artifacts..."
    echo "======================================"
    rm -rf build install log
fi

colcon build \
    --merge-install \
    --base-paths src \
    --install-base install \
    --packages-ignore tesseract_ros_examples \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DTESSERACT_BUILD_TASK_COMPOSER_TASKFLOW=OFF \
    -DTESSERACT_ENABLE_TESTING=OFF \
    -DTESSERACT_ENABLE_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_TESTS=OFF \
    -DJUST_INSTALL_CEREAL=ON \
    -DFETCHCONTENT_SOURCE_DIR_QDLDL=$PWD/src/qdldl \
    -DBUILD_STUDIO=OFF

