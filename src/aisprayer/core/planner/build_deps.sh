#!/bin/bash
set -e

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
    "opw_kinematics|0.5.5|https://github.com/Jmeyer1292/opw_kinematics.git"
    "ruckig|0.9.2|https://github.com/pantor/ruckig.git"
    "tesseract|0.35.0|https://github.com/tesseract-robotics/tesseract.git"
    "tesseract_planning|0.35.0|https://github.com/tesseract-robotics/tesseract_planning.git"
    "trajopt|0.35.0|https://github.com/tesseract-robotics/trajopt.git"
)

TEMP_DIR="$DEPS_DIR/temp_restore"
rm -rf "$TEMP_DIR"
mkdir -p "$TEMP_DIR"

for dep in "${DEPS[@]}"; do
    IFS="|" read -r name version url <<< "$dep"
    
    target_dep_dir="$DEPS_DIR/src/$name"
    if [ ! -d "$target_dep_dir" ]; then
        echo "Target directory $target_dep_dir does not exist. Skipping copy."
        continue
    fi

    if [ -f "$target_dep_dir/.downloaded" ]; then
        echo "====================================="
        echo "$name is already downloaded. Skipping restore."
        echo "====================================="
        continue
    fi

    echo "====================================="
    echo "Restoring $name (version $version)..."
    echo "====================================="

    clone_path="$TEMP_DIR/$name"
    rm -rf "$clone_path"

    # Try different tags/branches
    success=false
    for tag in "$version" "v$version" "main" "master"; do
        echo "Trying to clone tag/branch: $tag"
        if git clone --depth 1 --branch "$tag" "$url" "$clone_path" &> /dev/null; then
            success=true
            break
        else
            rm -rf "$clone_path"
        fi
    done

    if [ "$success" = false ]; then
        # Fallback to cloning default branch
        echo "Cloning default branch without specifying tag..."
        if ! git clone --depth 1 "$url" "$clone_path" &> /dev/null; then
            echo "Failed to clone $name"
            rm -rf "$clone_path"
            continue
        fi
    fi

    # Copy .cmake, .json, .png, .jpg, .obj files
    if [ -d "$clone_path" ]; then
        copied_count=0
        # Find files inside clone_path with given extensions and copy preserving directory structure
        (
            cd "$clone_path"
            find . -type f \( -name "*.cmake" -o -name "*.json" -o -name "*.png" -o -name "*.jpg" -o -name "*.obj" \)
        ) | while read -r rel_file; do
            rel_file="${rel_file#./}"
            src_file="$clone_path/$rel_file"
            dest_file="$target_dep_dir/$rel_file"
            mkdir -p "$(dirname "$dest_file")"
            cp -p "$src_file" "$dest_file"
        done

        copied_count=$(find "$clone_path" -type f \( -name "*.cmake" -o -name "*.json" -o -name "*.png" -o -name "*.jpg" -o -name "*.obj" \) | wc -l)
        echo "Successfully copied $copied_count files for $name"

        rm -rf "$clone_path"
        touch "$target_dep_dir/.downloaded"
    fi
done

# Clean up temp_dir
rm -rf "$TEMP_DIR"
echo "Done restoring all dependencies!"

echo "======================================"
echo " Building all dependencies in deps/src..."
echo "======================================"

cd "$DEPS_DIR"
colcon build --merge-install --base-paths src --install-base install --cmake-args -DCMAKE_BUILD_TYPE=Release -DTESSERACT_BUILD_TASK_COMPOSER_TASKFLOW=OFF -DTESSERACT_ENABLE_TESTING=OFF -DTESSERACT_ENABLE_EXAMPLES=OFF

