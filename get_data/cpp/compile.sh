#!/bin/sh
# The recorders are built inside the SDK trees, whose CMakeLists reference
# symlinks to the sources in this folder. Make sure the shared header is
# linked there too, then build.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
LIVOX_SAMPLES=/home/ubm/SDKs/Livox-SDK2/samples/livox_lidar_quick_start
HESAI_TOOL=/home/ubm/SDKs/HesaiLidar_SDK_2.0/tool

for dir in "$LIVOX_SAMPLES" "$HESAI_TOOL"; do
    ln -sfn "$HERE/h5_point_writer.hpp" "$dir/h5_point_writer.hpp"
done

make --directory=/home/ubm/SDKs/Livox-SDK2/build save_point_cloud_livox
make --directory="$HESAI_TOOL/build" save_point_cloud_hesai
