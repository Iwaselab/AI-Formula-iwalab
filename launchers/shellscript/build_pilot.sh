#!/bin/bash
set -e

TARGET_PACKEGES=(
    launchers
    common_python

    # remote control
    compressed_image_viewer
    teleop_twist_handle_controller
)

echo "Build Targets:"
for packege in ${TARGET_PACKEGES[@]}; do
    echo " - ${packege}"
done
echo ""

cd ${HOME}/workspace/ros
colcon build --symlink-install --packages-up-to ${TARGET_PACKEGES[@]}
