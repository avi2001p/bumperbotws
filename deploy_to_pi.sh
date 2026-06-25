#!/bin/bash
# =============================================================
# deploy_to_pi.sh
# =============================================================
# Deploys only the necessary BumperBot packages to Raspberry Pi.
#
# Usage:
#   ./deploy_to_pi.sh <PI_USER>@<PI_IP>
#
# Example:
#   ./deploy_to_pi.sh pi@192.168.1.100
#
# This copies ONLY the 2 packages needed on the Pi:
#   - bumperbot_hardware (motor, encoder, PID, odometry, water actuator)
#   - bumperbot_coverage (stadium coverage planner)
#
# It does NOT copy simulation, URDF, Gazebo, or example packages.
# =============================================================

set -e

# --- Configuration ---
PI_TARGET="${1:?Usage: $0 <user@pi_ip>}"
PI_WORKSPACE="bumperbot_wsv2"
SRC_DIR="$(cd "$(dirname "$0")" && pwd)/src"

echo "=============================================="
echo "  BumperBot Deployment to Raspberry Pi"
echo "=============================================="
echo "  Target: ${PI_TARGET}:~/${PI_WORKSPACE}"
echo "  Source: ${SRC_DIR}"
echo "=============================================="
echo ""

# --- Packages to deploy ---
PACKAGES=(
    "bumperbot_hardware"
    "bumperbot_coverage"
    "bumperbot_mapping"
)

# --- Step 1: Create workspace structure on Pi ---
echo "[1/4] Creating workspace structure on Pi..."
ssh "${PI_TARGET}" "mkdir -p ~/${PI_WORKSPACE}/src"

# --- Step 2: Copy packages ---
echo "[2/4] Copying packages to Pi..."
for pkg in "${PACKAGES[@]}"; do
    echo "  → ${pkg}"
    rsync -avz --delete \
        "${SRC_DIR}/${pkg}/" \
        "${PI_TARGET}:~/${PI_WORKSPACE}/src/${pkg}/"
done

# --- Step 3: Install Python dependencies on Pi ---
echo ""
echo "[3/4] Installing Python dependencies on Pi..."
ssh "${PI_TARGET}" bash -s << 'REMOTE_SCRIPT'
set -e

echo "  Checking RPi.GPIO..."
pip3 install --quiet RPi.GPIO 2>/dev/null || echo "  RPi.GPIO already installed or using system package"

echo "  Checking transforms3d (for tf_transformations)..."
pip3 install --quiet transforms3d 2>/dev/null || echo "  transforms3d already installed"

# Check if tf_transformations ROS package exists, if not install via pip
python3 -c "import tf_transformations" 2>/dev/null || {
    echo "  Installing tf_transformations via pip..."
    pip3 install --quiet tf_transformations 2>/dev/null || echo "  May need: sudo apt install ros-${ROS_DISTRO}-tf-transformations"
}

echo "  Python dependencies OK."
REMOTE_SCRIPT

# --- Step 4: Build on Pi ---
echo ""
echo "[4/4] Building workspace on Pi..."
ssh "${PI_TARGET}" bash -s << REMOTE_BUILD
set -e
cd ~/${PI_WORKSPACE}

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
else
    echo "ERROR: Could not find ROS 2 installation!"
    echo "Available in /opt/ros/:"
    ls /opt/ros/ 2>/dev/null || echo "  (none found)"
    exit 1
fi

echo "  Using ROS 2: \${ROS_DISTRO}"
echo "  Building bumperbot_hardware and bumperbot_coverage..."

colcon build --packages-select bumperbot_hardware bumperbot_coverage bumperbot_mapping --symlink-install

echo ""
echo "  Build complete!"
REMOTE_BUILD

echo ""
echo "=============================================="
echo "  ✅ DEPLOYMENT COMPLETE!"
echo "=============================================="
echo ""
echo "  SSH into your Pi and run:"
echo ""
echo "    ssh ${PI_TARGET}"
echo "    cd ~/${PI_WORKSPACE}"
echo "    source install/setup.bash"
echo ""
echo "  Then to test hardware:"
echo "    ros2 launch bumperbot_hardware hardware.launch.py"
echo ""
echo "  To run full coverage mission:"
echo "    ros2 launch bumperbot_hardware coverage_mission.launch.py"
echo ""
