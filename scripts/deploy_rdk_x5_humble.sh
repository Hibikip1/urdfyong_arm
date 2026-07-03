#!/usr/bin/env bash
# Deploy urdfyong_arm on D-Robotics RDK X5.
#
# Target platform:
#   - Board: RDK X5
#   - OS: Ubuntu 22.04
#   - CPU: ARM64/aarch64
#   - ROS: ROS_DISTRO=humble
#
# Run this script from the repository root on the RDK X5:
#   ./scripts/deploy_rdk_x5_humble.sh
#
# Useful options:
#   --skip-apt       Do not configure apt or install apt packages.
#   --skip-rosdep    Do not run rosdep init/update/install.
#   --skip-u2can     Do not build the u2can test tools.
#   --workers N      Limit build parallelism. Default: 2.
#   --serial-port P  USB2CAN serial device. Default: /dev/ttyACM0.

set -Eeuo pipefail

ROS_DISTRO=humble
WORKERS=2
SERIAL_PORT=/dev/ttyACM0
SKIP_APT=0
SKIP_ROSDEP=0
SKIP_U2CAN=0

usage() {
  sed -n '1,26p' "$0"
}

log() {
  printf '\n[deploy-rdk-x5] %s\n' "$*"
}

warn() {
  printf '\n[deploy-rdk-x5][warn] %s\n' "$*" >&2
}

die() {
  printf '\n[deploy-rdk-x5][error] %s\n' "$*" >&2
  exit 1
}

run() {
  printf '+'
  printf ' %q' "$@"
  printf '\n'
  "$@"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    --skip-rosdep)
      SKIP_ROSDEP=1
      shift
      ;;
    --skip-u2can)
      SKIP_U2CAN=1
      shift
      ;;
    --workers)
      [[ $# -ge 2 ]] || die "--workers needs a value"
      WORKERS="$2"
      shift 2
      ;;
    --serial-port)
      [[ $# -ge 2 ]] || die "--serial-port needs a value"
      SERIAL_PORT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

case "$WORKERS" in
  ''|*[!0-9]*)
    die "--workers must be a positive integer"
    ;;
  0)
    die "--workers must be greater than 0"
    ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

[[ -d urdfyong && -d urdfyong_hardware && -d urdfyong_moveit_config ]] \
  || die "run this script from an urdfyong_arm checkout"

if [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
  die "do not run this script with sudo; run it as the target user and enter sudo when prompted"
fi

check_platform() {
  log "Checking target platform"

  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
      warn "Expected Ubuntu 22.04 for ROS 2 Humble; detected ${PRETTY_NAME:-unknown}."
    fi
  else
    warn "Cannot read /etc/os-release; skipping OS check."
  fi

  case "$(uname -m)" in
    aarch64|arm64)
      ;;
    *)
      warn "Expected ARM64/aarch64 RDK X5; detected $(uname -m). Continuing anyway."
      ;;
  esac
}

configure_ros_apt() {
  [[ "$SKIP_APT" -eq 0 ]] || {
    log "Skipping apt setup by request"
    return
  }

  log "Installing base tools and configuring ROS 2 apt repository"
  run sudo apt-get update
  run sudo apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common

  run sudo add-apt-repository -y universe

  if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
    run sudo curl -sSL \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi

  if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
    local codename
    codename="$(. /etc/os-release && printf '%s' "$UBUNTU_CODENAME")"
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu %s main\n' \
      "$(dpkg --print-architecture)" "$codename" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi

  run sudo apt-get update
}

install_apt_packages() {
  [[ "$SKIP_APT" -eq 0 ]] || return

  log "Installing ROS 2 Humble, MoveIt 2, ros2_control, and build tools"
  run sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    "ros-${ROS_DISTRO}-controller-manager" \
    "ros-${ROS_DISTRO}-gazebo-ros" \
    "ros-${ROS_DISTRO}-gazebo-ros2-control" \
    "ros-${ROS_DISTRO}-joint-state-broadcaster" \
    "ros-${ROS_DISTRO}-joint-state-publisher" \
    "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
    "ros-${ROS_DISTRO}-joint-trajectory-controller" \
    "ros-${ROS_DISTRO}-moveit" \
    "ros-${ROS_DISTRO}-moveit-configs-utils" \
    "ros-${ROS_DISTRO}-robot-state-publisher" \
    "ros-${ROS_DISTRO}-ros2-control" \
    "ros-${ROS_DISTRO}-ros2-controllers" \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-warehouse-ros-mongo" \
    "ros-${ROS_DISTRO}-xacro"
}

setup_rosdep() {
  [[ "$SKIP_ROSDEP" -eq 0 ]] || {
    log "Skipping rosdep by request"
    return
  }

  log "Resolving package dependencies with rosdep"
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    run sudo rosdep init
  fi
  run rosdep update
  run rosdep install -r -y \
    --from-paths urdfyong urdfyong_hardware urdfyong_moveit_config \
    --ignore-src \
    --rosdistro "$ROS_DISTRO"
}

build_ros_workspace() {
  [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] \
    || die "/opt/ros/${ROS_DISTRO}/setup.bash not found. Install ROS 2 Humble first or rerun without --skip-apt."

  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"

  log "Building ROS workspace with colcon"
  run colcon build \
    --symlink-install \
    --parallel-workers "$WORKERS" \
    --packages-select urdfyong urdfyong_hardware urdfyong_moveit_config
}

build_u2can_tools() {
  [[ "$SKIP_U2CAN" -eq 0 ]] || {
    log "Skipping u2can build by request"
    return
  }

  log "Building u2can test tools"
  run cmake -S u2can -B u2can/build -DCMAKE_BUILD_TYPE=Release
  run cmake --build u2can/build --parallel "$WORKERS"
}

check_serial_permissions() {
  log "Checking USB2CAN serial device permissions"

  if [[ -e "$SERIAL_PORT" ]]; then
    run ls -l "$SERIAL_PORT"
  else
    warn "$SERIAL_PORT does not exist yet. Plug in USB2CANFD and verify the device path."
  fi

  if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    log "User '$USER' is already in dialout."
  else
    warn "User '$USER' is not in dialout; adding it so $SERIAL_PORT can be opened without sudo."
    run sudo usermod -aG dialout "$USER"
    warn "Log out and log back in before running the hardware launch."
  fi
}

print_next_steps() {
  cat <<EOF

[deploy-rdk-x5] Deployment finished.

Next terminal on the RDK X5:
  cd "$REPO_ROOT"
  source /opt/ros/${ROS_DISTRO}/setup.bash
  source install/setup.bash
  ros2 launch urdfyong_hardware hardware_moveit.launch.py

Hardware defaults used by this repository:
  USB2CAN serial: ${SERIAL_PORT}
  Baud rate:      921600

For headless deployment, consider running RViz/MoveIt visualization on the
development PC and keeping the RDK X5 focused on ros2_control + USB2CAN.
EOF
}

check_platform
configure_ros_apt
install_apt_packages
setup_rosdep
build_ros_workspace
build_u2can_tools
check_serial_permissions
print_next_steps
