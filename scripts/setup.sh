#!/bin/bash
# setup.sh - Workspace setup script for ROS 2 Humble
# Handles repository import, dependency installation, and environment setup
set -e

# ============================================================================
# Configuration
# ============================================================================
WORKSPACE="${WORKSPACE:-$(pwd)}"
PARALLEL_WORKERS=$(($(nproc) / 2))

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info()  { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*"; }

# ============================================================================
# Repository Management
# ============================================================================
update_submodules() {
    # 自动检测：如果子模块目录已存在且包含 .git 文件夹，则跳过
    if [[ -d "libs/lerobot/.git" && -d "src/.git" ]]; then
        log_info "Submodules already initialized. Skipping update."
        return 0
    fi

    log_info "Updating git submodules (skipping LFS)..."
    export GIT_LFS_SKIP_SMUDGE=1
    git submodule update --init --recursive
}

setup_developer_forks() {
    echo ""
    echo -e "${YELLOW}--- Developer Fork Setup ---${NC}"
    echo "If you have forked the repositories on GitCode, enter your username"
    echo "to automatically set up your personal forks as 'origin' and the"
    echo "original repositories as 'upstream'."
    echo ""
    read -p "Enter your GitCode username (leave empty to skip): " USERNAME

    if [[ -n "${USERNAME}" ]]; then
        local LEROBOT_FORK="git@gitcode.com:${USERNAME}/lerobot_ros2.git"
        local LEDOG_FORK="git@gitcode.com:${USERNAME}/ledog_ros2.git"

        echo -e "\nProposed Fork URLs:"
        echo -e "  libs/lerobot: ${LEROBOT_FORK}"
        echo -e "  src:          ${LEDOG_FORK}"
        echo ""
        read -p "Confirm setting these as 'origin'? [y/N]: " CONFIRM

        if [[ "${CONFIRM}" == "y" || "${CONFIRM}" == "Y" ]]; then
            log_info "Configuring personal forks (local only)..."
            
            # Get original URLs (upstream)
            local LEROBOT_UPSTREAM=$(git config -f .gitmodules submodule.libs/lerobot.url)
            local LEDOG_UPSTREAM=$(git config -f .gitmodules submodule.src.url)

            # 1. Update origin URLs inside submodules directly
            # This doesn't touch .gitmodules
            (cd libs/lerobot && git remote set-url origin "${LEROBOT_FORK}")
            (cd src && git remote set-url origin "${LEDOG_FORK}")

            # 2. Add/Update upstream remotes
            (cd libs/lerobot && git remote add upstream "${LEROBOT_UPSTREAM}" 2>/dev/null || git remote set-url upstream "${LEROBOT_UPSTREAM}")
            (cd src && git remote add upstream "${LEDOG_UPSTREAM}" 2>/dev/null || git remote set-url upstream "${LEDOG_UPSTREAM}")

            log_info "Forks configured locally! .gitmodules remains pointing to upstream."
        else
            log_info "Fork setup cancelled."
        fi
    else
        log_info "Skipping fork setup. Using default repositories."
    fi
}

# ============================================================================
# Dependency Management
# ============================================================================
install_system_deps() {
    log_info "Updating apt package lists..."
    sudo apt-get update -qq
    
    log_info "Updating rosdep database..."
    rosdep update --rosdistro=humble
    
    log_info "Installing ROS dependencies..."
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro=humble \
        -y -r \
        --skip-keys "catkin roscpp lerobot trimesh[easy] simple-parsing cupy-cuda12x ctl_system_interface numpy_lessthan_2 ament_python feetech-servo-sdk pyserial"
}

setup_python_venv() {
    local venv_path="${WORKSPACE}/venv"
    
    # 1. 确保系统级 venv 工具已安装
    log_info "Checking for Python venv and pip..."
    sudo apt-get update -qq
    sudo apt-get install -y python3-venv python3-pip -qq

    # 2. 创建虚拟环境 (必须包含 --system-site-packages 以使用系统的 rclpy)
    if [[ ! -d "${venv_path}" ]]; then
        log_info "Creating virtual environment at ${venv_path} with --system-site-packages..."
        python3 -m venv --system-site-packages "${venv_path}"
    else
        log_info "Virtual environment already exists at ${venv_path}."
    fi

    # 3. 激活虚拟环境并安装依赖
    log_info "Configuring Python environment and dependencies..."
    source "${venv_path}/bin/activate"
    
    # 升级 pip
    python3 -m pip install --upgrade pip --quiet
    
    # 解决 setuptools 版本冲突 (兼容 LeRobot 和 colcon)
    python3 -m pip install "setuptools<80" "setuptools>=71" --quiet
    
    # 以可编辑模式安装 LeRobot
    if [[ -d "${WORKSPACE}/libs/lerobot" ]]; then
        log_info "Installing LeRobot in editable mode..."
        python3 -m pip install -e "${WORKSPACE}/libs/lerobot"
    fi

    # 安装原有的硬件依赖
    log_info "Installing hardware dependencies (pyserial, feetech)..."
    python3 -m pip install pyserial feetech-servo-sdk --quiet

    # 4. 环境验证
    log_info "Verifying ROS 2 connection..."
    # 显式加载 ROS 2 环境后再验证
    if (source /opt/ros/humble/setup.sh && python3 -c "import rclpy; print('ROS 2 Humble connection successful')") 2>/dev/null; then
        log_info "Verification complete: venv can access ROS 2 packages."
    else
        log_error "Verification failed: rclpy not found. Ensure ROS 2 is installed and --system-site-packages was used."
    fi
}

# ============================================================================
# Main
# ============================================================================
main() {
    cd "${WORKSPACE}"
    
    log_info "Setting up workspace at ${WORKSPACE}"
    
    # Update submodules
    update_submodules
    
    # Optional: Setup developer forks
    setup_developer_forks
    
    # Install dependencies
    install_system_deps
    setup_python_venv
    
    log_info "Setup complete! Run ./scripts/build.sh to build the workspace."
}

# Run if executed directly (not sourced)
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi