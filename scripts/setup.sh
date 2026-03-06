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
# Environment Checks
# ============================================================================
check_conda() {
    if [[ -n "${CONDA_PREFIX}" ]]; then
        log_error "Active Conda environment detected at: ${CONDA_PREFIX}"
        log_warn "Conda environments are known to conflict with ROS 2 dependencies (especially Python libraries)."
        log_warn "Please deactivate the Conda environment before running this script:"
        echo -e "    ${YELLOW}conda deactivate${NC}"
        exit 1
    fi
}

# ============================================================================
# Repository Management
# ============================================================================
update_submodules() {
    echo ""
    echo -e "${YELLOW}--- Git Submodule Management ---${NC}"

    # Define submodules
    local submodules=(
        "libs/lerobot:LeRobot"
        "src/pymoveit2:PyMoveIt2"
    )

    # Check which submodules need initialization
    local need_init=()
    for entry in "${submodules[@]}"; do
        local path="${entry%%:*}"
        local name="${entry##*:}"
        if [[ ! -d "${path}/.git" ]]; then
            need_init+=("${path}:${name}")
        fi
    done

    # If all submodules exist, ask if user wants to update
    if [[ ${#need_init} -eq 0 ]]; then
        log_info "All submodules are already initialized:"
        for entry in "${submodules[@]}"; do
            local path="${entry%%:*}"
            local name="${entry##*:}"
            echo "  ✓ ${name} (${path})"
        done
        echo ""
        read -p "Do you want to sync/update all submodules? [y/N]: " CONFIRM
        if [[ "${CONFIRM}" != "y" && "${CONFIRM}" != "Y" ]]; then
            log_info "Skipping submodule update."
            return 0
        fi
        log_info "Updating all submodules..."
        export GIT_LFS_SKIP_SMUDGE=1
        git submodule update --init --recursive
        return 0
    fi

    # Some submodules need initialization
    log_warn "The following submodules are not initialized:"
    for entry in "${need_init[@]}"; do
        local path="${entry%%:*}"
        local name="${entry##*:}"
        echo "  ✗ ${name} (${path})"
    done
    echo ""

    # Ask which submodules to initialize
    log_info "Select which submodules to initialize:"
    echo "  1) All submodules"
    echo "  2) LeRobot only (libs/lerobot)"
    echo "  3) PyMoveIt2 only (src/pymoveit2)"
    echo "  4) Select individually"
    echo "  0) Skip"
    echo ""
    read -p "Enter your choice [1-4, 0]: " CHOICE

    case "${CHOICE}" in
        1)
            log_info "Initializing all submodules..."
            export GIT_LFS_SKIP_SMUDGE=1
            git submodule update --init --recursive
            ;;
        2)
            log_info "Initializing LeRobot (libs/lerobot)..."
            export GIT_LFS_SKIP_SMUDGE=1
            git submodule update --init --recursive libs/lerobot
            ;;
        3)
            log_info "Initializing PyMoveIt2 (src/pymoveit2)..."
            export GIT_LFS_SKIP_SMUDGE=1
            git submodule update --init --recursive src/pymoveit2
            ;;
        4)
            echo ""
            for entry in "${need_init[@]}"; do
                local path="${entry%%:*}"
                local name="${entry##*:}"
                read -p "Initialize ${name} (${path})? [Y/n]: " CONFIRM
                if [[ "${CONFIRM}" != "n" && "${CONFIRM}" != "N" ]]; then
                    log_info "Initializing ${name}..."
                    export GIT_LFS_SKIP_SMUDGE=1
                    git submodule update --init --recursive "${path}"
                else
                    log_warn "Skipped ${name}"
                fi
            done
            ;;
        0)
            log_warn "Submodule initialization skipped."
            ;;
        *)
            log_error "Invalid choice. Skipping submodule initialization."
            ;;
    esac
}

setup_developer_forks() {
    echo ""
    echo -e "${YELLOW}--- Developer Fork Setup ---${NC}"
    echo "If you have forked the repository on GitCode, enter your username"
    echo "to automatically set up your personal fork as 'origin' and the"
    echo "original repository as 'upstream'."
    echo ""
    read -p "Enter your GitCode username (leave empty to skip): " USERNAME

    if [[ -n "${USERNAME}" ]]; then
        local MAIN_FORK="git@gitcode.com:${USERNAME}/IB_Robot.git"
        local LEROBOT_FORK="git@gitcode.com:${USERNAME}/lerobot_ros2.git"

        echo -e "\nProposed Fork URLs:"
        echo -e "  Main Repo:    ${MAIN_FORK}"
        echo -e "  libs/lerobot: ${LEROBOT_FORK}"
        echo ""
        read -p "Confirm setting these as 'origin'? [y/N]: " CONFIRM

        if [[ "${CONFIRM}" == "y" || "${CONFIRM}" == "Y" ]]; then
            log_info "Configuring personal forks..."
            
            # 1. Update main repo remotes
            git remote set-url origin "${MAIN_FORK}"
            git remote add upstream git@atomgit.com:openeuler/IB_Robot.git 2>/dev/null || git remote set-url upstream git@atomgit.com:openeuler/IB_Robot.git

            # 2. Update submodule fork
            if [[ -d "libs/lerobot/.git" ]]; then
                (cd libs/lerobot && git remote set-url origin "${LEROBOT_FORK}")
                local LEROBOT_UPSTREAM=$(git config -f .gitmodules submodule.libs/lerobot.url)
                (cd libs/lerobot && git remote add upstream "${LEROBOT_UPSTREAM}" 2>/dev/null || git remote set-url upstream "${LEROBOT_UPSTREAM}")
            fi

            log_info "Forks configured successfully!"
        else
            log_info "Fork setup cancelled."
        fi
    else
        log_info "Skipping fork setup."
    fi
}

# ============================================================================
# Dependency Management
# ============================================================================
install_system_deps() {
    if command -v apt-get &> /dev/null; then
        log_info "Updating apt package lists..."
        sudo apt-get update -qq
        
        log_info "Updating rosdep database..."
        rosdep update --rosdistro=humble
        
        log_info "Installing ROS dependencies via apt..."
        rosdep install \
            --from-paths src \
            --ignore-src \
            --rosdistro=humble \
            -y -r \
            --skip-keys "catkin roscpp lerobot trimesh[easy] simple-parsing cupy-cuda12x ctl_system_interface numpy_lessthan_2 ament_python feetech-servo-sdk pyserial"
    elif command -v dnf &> /dev/null; then
        log_info "Updating dnf package repositories..."
        # openEuler Embedded might not need full dnf update every time
        
        log_info "Updating rosdep database..."
        rosdep update --rosdistro=humble
        
        log_info "Installing ROS dependencies via dnf..."
        rosdep install \
            --from-paths src \
            --ignore-src \
            --rosdistro=humble \
            -y -r \
            --skip-keys "catkin roscpp lerobot trimesh[easy] simple-parsing cupy-cuda12x ctl_system_interface numpy_lessthan_2 ament_python feetech-servo-sdk pyserial"
    else
        log_warn "Unknown package manager. Please ensure ROS 2 Humble dependencies are installed manually."
    fi
}

setup_python_venv() {
    local venv_path="${WORKSPACE}/venv"
    
    # 1. Ensure system-level venv tools are installed
    log_info "Checking for Python venv and pip..."
    if command -v apt-get &> /dev/null; then
        sudo apt-get update -qq
        sudo apt-get install -y python3-venv python3-pip -qq
    elif command -v dnf &> /dev/null; then
        sudo dnf install -y python3-virtualenv python3-pip -q
    fi

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
    
    # 核心修复：强制安装 NumPy 1.x 以兼容 ROS 2 系统组件
    log_info "Ensuring NumPy 1.x compatibility..."
    python3 -m pip install "numpy<2" --quiet

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

    # 安装 scipy 用于数学计算 (四元数/旋转矩阵转换)
    log_info "Installing scipy for mathematical computations..."
    python3 -m pip install scipy --quiet

    # 安装 gitlint 并设置 git hook
    log_info "Installing gitlint..."
    python3 -m pip install gitlint --quiet
    log_info "Installing gitlint pre-commit hook..."
    gitlint install-hook

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
    
    # Check for conflicting environments
    check_conda
    
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