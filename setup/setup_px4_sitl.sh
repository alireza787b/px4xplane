#!/bin/bash

# === PX4 X-Plane SITL Setup Script ===
#
# Author: Alireza Ghaderi
# LinkedIn: alireza787b
# Updated: June 2026
# GitHub Repo: alireza787b/px4xplane
#
# === Purpose ===
# This script automates the setup of PX4 Software-In-The-Loop (SITL) integration with X-Plane.
#
# ⚠️  TESTING PHASE - TEMPORARY WORKAROUND ⚠️
# This script is currently in the TESTING PHASE. It clones a forked version of PX4
# (https://github.com/alireza787b/PX4-Autopilot-Me.git) which contains X-Plane SITL support
# that is being prepared for submission to the official PX4 repository.
#
# Once the Pull Request is merged into PX4/PX4-Autopilot, you will be able to use the
# official PX4 repository directly, and this custom fork will no longer be needed.
#
# Current Status:
# - Fork Branch: px4xplane-sitl
# - Target: Official PX4 repository (pending PR approval)
# - Progress: https://github.com/PX4/PX4-Autopilot/pull/22493
#
# === Functionality ===
# The script performs the following tasks:
# 1. Clones the PX4 repository from the forked repo (customized for X-Plane).
# 2. Auto-syncs with remote updates (smart pull detection).
# 3. Sets up the necessary dependencies by running the provided setup script (`ubuntu.sh`).
# 4. Optionally sets up MAVLink Router for communication with X-Plane.
# 5. Optionally sets up global access to the `px4xplane` command.
# 6. Prompts the user to select a platform to build and initiates the build process.
#
# === Key Features ===
# - **Auto-Sync**: Automatically detects and pulls updates from the remote repository
# - **Smart Workflow**: Skips already-completed steps for faster subsequent runs
# - **Beginner-Friendly**: Clear prompts with sensible defaults and timeouts
# - **Expert Options**: Full control via flags and manual overrides
# - **Robust Error Handling**: Graceful fallbacks and informative error messages
# - **Progress Indicators**: Visual feedback for all major operations
# - **MAVLink Router**: Optional automated setup for network communication
# - **Global Access**: Optional `px4xplane` command for system-wide access
# - **Repair Mode**: Force full re-setup if needed
#
# === Usage ===
# - **Normal Run**: Run the script without any arguments
#   Example: `./setup_px4_sitl.sh`
# - **Custom Installation Path**: Specify a custom installation directory
#   Example: `./setup_px4_sitl.sh ~/custom_install_path`
# - **Repair Mode**: Force full setup process (useful for troubleshooting)
#   Example: `./setup_px4_sitl.sh --repair`
# - **Uninstall**: Remove the global `px4xplane` command and installation
#   Example: `./setup_px4_sitl.sh --uninstall`
#
# === Customization Options ===
# The following variables can be customized at the top of the script:
# - `REPO_URL`: URL of the forked PX4 repository
# - `BRANCH_NAME`: Branch to checkout (currently: px4xplane-sitl)
# - `UPSTREAM_URL`: URL of the upstream PX4 repository
# - `DEFAULT_CLONE_PATH`: Default installation directory
# - `PLATFORM_CHOICES`: Available airframes for X-Plane
# - `USE_MAVLINK_ROUTER`: Enable/disable MAVLink Router setup
#
# === Requirements ===
# - Git (required)
# - Ubuntu/Debian-based Linux (or WSL on Windows)
# - X-Plane 11 or 12
# - px4xplane bridge plugin (download from: https://github.com/alireza787b/px4xplane/releases)
#
# === Supported Aircraft ===
# - xplane_cessna172: Cessna 172 (fixed-wing trainer)
# - xplane_tb2: TB2 UAV (fixed-wing)
# - xplane_ehang184: Ehang 184 (quadcopter airtaxi)
# - xplane_alia250: Alia-250 (eVTOL quadplane)
# - xplane_qtailsitter: QuadTailsitter (small VTOL tailsitter)
#
# === Future ===
# Once the PR is merged into the official PX4 repository, this script will be updated
# to use the official repository, eliminating the need for the custom fork.
#
# For updates and more information, visit:
# - PX4 X-Plane Integration: https://github.com/alireza787b/px4xplane
# - This Fork: https://github.com/alireza787b/PX4-Autopilot-Me
# - Official PX4: https://github.com/PX4/PX4-Autopilot

# === Configurable Variables ===
REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
BRANCH_NAME="px4xplane-sitl"
EKF_GSF_GUARD_BRANCH="ekf2-gnss-yaw-reset-guard"
VTOL_HANDOFF_GUARD_BRANCH="vtol-standard-throttle-handoff-xplane"
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME"
DEFAULT_CONFIG_FILE="$HOME/.px4sitl_config"
DEFAULT_FALLBACK_IP="127.0.0.1"
SCRIPT_NAME="px4xplane_script.sh"
MAVLINK2REST_IP="127.0.0.1"
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2" "xplane_qtailsitter")
SETUP_SCRIPT_URL="https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup/setup_px4_sitl.sh"

# === MAVLink Router Configuration ===
USE_MAVLINK_ROUTER=true  # Set to true to enable MAVLink Router installation and setup
MAVLINK_ROUTER_INSTALL_SCRIPT_URL="https://raw.githubusercontent.com/alireza787b/mavlink-anywhere/main/install_mavlink_router.sh"
MAVLINK_ROUTER_COMMAND="mavlink-routerd -e __PX4_SIM_HOSTNAME__:14540 -e __PX4_SIM_HOSTNAME__:14550 -e __PX4_SIM_HOSTNAME__:14569 -e __MAVLINK2REST_IP__:14569 0.0.0.0:14550"

# === Repair Mode (Configurable) ===
REPAIR_MODE=false  # Set to true if you always want full repair mode

# === Flags and Arguments ===
REPAIR_MODE_FLAG="--repair"
UNINSTALL_FLAG="--uninstall"
SYNC_MODE_FLAG="--sync"
RESET_CONFIG_FLAG="--reset-config"
RESET_IP_FLAG="--reset-ip"
EKF_GSF_GUARD_FLAG="--with-ekf-gsf-guard"
NO_EKF_GSF_GUARD_FLAG="--without-ekf-gsf-guard"
VTOL_HANDOFF_GUARD_FLAG="--with-vtol-handoff-guard"
NO_VTOL_HANDOFF_GUARD_FLAG="--without-vtol-handoff-guard"
HELP_FLAG="--help"

SYNC_MODE=false
RESET_CONFIG_MODE=false
RESET_IP_MODE=false
UNINSTALL_MODE=false
APPLY_EKF_GSF_GUARD=false
SKIP_EKF_GSF_GUARD=false
APPLY_VTOL_HANDOFF_GUARD=false
SKIP_VTOL_HANDOFF_GUARD=false

print_usage() {
    cat <<EOF
Usage: $0 [options] [install_path]

Options:
  --sync          Force-sync PX4-Autopilot-Me to origin/$BRANCH_NAME, update submodules,
                  and reset saved SITL parameters before launching.
  --repair        Run the full dependency/setup path and sync the PX4 fork.
  --reset-ip      Ignore the saved PX4_SIM_HOSTNAME and choose auto/default IP again.
  --reset-config  Remove saved px4xplane CLI config and re-prompt all remembered values.
  --with-ekf-gsf-guard
                  Local test mode: sync the X-Plane PR branch, then cherry-pick
                  the open EKF-GSF yaw reset guard PR on top before launching.
  --without-ekf-gsf-guard
                  Do not prompt for or apply the temporary EKF-GSF guard.
  --with-vtol-handoff-guard
                  Local test mode: also cherry-pick the open Standard VTOL
                  front-transition setpoint handoff PR before launching.
  --without-vtol-handoff-guard
                  Do not prompt for or apply the temporary VTOL handoff guard.
  --uninstall     Remove the global px4xplane command and default PX4 fork checkout.
  --help          Show this help.
EOF
}

POSITIONAL_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        "$UNINSTALL_FLAG")
            UNINSTALL_MODE=true
            shift
            ;;
        "$REPAIR_MODE_FLAG")
            REPAIR_MODE=true
            shift
            ;;
        "$SYNC_MODE_FLAG")
            SYNC_MODE=true
            shift
            ;;
        "$RESET_CONFIG_FLAG")
            RESET_CONFIG_MODE=true
            RESET_IP_MODE=true
            shift
            ;;
        "$RESET_IP_FLAG")
            RESET_IP_MODE=true
            shift
            ;;
        "$EKF_GSF_GUARD_FLAG")
            APPLY_EKF_GSF_GUARD=true
            SYNC_MODE=true
            shift
            ;;
        "$NO_EKF_GSF_GUARD_FLAG")
            SKIP_EKF_GSF_GUARD=true
            APPLY_EKF_GSF_GUARD=false
            shift
            ;;
        "$VTOL_HANDOFF_GUARD_FLAG")
            APPLY_VTOL_HANDOFF_GUARD=true
            SYNC_MODE=true
            shift
            ;;
        "$NO_VTOL_HANDOFF_GUARD_FLAG")
            SKIP_VTOL_HANDOFF_GUARD=true
            APPLY_VTOL_HANDOFF_GUARD=false
            shift
            ;;
        "$HELP_FLAG"|"-h")
            print_usage
            exit 0
            ;;
        --*)
            echo "Unknown option: $1"
            print_usage
            exit 1
            ;;
        *)
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done
set -- "${POSITIONAL_ARGS[@]}"

if [ "$UNINSTALL_MODE" = true ]; then
    # Uninstall the global access command
    echo "Uninstalling global access for px4xplane..."
    if [ -L "$HOME/bin/px4xplane" ] || [ -e "$HOME/bin/px4xplane" ]; then
        rm "$HOME/bin/px4xplane"
        echo "Removed global command from $HOME/bin."
    elif [ -L "$HOME/.local/bin/px4xplane" ] || [ -e "$HOME/.local/bin/px4xplane" ]; then
        rm "$HOME/.local/bin/px4xplane"
        echo "Removed global command from $HOME/.local/bin."
    else
        echo "Global command not found."
    fi

    # Attempt to remove PX4-Autopilot-Me directory and config file
    if [ -d "$DEFAULT_CLONE_PATH/PX4-Autopilot-Me" ]; then
        sudo rm -rf "$DEFAULT_CLONE_PATH/PX4-Autopilot-Me"
        echo "Removed PX4-Autopilot-Me Cloned Repository."
    fi
    if [ -e "$DEFAULT_CONFIG_FILE" ]; then
        rm "$DEFAULT_CONFIG_FILE"
        echo "Removed .px4sitl_config file."
    fi

    echo "Uninstallation complete."
    exit 0
fi

# === Check for Custom Installation Directory Parameter ===
if [ -n "$1" ]; then
    INSTALL_PATH="$1"
    CLONE_PATH="$INSTALL_PATH/PX4-Autopilot-Me"
    CONFIG_FILE="$INSTALL_PATH/.px4sitl_config"
else
    INSTALL_PATH="$DEFAULT_CLONE_PATH"
    CLONE_PATH="$INSTALL_PATH/PX4-Autopilot-Me"
    CONFIG_FILE="$DEFAULT_CONFIG_FILE"
fi

# === Create Parent Directory if Needed ===
if [ ! -d "$INSTALL_PATH" ]; then
    echo "Directory $INSTALL_PATH does not exist. Creating it..."
    mkdir -p "$INSTALL_PATH"
fi

# === Copy Script to Parent Directory ===
SCRIPT_PATH="$INSTALL_PATH/$SCRIPT_NAME"
if [ "$(basename "$0")" != "$SCRIPT_NAME" ]; then
    echo "Copying script to $SCRIPT_PATH..."
    cp "$0" "$SCRIPT_PATH"
    chmod +x "$SCRIPT_PATH"
fi


# === Distinguish Important Prompts ===
highlight() {
    echo -e "\n\033[1;33m$1\033[0m\n"  # Yellow bold text
}

info() {
    echo -e "\033[1;36m[INFO]\033[0m $1"  # Cyan info messages
}

success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"  # Green success messages
}

warning() {
    echo -e "\033[1;31m[WARNING]\033[0m $1"  # Red warning messages
}

progress() {
    echo -e "\033[1;34m[PROGRESS]\033[0m $1"  # Blue progress messages
}

is_valid_ipv4() {
    local ip="$1"
    local IFS=.
    local -a parts

    [[ "$ip" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || return 1
    read -r -a parts <<< "$ip"

    for part in "${parts[@]}"; do
        [[ "$part" =~ ^[0-9]+$ ]] || return 1
        ((part >= 0 && part <= 255)) || return 1
    done
}

normalize_ip_input() {
    local value="$1"
    value="${value#"${value%%[![:space:]]*}"}"
    value="${value%"${value##*[![:space:]]}"}"

    case "$value" in
        localhost|LOCALHOST)
            echo "127.0.0.1"
            ;;
        *)
            echo "$value"
            ;;
    esac
}

detect_wsl_host_ip() {
    local detected
    detected="$(ip route 2>/dev/null | awk '/^default / {print $3; exit}')"

    if is_valid_ipv4 "$detected"; then
        echo "$detected"
    else
        echo "$DEFAULT_FALLBACK_IP"
    fi
}

choose_px4_sim_ip() {
    local environment="$1"
    local auto_ip="$2"
    local default_ip="$3"
    local saved_ip="$4"
    local prompt_text="$5"
    local input
    local candidate=""

    auto_ip="$(normalize_ip_input "$auto_ip")"
    default_ip="$(normalize_ip_input "$default_ip")"
    saved_ip="$(normalize_ip_input "$saved_ip")"

    if ! is_valid_ipv4 "$auto_ip"; then
        warning "Auto-detected $environment IP '$auto_ip' is invalid; using $default_ip."
        auto_ip="$default_ip"
    fi

    if [ "$RESET_IP_MODE" = true ]; then
        info "Saved IP reset requested; ignoring previous PX4_SIM_HOSTNAME."
        saved_ip=""
    elif [ -n "$saved_ip" ]; then
        if is_valid_ipv4 "$saved_ip"; then
            info "Previous configuration found: $saved_ip"
        else
            warning "Saved PX4_SIM_HOSTNAME '$saved_ip' is invalid and will be ignored."
            saved_ip=""
        fi
    fi

    echo "$prompt_text"
    echo "  Current/default: ${saved_ip:-$auto_ip}"
    echo "  Auto-detected:   $auto_ip"
    echo ""
    echo "Press Enter to use current/default, type 'a' for auto, 'r' to reset to default,"
    echo "or enter a valid IPv4 address."

    read -t 15 -r input
    input="$(normalize_ip_input "$input")"

    case "$input" in
        "")
            candidate="${saved_ip:-$auto_ip}"
            ;;
        [Aa])
            candidate="$auto_ip"
            ;;
        [Rr]|reset|RESET|default|DEFAULT)
            candidate="$default_ip"
            ;;
        *)
            candidate="$input"
            ;;
    esac

    if ! is_valid_ipv4 "$candidate"; then
        warning "Invalid IP '$candidate'. Falling back to $auto_ip."
        candidate="$auto_ip"
    fi

    if ! is_valid_ipv4 "$candidate"; then
        warning "Fallback IP '$candidate' is invalid. Using $DEFAULT_FALLBACK_IP."
        candidate="$DEFAULT_FALLBACK_IP"
    fi

    CHOSEN_PX4_SIM_IP="$candidate"
}

last_config_value() {
    local key="$1"
    if [ -f "$CONFIG_FILE" ]; then
        grep -E "^${key}=" "$CONFIG_FILE" | tail -n 1 | cut -d= -f2-
    fi
}

airframe_file_for_platform() {
    case "$1" in
        xplane_ehang184) echo "ROMFS/px4fmu_common/init.d-posix/airframes/5010_xplane_ehang184" ;;
        xplane_alia250) echo "ROMFS/px4fmu_common/init.d-posix/airframes/5020_xplane_alia250" ;;
        xplane_cessna172) echo "ROMFS/px4fmu_common/init.d-posix/airframes/5001_xplane_cessna172" ;;
        xplane_tb2) echo "ROMFS/px4fmu_common/init.d-posix/airframes/5002_xplane_tb2" ;;
        xplane_qtailsitter) echo "ROMFS/px4fmu_common/init.d-posix/airframes/5021_xplane_qtailsitter" ;;
        *) echo "" ;;
    esac
}

sys_autostart_for_platform() {
    case "$1" in
        xplane_ehang184) echo "5010" ;;
        xplane_alia250) echo "5020" ;;
        xplane_cessna172) echo "5001" ;;
        xplane_tb2) echo "5002" ;;
        xplane_qtailsitter) echo "5021" ;;
        *) echo "unknown" ;;
    esac
}

airframe_hash_for_platform() {
    local relative_path
    relative_path="$(airframe_file_for_platform "$1")"

    if [ -z "$relative_path" ] || [ ! -f "$CLONE_PATH/$relative_path" ]; then
        echo ""
        return
    fi

    if command -v sha256sum &> /dev/null; then
        sha256sum "$CLONE_PATH/$relative_path" | awk '{print $1}'
    else
        cksum "$CLONE_PATH/$relative_path" | awk '{print $1 "-" $2}'
    fi
}

reset_saved_sitl_parameters() {
    local rootfs="$CLONE_PATH/build/px4_sitl_default/rootfs"
    local removed=false

    for param_file in "$rootfs/parameters.bson" "$rootfs/parameters_backup.bson"; do
        if [ -f "$param_file" ]; then
            rm -f "$param_file"
            removed=true
        fi
    done

    if [ "$removed" = true ]; then
        success "Saved PX4 SITL parameters reset so airframe defaults reload."
    else
        info "No saved PX4 SITL parameter store found to reset."
    fi
}

reset_params_if_airframe_changed() {
    local platform="$1"
    local current_hash
    current_hash="$(airframe_hash_for_platform "$platform")"
    SELECTED_AIRFRAME_HASH=""

    if [ -z "$current_hash" ]; then
        warning "Could not fingerprint the PX4 airframe file for $platform."
        return
    fi

    SELECTED_AIRFRAME_HASH="$current_hash"

    if [ "$PREVIOUS_PLATFORM" != "$platform" ] || [ "$PREVIOUS_AIRFRAME_HASH" != "$current_hash" ]; then
        info "Selected airframe/defaults changed since the previous run."
        reset_saved_sitl_parameters
    else
        info "Selected airframe/defaults unchanged; keeping saved SITL parameters."
    fi
}

apply_guard_commits() {
    local base_ref="$1"
    local enabled="$2"
    local guard_branch="$3"
    local label="$4"
    local guard_ref="origin/$guard_branch"
    local guard_commits
    local guard_commit

    if [ "$enabled" != true ]; then
        return 0
    fi

    guard_commits="$(git rev-list --reverse "$base_ref..$guard_ref" 2>/dev/null || true)"

    if [ -z "$guard_commits" ]; then
        warning "No $label commits found on $guard_ref."
        return 1
    fi

    progress "Applying $label locally for validation..."
    for guard_commit in $guard_commits; do
        if ! git cherry-pick -x "$guard_commit"; then
            git cherry-pick --abort >/dev/null 2>&1 || true
            warning "Could not apply $label commit $guard_commit."
            return 1
        fi
    done

    success "Local validation branch includes $label: $(git rev-parse --short HEAD)."
}

sync_px4_repo_to_remote() {
    local clean_untracked="${1:-false}"
    local remote_ref="origin/$BRANCH_NAME"
    local local_branch="$BRANCH_NAME"
    local status_output
    local ahead_count
    local backup_branch

    cd "$CLONE_PATH" || return 1

    progress "Syncing PX4 fork with $remote_ref..."

    if ! git remote get-url origin >/dev/null 2>&1; then
        git remote add origin "$REPO_URL"
        info "Added origin remote: $REPO_URL"
    fi

    if ! git fetch origin "+$BRANCH_NAME:refs/remotes/origin/$BRANCH_NAME"; then
        warning "Could not fetch origin/$BRANCH_NAME."
        return 1
    fi

    if [ "$APPLY_EKF_GSF_GUARD" = true ]; then
        progress "Fetching EKF-GSF guard branch for local validation..."
        if ! git fetch origin "+$EKF_GSF_GUARD_BRANCH:refs/remotes/origin/$EKF_GSF_GUARD_BRANCH"; then
            warning "Could not fetch origin/$EKF_GSF_GUARD_BRANCH."
            return 1
        fi
        local_branch="${BRANCH_NAME}-validation"
    fi

    if [ "$APPLY_VTOL_HANDOFF_GUARD" = true ]; then
        progress "Fetching Standard VTOL handoff guard branch for local validation..."
        if ! git fetch origin "+$VTOL_HANDOFF_GUARD_BRANCH:refs/remotes/origin/$VTOL_HANDOFF_GUARD_BRANCH"; then
            warning "Could not fetch origin/$VTOL_HANDOFF_GUARD_BRANCH."
            return 1
        fi
        local_branch="${BRANCH_NAME}-validation"
    fi

    if ! git rev-parse --verify "$remote_ref" >/dev/null 2>&1; then
        warning "Remote branch $remote_ref was not found."
        return 1
    fi

    if [ "$APPLY_EKF_GSF_GUARD" = true ] && ! git rev-parse --verify "origin/$EKF_GSF_GUARD_BRANCH" >/dev/null 2>&1; then
        warning "Remote branch origin/$EKF_GSF_GUARD_BRANCH was not found."
        return 1
    fi

    if [ "$APPLY_VTOL_HANDOFF_GUARD" = true ] && ! git rev-parse --verify "origin/$VTOL_HANDOFF_GUARD_BRANCH" >/dev/null 2>&1; then
        warning "Remote branch origin/$VTOL_HANDOFF_GUARD_BRANCH was not found."
        return 1
    fi

    status_output="$(git status --porcelain --untracked-files=normal 2>/dev/null)"
    if [ -n "$status_output" ]; then
        info "Saving local uncommitted/untracked PX4 changes to git stash before sync."
        git stash push --include-untracked -m "px4xplane auto-backup before sync $(date -u +%Y%m%dT%H%M%SZ)" >/dev/null 2>&1 || true
    fi

    ahead_count="$(git rev-list --count "$remote_ref"..HEAD 2>/dev/null || echo 0)"
    if [[ "$ahead_count" =~ ^[0-9]+$ ]] && [ "$ahead_count" -gt 0 ]; then
        backup_branch="px4xplane-local-backup-$(date -u +%Y%m%dT%H%M%SZ)"
        git branch "$backup_branch" HEAD >/dev/null 2>&1 || true
        info "Local commits were ahead of remote; backup branch created: $backup_branch"
    fi

    if ! git checkout -B "$local_branch" "$remote_ref"; then
        warning "Could not switch local branch to $remote_ref."
        return 1
    fi

    if ! git reset --hard "$remote_ref"; then
        warning "Could not reset local branch to $remote_ref."
        return 1
    fi

    apply_guard_commits "$remote_ref" "$APPLY_EKF_GSF_GUARD" "$EKF_GSF_GUARD_BRANCH" "EKF-GSF yaw reset guard" || return 1
    apply_guard_commits "$remote_ref" "$APPLY_VTOL_HANDOFF_GUARD" "$VTOL_HANDOFF_GUARD_BRANCH" "Standard VTOL handoff guard" || return 1

    if [ "$clean_untracked" = true ]; then
        git clean -fd
    fi

    progress "Synchronizing submodules..."
    git submodule sync --recursive
    if git submodule update --init --recursive --force --checkout --quiet; then
        success "Submodules updated."
    else
        warning "Submodule update had issues. Continuing with the synced PX4 branch."
    fi

    if [ "$clean_untracked" = true ]; then
        git submodule foreach --recursive 'git reset --hard >/dev/null 2>&1; git clean -fd >/dev/null 2>&1' >/dev/null 2>&1 || true
    fi

    reset_saved_sitl_parameters
    success "PX4 fork is synced to $(git rev-parse --short HEAD) on $(git branch --show-current)."
}

update_launcher_script_if_needed() {
    local current_script_path
    local temp_script

    command -v curl >/dev/null 2>&1 || return 0

    current_script_path="$(readlink -f "$0" 2>/dev/null || echo "$0")"
    temp_script="$(mktemp)"

    if ! curl -fsSL "$SETUP_SCRIPT_URL" -o "$temp_script" 2>/dev/null; then
        rm -f "$temp_script"
        warning "Could not check for the latest px4xplane launcher script."
        return 0
    fi

    if ! cmp -s "$temp_script" "$current_script_path" 2>/dev/null; then
        info "Updating px4xplane launcher script from master."
        cp "$temp_script" "$SCRIPT_PATH"
        chmod +x "$SCRIPT_PATH"

        if [ "$current_script_path" != "$SCRIPT_PATH" ]; then
            cp "$temp_script" "$current_script_path" 2>/dev/null || true
            chmod +x "$current_script_path" 2>/dev/null || true
        fi

        rm -f "$temp_script"
        success "Launcher script updated: $SCRIPT_PATH"
        highlight "The px4xplane launcher was updated. Re-run the same command now."
        exit 0
    fi

    rm -f "$temp_script"
}

prompt_for_ekf_gsf_guard() {
    if [ "$APPLY_EKF_GSF_GUARD" = true ] || [ "$SKIP_EKF_GSF_GUARD" = true ]; then
        return
    fi

    echo ""
    highlight "EKF-GSF yaw-reset guard"
    echo "A separate PX4 PR fixes a fast-VTOL transition yaw-reset edge case:"
    echo "  https://github.com/PX4/PX4-Autopilot/pull/27533"
    echo ""
    echo "Apply that guard locally on top of the X-Plane SITL branch? (Recommended)"
    echo "Press Enter for yes (default) or type 'n' to run without it."
    read -t 10 -r EKF_GSF_GUARD_CHOICE

    if [[ -z "$EKF_GSF_GUARD_CHOICE" || ! "$EKF_GSF_GUARD_CHOICE" =~ ^[Nn]$ ]]; then
        APPLY_EKF_GSF_GUARD=true
        SYNC_MODE=true
        info "EKF-GSF guard will be applied locally for this run."
    else
        SKIP_EKF_GSF_GUARD=true
        info "Continuing without the temporary EKF-GSF guard."
    fi
}

prompt_for_vtol_handoff_guard() {
    if [ "$APPLY_VTOL_HANDOFF_GUARD" = true ] || [ "$SKIP_VTOL_HANDOFF_GUARD" = true ]; then
        return
    fi

    echo ""
    highlight "Standard VTOL transition handoff guard"
    echo "A separate PX4 PR keeps Standard VTOL pusher thrust and pitch setpoints"
    echo "continuous just after front transition:"
    echo "  https://github.com/PX4/PX4-Autopilot/pull/27601"
    echo ""
    echo "Apply that guard locally on top of the X-Plane SITL branch? (Recommended)"
    echo "Press Enter for yes (default) or type 'n' to run without it."
    read -t 10 -r VTOL_HANDOFF_GUARD_CHOICE

    if [[ -z "$VTOL_HANDOFF_GUARD_CHOICE" || ! "$VTOL_HANDOFF_GUARD_CHOICE" =~ ^[Nn]$ ]]; then
        APPLY_VTOL_HANDOFF_GUARD=true
        SYNC_MODE=true
        info "Standard VTOL handoff guard will be applied locally for this run."
    else
        SKIP_VTOL_HANDOFF_GUARD=true
        info "Continuing without the temporary Standard VTOL handoff guard."
    fi
}

if [ "$RESET_CONFIG_MODE" = true ] && [ -f "$CONFIG_FILE" ]; then
    rm -f "$CONFIG_FILE"
    success "Saved px4xplane CLI configuration reset: $CONFIG_FILE"
fi

if [ "$SYNC_MODE" = true ] || [ "$REPAIR_MODE" = true ]; then
    update_launcher_script_if_needed
fi

# === Trap for Cleanup ===
cleanup() {
    if [[ -n "$MAVLINK_ROUTER_PID" ]]; then
        highlight "Stopping MAVLink Router..."
        if kill -0 "$MAVLINK_ROUTER_PID" 2>/dev/null; then
            kill "$MAVLINK_ROUTER_PID" 2>/dev/null || true
        fi
    fi
}
trap cleanup EXIT

# === Introductory Information ===
clear
echo "╔════════════════════════════════════════════════════════════════════════════╗"
echo "║                    PX4 X-Plane SITL Setup Script                           ║"
echo "╚════════════════════════════════════════════════════════════════════════════╝"
echo ""
highlight "Author: Alireza Ghaderi (@alireza787b)"
highlight "GitHub: https://github.com/alireza787b/px4xplane"
highlight "Updated: June 2026"
echo ""
echo "────────────────────────────────────────────────────────────────────────────"
warning "⚠️  TESTING PHASE - TEMPORARY WORKAROUND ⚠️"
echo ""
echo "This script installs X-Plane SITL support from a FORKED PX4 repository."
echo "This is a temporary solution while the integration is being reviewed for"
echo "official inclusion in PX4/PX4-Autopilot."
echo ""
info "What happens after the PR merges?"
echo "  → You'll be able to use the official PX4 repository directly"
echo "  → This custom fork will no longer be needed"
echo "  → A simple 'git remote' change will migrate you to official PX4"
echo ""
info "Current Status:"
echo "  → Fork: github.com/alireza787b/PX4-Autopilot-Me"
echo "  → Branch: px4xplane-sitl"
echo "  → PX4 PR: https://github.com/PX4/PX4-Autopilot/pull/22493"
echo "  → EKF-GSF guard: prompted by default while PX4 PR #27533 is pending"
echo "  → VTOL handoff guard: prompted by default while PX4 PR #27601 is pending"
echo ""
echo "────────────────────────────────────────────────────────────────────────────"
echo ""
highlight "Requirements:"
echo "  ✓ X-Plane 11 or 12"
echo "  ✓ px4xplane bridge plugin (download from releases)"
echo "  ✓ Ubuntu/Debian Linux (or WSL on Windows)"
echo ""
highlight "Supported Aircraft:"
echo "  • Cessna 172 (fixed-wing trainer)"
echo "  • TB2 UAV (fixed-wing)"
echo "  • Ehang 184 (quadcopter airtaxi)"
echo "  • Alia-250 (eVTOL quadplane)"
echo "  • QuadTailsitter (small VTOL tailsitter)"
echo ""
echo "────────────────────────────────────────────────────────────────────────────"
highlight "Press Enter to continue (auto-continue in 10 seconds)..."
read -t 10 -r

# === Prerequisites Check ===
progress "Checking for required tools..."
if ! command -v git &> /dev/null; then
    warning "Git is not installed. Please install Git and run the script again."
    echo "  Ubuntu/Debian: sudo apt-get install git"
    exit 1
fi
success "Git found."

prompt_for_ekf_gsf_guard
prompt_for_vtol_handoff_guard

# === Simplified Process for Subsequent Runs ===
if [ -d "$CLONE_PATH/.git" ] && [ "$REPAIR_MODE" = false ]; then
    info "Detected existing installation at: $CLONE_PATH"

    # === Auto-Sync Detection ===
    cd "$CLONE_PATH" || exit

    if [ "$SYNC_MODE" = true ]; then
        if sync_px4_repo_to_remote true; then
            LOCAL_HASH="$(git rev-parse HEAD 2>/dev/null)"
            REMOTE_HASH="$LOCAL_HASH"
        else
            LOCAL_HASH=""
            REMOTE_HASH=""
        fi
    else
        # Prompt user if they want to check for updates
        highlight "Check for updates from remote repository?"
        echo "Press 's' to skip (fast start) or any other key to check (default: check in 5 seconds)"
        read -t 5 -n 1 -r SKIP_UPDATE_CHECK
        echo ""

        if [[ "$SKIP_UPDATE_CHECK" =~ ^[Ss]$ ]]; then
            warning "Skipped update check. Using current local version."
            info "To force a clean sync later, run: $0 --sync"
            LOCAL_HASH=""
            REMOTE_HASH=""
        else
            progress "Checking for updates from remote repository..."

            # Fetch latest changes silently (with error handling)
            if git fetch origin "$BRANCH_NAME" --quiet 2>/dev/null; then
                success "Remote repository checked."
            else
                warning "Could not fetch updates from remote. Continuing with local version..."
                warning "Check your internet connection or try again later."
            fi

            # Check if local branch differs from remote (with error handling)
            LOCAL_HASH=$(git rev-parse HEAD 2>/dev/null)
            REMOTE_HASH=$(git rev-parse origin/"$BRANCH_NAME" 2>/dev/null)
        fi
    fi

    if [ -n "$LOCAL_HASH" ] && [ -n "$REMOTE_HASH" ] && [ "$LOCAL_HASH" != "$REMOTE_HASH" ]; then
        warning "Updates available from remote repository!"
        echo ""
        echo "Local commit:  $LOCAL_HASH"
        echo "Remote commit: $REMOTE_HASH"
        echo ""
        highlight "Do you want to pull the latest updates? (Recommended)"
        echo "Press Enter to pull updates (default: yes) or type 'n' to skip"
        read -t 10 -r PULL_UPDATES

        if [[ -z "$PULL_UPDATES" || ! "$PULL_UPDATES" =~ ^[Nn]$ ]]; then
            if sync_px4_repo_to_remote false; then

                # Check if setup script itself was updated
                if [ -f "$CLONE_PATH/setup_px4_sitl.sh" ]; then
                    info "Checking if setup script was updated..."

                    # Compare script with current running script
                    CURRENT_SCRIPT_PATH="$(readlink -f "$0")"

                    if ! cmp -s "$CLONE_PATH/setup_px4_sitl.sh" "$CURRENT_SCRIPT_PATH" 2>/dev/null; then
                        warning "The setup script has been updated!"
                        info "Copying new version to: $SCRIPT_PATH"

                        if cp "$CLONE_PATH/setup_px4_sitl.sh" "$SCRIPT_PATH" 2>/dev/null; then
                            chmod +x "$SCRIPT_PATH"
                            success "Setup script updated successfully!"
                            echo ""
                            highlight "⚠️  The script has been updated. Please re-run the command."
                            info "Run: $0"
                            exit 0
                        else
                            warning "Could not update setup script. Continuing with current version..."
                        fi
                    fi
                fi

                success "Successfully updated to latest version!"
                highlight "Changes pulled. You may want to rebuild your airframe."
            else
                warning "Could not sync to remote branch. Continuing with local version..."
                info "You may need to run: $0 --sync"
            fi
        else
            info "Skipping updates. Using current local version."
        fi
    elif [ -n "$LOCAL_HASH" ] && [ -n "$REMOTE_HASH" ]; then
        success "Your installation is up-to-date!"
    else
        warning "Could not check for updates (local or remote hash missing)."
        info "Continuing with current installation..."
    fi

    highlight "Skipping initial setup. Proceeding to build selection..."
else
    # === Full Repair Mode (If Flag is Set or Configured) ===
    if [ "$REPAIR_MODE" = true ]; then
        warning "Repair mode enabled. Running full setup and pulling the latest code..."
    fi

    # === Clone the Repository ===
    if [ ! -d "$CLONE_PATH/.git" ]; then
        progress "Cloning the repository from $REPO_URL..."
        info "This may take a few minutes depending on your connection..."
        git clone --recursive "$REPO_URL" "$CLONE_PATH"
        success "Repository cloned successfully!"
    else
        info "Repository already exists at $CLONE_PATH."
    fi

    cd "$CLONE_PATH" || exit

    progress "Checking out branch: $BRANCH_NAME..."
    git checkout "$BRANCH_NAME"
    success "Switched to branch: $BRANCH_NAME"

    # === Add Upstream and Fetch Tags ===
    progress "Adding upstream repository and fetching tags..."

    # Check if upstream already exists
    if ! git remote | grep -q "^upstream$"; then
        git remote add upstream "$UPSTREAM_URL"
        info "Added upstream remote: $UPSTREAM_URL"
    else
        info "Upstream remote already configured."
    fi

    git fetch upstream --quiet
    git fetch upstream --tags --quiet
    success "Upstream repository configured."

    # If tag issue persists, create a missing tag manually
    if ! git tag | grep -q "v1.14.0-dev"; then
        info "Creating missing tag v1.14.0-dev..."
        git tag v1.14.0-dev
    fi

    # === Run the ubuntu.sh Script ===
    highlight "Installing PX4 dependencies..."
    warning "This step may prompt for sudo password and take several minutes."
    info "The setup script will install compilers, simulators, and other tools."
    echo ""

    if bash Tools/setup/ubuntu.sh; then
        success "Dependencies installed successfully!"
        info "For WSL users: It's recommended to restart WSL after first installation."
        echo "  Exit WSL, then restart it before running this script again."
    else
        warning "Dependency installation completed with warnings. This is usually okay."
    fi

    # === Fetch Latest Changes and Normalize Branch State ===
    if ! sync_px4_repo_to_remote true; then
        warning "Could not fully sync the PX4 fork. Continuing with the checked-out branch."
    fi
fi

# === Clean Build Options ===
highlight "Build Options"
echo "You can clean the build directory if you're experiencing build issues."
echo ""
info "Option 1: 'make clean' - Removes build artifacts (safe, fast)"
info "Option 2: 'make distclean' - Full reset including CMake cache (slower, thorough)"
warning "After pulling or copying new airframe parameter files, choose 'd'."
info "A normal 'make clean' can leave saved SITL parameters in place and mask new airframe defaults."
echo ""
echo "Press Enter to skip (default) or type:"
echo "  'c' for clean"
echo "  'd' for distclean"
echo ""
read -t 5 -r CLEAN_OPTION

cd "$CLONE_PATH" || exit

if [[ "$CLEAN_OPTION" =~ ^[Cc]$ ]]; then
    progress "Running 'make clean'..."
    make clean
    success "Build directory cleaned."
elif [[ "$CLEAN_OPTION" =~ ^[Dd]$ ]]; then
    progress "Running 'make distclean'..."
    make distclean
    success "Build directory fully reset."
    reset_saved_sitl_parameters
else
    info "Skipping clean. Using existing build cache."
fi

# === Network Setup for X-Plane ===
# Detect environment and configure IP address for X-Plane connection
if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
    # WSL Environment: X-Plane runs on Windows, PX4 on WSL
    highlight "WSL Environment Detected"
    echo "For X-Plane integration, PX4 needs to know your Windows IP address."
    echo ""

    # Try to auto-detect the Windows IP from WSL
    AUTO_DETECTED_IP="$(detect_wsl_host_ip)"
    SAVED_IP="$(last_config_value PX4_SIM_HOSTNAME)"

    echo "💡 Tip: Find your Windows IP by running 'ipconfig' in PowerShell."
    echo "   Look for 'Ethernet adapter vEthernet (WSL)' or similar."
    echo ""
    highlight "PX4/X-Plane network target"
    choose_px4_sim_ip "WSL host" "$AUTO_DETECTED_IP" "$AUTO_DETECTED_IP" "$SAVED_IP" "Choose the Windows host IPv4 address for X-Plane."
    PX4_SIM_HOSTNAME="$CHOSEN_PX4_SIM_IP"
    info "Using PX4_SIM_HOSTNAME: $PX4_SIM_HOSTNAME"
else
    # Native Linux: Both X-Plane and PX4 run on the same machine
    highlight "Native Linux Environment Detected"
    echo "Running both X-Plane and PX4 SITL on the same Linux machine."
    echo ""

    # Check if user had previous configuration
    SAVED_IP="$(last_config_value PX4_SIM_HOSTNAME)"
    AUTO_DETECTED_IP="127.0.0.1"

    info "Auto-detected IP: $AUTO_DETECTED_IP (localhost)"
    echo ""
    echo "💡 Note: For native Linux, use 127.0.0.1 (localhost) since both"
    echo "   X-Plane and PX4 SITL run on the same machine."
    echo ""
    echo "   If X-Plane runs on a different machine, enter that machine's IP address."
    echo ""
    highlight "PX4/X-Plane network target"
    choose_px4_sim_ip "native Linux" "$AUTO_DETECTED_IP" "$DEFAULT_FALLBACK_IP" "$SAVED_IP" "Choose the X-Plane host IPv4 address."
    PX4_SIM_HOSTNAME="$CHOSEN_PX4_SIM_IP"
    info "Using PX4_SIM_HOSTNAME: $PX4_SIM_HOSTNAME"
fi

# Ensure config directory exists and save the configuration
CONFIG_DIR=$(dirname "$CONFIG_FILE")
if [ ! -d "$CONFIG_DIR" ]; then
    mkdir -p "$CONFIG_DIR"
fi

if ! is_valid_ipv4 "$PX4_SIM_HOSTNAME"; then
    warning "PX4_SIM_HOSTNAME '$PX4_SIM_HOSTNAME' is invalid. Resetting to $DEFAULT_FALLBACK_IP."
    PX4_SIM_HOSTNAME="$DEFAULT_FALLBACK_IP"
fi

# Preserve the previous airframe fingerprint before rewriting the config file.
PREVIOUS_PLATFORM="$(last_config_value LAST_PLATFORM)"
PREVIOUS_AIRFRAME_HASH="$(last_config_value LAST_AIRFRAME_HASH)"

# Save the IP; the selected platform and airframe fingerprint are appended after selection.
echo "PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME" > "$CONFIG_FILE"
export PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME"
success "IP address configured: $PX4_SIM_HOSTNAME"

# === MAVLink Router Setup ===
if [ "$USE_MAVLINK_ROUTER" = true ]; then
    highlight "MAVLink Router Setup"
    echo "MAVLink Router enables communication between PX4 and X-Plane/GCS."
    echo ""

    MAVLINK_ROUTER_INSTALL_SCRIPT="$INSTALL_PATH/install_mavlink_router.sh"

    # Download the MAVLink Router installation script
    if [ ! -f "$MAVLINK_ROUTER_INSTALL_SCRIPT" ]; then
        progress "Downloading MAVLink Router installation script..."
        curl -sS -o "$MAVLINK_ROUTER_INSTALL_SCRIPT" "$MAVLINK_ROUTER_INSTALL_SCRIPT_URL"
        chmod +x "$MAVLINK_ROUTER_INSTALL_SCRIPT"
    fi

    # Run the MAVLink Router installation script
    progress "Installing MAVLink Router..."
    if bash "$MAVLINK_ROUTER_INSTALL_SCRIPT"; then
        success "MAVLink Router installed."
    else
        warning "MAVLink Router installation had issues. Continuing anyway..."
    fi

    # Replace specific IP placeholders in the MAVLink Router command
    MAVLINK_ROUTER_CMD="${MAVLINK_ROUTER_COMMAND//__PX4_SIM_HOSTNAME__/$PX4_SIM_HOSTNAME}"
    MAVLINK_ROUTER_CMD="${MAVLINK_ROUTER_CMD//__MAVLINK2REST_IP__/$MAVLINK2REST_IP}"

    progress "Starting MAVLink Router..."
    info "Command: $MAVLINK_ROUTER_CMD"
    bash -c "$MAVLINK_ROUTER_CMD" &
    MAVLINK_ROUTER_PID=$!

    sleep 1
    if kill -0 "$MAVLINK_ROUTER_PID" 2>/dev/null; then
        # Provide additional instructions
        success "MAVLink Router is running (PID: $MAVLINK_ROUTER_PID)"
        echo ""
        highlight "MAVLink Endpoints Available:"
        echo "  • $PX4_SIM_HOSTNAME:14540 (X-Plane connection)"
        echo "  • $PX4_SIM_HOSTNAME:14550 (Ground Control Station)"
        echo "  • $PX4_SIM_HOSTNAME:14569 (Additional endpoint)"
        echo "  • $MAVLINK2REST_IP:14569 (MAVLink2REST interface)"
        echo ""
        info "Configure these endpoints in QGroundControl or your GCS."
        sleep 2
    else
        warning "MAVLink Router exited immediately. Check the command above and rerun with --reset-ip if the IP is wrong."
        MAVLINK_ROUTER_PID=""
    fi
fi

# === Global Access Setup (Optional) ===
if ! command -v px4xplane &> /dev/null; then
    highlight "Global Command Setup"
    echo "Set up 'px4xplane' command for system-wide access?"
    echo "This allows you to run this script from any directory."
    echo ""
    echo "Press Enter to enable (default: yes) or type 'n' to skip."
    read -t 10 -r GLOBAL_SETUP

    if [[ -z "$GLOBAL_SETUP" || "$GLOBAL_SETUP" =~ ^[Yy]$ ]]; then
        if [ -d "$HOME/bin" ]; then
            ln -sf "$SCRIPT_PATH" "$HOME/bin/px4xplane"
            success "Global command set up in: $HOME/bin/px4xplane"
        elif [ -d "$HOME/.local/bin" ]; then
            ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/px4xplane"
            success "Global command set up in: $HOME/.local/bin/px4xplane"
        else
            mkdir -p "$HOME/.local/bin"
            ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/px4xplane"
            echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
            source "$HOME/.bashrc"
            success "Global command set up in: $HOME/.local/bin/px4xplane"
        fi
        chmod +x "$SCRIPT_PATH"
        info "You can now run 'px4xplane' from anywhere!"
    else
        info "Global access setup skipped."
    fi
fi

# === Platform Selection and Build ===
highlight "Platform Selection"
echo "Select the X-Plane airframe to build and launch:"
echo ""

select PLATFORM in "${PLATFORM_CHOICES[@]}" "Exit"; do
    if [[ "$PLATFORM" == "Exit" ]]; then
        info "Exiting without building..."
        exit 0
    elif [[ " ${PLATFORM_CHOICES[@]} " =~ " ${PLATFORM} " ]]; then
        highlight "Building: $PLATFORM"
        LAST_PLATFORM="$PLATFORM"
        info "PX4 target: $PLATFORM (SYS_AUTOSTART=$(sys_autostart_for_platform "$PLATFORM"))"
        info "Airframe file: $(airframe_file_for_platform "$PLATFORM")"
        warning "Confirm X-Plane is loaded with the matching aircraft/config before arming."

        cd "$CLONE_PATH" || exit
        reset_params_if_airframe_changed "$PLATFORM"
        CURRENT_AIRFRAME_HASH="$SELECTED_AIRFRAME_HASH"

        {
            echo "LAST_PLATFORM=$LAST_PLATFORM"
            if [ -n "$CURRENT_AIRFRAME_HASH" ]; then
                echo "LAST_AIRFRAME_HASH=$CURRENT_AIRFRAME_HASH"
            fi
        } >> "$CONFIG_FILE"

        progress "Running: make px4_sitl_default $PLATFORM"
        echo ""

        if make px4_sitl_default "$PLATFORM"; then
            success "Build completed successfully!"
        else
            warning "Build completed with errors. Check the output above."
        fi
        break
    else
        warning "Invalid selection. Please choose a valid platform or 'Exit'."
    fi
done

# === Final Instructions ===
echo ""
echo "╔════════════════════════════════════════════════════════════════════════════╗"
echo "║                         Setup Complete!                                    ║"
echo "╚════════════════════════════════════════════════════════════════════════════╝"
echo ""
success "PX4 SITL is ready for X-Plane simulation!"
echo ""
highlight "Next Steps:"
echo ""
echo "1. Launch X-Plane and load the matching aircraft:"
info "   Selected airframe: $PLATFORM"
echo ""
echo "2. Start the px4xplane bridge plugin in X-Plane:"
info "   Plugin menu → PX4 X-Plane Bridge → Enable"
echo ""
echo "3. Connect your Ground Control Station:"
info "   QGroundControl will auto-connect to UDP:14550"
echo ""
echo "4. Verify the connection in PX4 console:"
info "   Look for 'HIL_SENSOR' and 'HIL_GPS' messages"
echo ""
warning "Important Reminders:"
echo "  • Make sure the correct airframe is loaded in X-Plane"
echo "  • The aircraft model should match the PX4 airframe type"
echo "  • For WSL users, verify Windows Firewall allows TCP 4560 and the GCS UDP endpoints"
echo ""
highlight "Resources:"
echo "  • README: https://github.com/alireza787b/px4xplane#quick-start"
echo "  • Documentation: https://github.com/alireza787b/px4xplane/tree/master/docs"
echo "  • Troubleshooting: https://github.com/alireza787b/px4xplane/blob/master/docs/index.md#troubleshooting"
echo "  • Issues: https://github.com/alireza787b/px4xplane/issues"
echo ""
info "Installation Summary:"
echo "  ✓ Repository: $CLONE_PATH"
echo "  ✓ Branch: $BRANCH_NAME"
echo "  ✓ Airframe: $PLATFORM"
echo "  ✓ IP Address: ${PX4_SIM_HOSTNAME:-Not configured}"
echo ""
echo "────────────────────────────────────────────────────────────────────────────"
warning "Remember: This is a TESTING PHASE installation using a forked repository."
info "Once the PR merges, you can switch to the official PX4 repository."
echo "────────────────────────────────────────────────────────────────────────────"
echo ""
highlight "Happy Flying! 🚁✈️"
echo ""

# === End of Script ===
