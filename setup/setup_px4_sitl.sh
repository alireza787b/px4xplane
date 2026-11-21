#!/bin/bash

# === PX4 X-Plane SITL Setup Script ===
#
# Author: Alireza Ghaderi
# LinkedIn: alireza787b
# Updated: November 2026
# GitHub Repo: alireza787b/px4xplane
#
# === Purpose ===
# This script automates the setup of PX4 Software-In-The-Loop (SITL) integration
# with X-Plane. Official X-Plane SITL support is merged into PX4 main. The helper
# can also apply experimental TECS altitude reset PR #27670 for developer
# validation while that PX4-side PR remains open.
#
# Current Status:
# - Official PX4 integration: https://github.com/PX4/PX4-Autopilot/pull/22493
# - Validation mode: official PX4 main plus experimental TECS PR #27670 only
#
# === Functionality ===
# The script performs the following tasks:
# 1. Uses the maintained X-Plane airframes fork, or explicit official PX4 main.
# 2. Auto-syncs with remote updates (smart pull detection).
# 3. Sets up the necessary dependencies by running the provided setup script (`ubuntu.sh`).
# 4. Targets QGC directly, with an optional already-installed MAVLink Router.
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
# - **Saved tuning**: Isolated per airframe; explicit resets are backed up
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
# - `REPO_URL`: URL of the selected PX4 repository
# - `BRANCH_NAME`: Branch to checkout
# - `OFFICIAL_PX4_REPO_URL`: URL of the upstream PX4 repository
# - `DEFAULT_CLONE_PATH`: Default installation directory
# - `PLATFORM_CHOICES`: Available airframes for X-Plane
# - `--with-mavlink-router`: Opt in to an installed router for remote QGC
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
# For updates and more information, visit:
# - PX4 X-Plane Integration: https://github.com/alireza787b/px4xplane
# - Official PX4: https://github.com/PX4/PX4-Autopilot

# === Configurable Variables ===
OFFICIAL_PX4_REPO_URL="https://github.com/PX4/PX4-Autopilot.git"
OFFICIAL_PX4_BRANCH="main"
MAINTAINED_PX4_REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
MAINTAINED_PX4_BRANCH="px4xplane-airframes"
DEFAULT_PX4_BRANCH="${PX4XPLANE_DEFAULT_PX4_BRANCH:-$MAINTAINED_PX4_BRANCH}"
REPO_URL="${PX4XPLANE_PX4_REPO:-$MAINTAINED_PX4_REPO_URL}"
BRANCH_NAME="${PX4XPLANE_PX4_BRANCH:-$DEFAULT_PX4_BRANCH}"
PX4_REPO_EXPLICIT=false
PX4_BRANCH_EXPLICIT=false
[ -n "${PX4XPLANE_PX4_REPO:-}" ] && PX4_REPO_EXPLICIT=true
[ -n "${PX4XPLANE_PX4_BRANCH:-}" ] && PX4_BRANCH_EXPLICIT=true
PX4_REMOTE_NAME="${PX4XPLANE_PX4_REMOTE:-px4xplane}"
TECS_ALT_FRAME_GUARD_SOURCE_REF="${PX4XPLANE_TECS_ALT_FRAME_GUARD_REF:-refs/pull/27670/head}"
TECS_ALT_FRAME_GUARD_LOCAL_REF="pr-27670-tecs-alt-reference-guard"
DEFAULT_CLONE_PATH="$HOME"
MANAGED_CHECKOUT_NAME="${PX4XPLANE_CHECKOUT_NAME:-PX4-Autopilot}"
LEGACY_CHECKOUT_NAME="PX4-Autopilot-Me"
DEFAULT_CONFIG_FILE="$HOME/.px4sitl_config"
PX4_PATH_OVERRIDE="${PX4XPLANE_PX4_PATH:-}"
DEFAULT_FALLBACK_IP="127.0.0.1"
SCRIPT_NAME="px4xplane_script.sh"
COMMAND_NAME="px4xplane"
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2" "xplane_qtailsitter")
LAUNCHER_BASE_URL="${PX4XPLANE_LAUNCHER_BASE_URL:-}"
LAUNCHER_VERSION="2026-11-21.1"
LAUNCHER_API_VERSION=2

# === MAVLink Router Configuration ===
USE_MAVLINK_ROUTER=false
QGC_HOST=""
RESET_PARAMS_MODE=false

# === Repair Mode (Configurable) ===
REPAIR_MODE=false  # Set to true if you always want full repair mode

# === Flags and Arguments ===
REPAIR_MODE_FLAG="--repair"
UNINSTALL_FLAG="--uninstall"
SYNC_MODE_FLAG="--sync"
NO_SYNC_MODE_FLAG="--no-sync"
RESET_CONFIG_FLAG="--reset-config"
RESET_IP_FLAG="--reset-ip"
PX4_REPO_FLAG="--px4-repo"
PX4_BRANCH_FLAG="--px4-branch"
PX4_PATH_FLAG="--px4-path"
RESTORE_OFFICIAL_FLAG="--restore-official"
VALIDATION_MODE_FLAG="--validation"
EXACT_PR_MODE_FLAG="--exact-pr"
OFFICIAL_MODE_FLAG="--official"
EKF_GSF_GUARD_FLAG="--with-ekf-gsf-guard"
NO_EKF_GSF_GUARD_FLAG="--without-ekf-gsf-guard"
VTOL_HANDOFF_GUARD_FLAG="--with-vtol-handoff-guard"
NO_VTOL_HANDOFF_GUARD_FLAG="--without-vtol-handoff-guard"
TAILSITTER_FW_FRAME_GUARD_FLAG="--with-tailsitter-fw-frame-guard"
NO_TAILSITTER_FW_FRAME_GUARD_FLAG="--without-tailsitter-fw-frame-guard"
TECS_ALT_FRAME_GUARD_FLAG="--with-tecs-alt-guard"
NO_TECS_ALT_FRAME_GUARD_FLAG="--without-tecs-alt-guard"
HELP_FLAG="--help"

SYNC_MODE=true
RESET_CONFIG_MODE=false
RESET_IP_MODE=false
UNINSTALL_MODE=false
RESTORE_OFFICIAL_MODE=false
LAUNCH_MODE=maintained
SELECTED_MODE=""
TECS_DISABLED=false
AIRFRAME=""
LIST_AIRFRAMES=false
UPDATE_LAUNCHER=false
APPLY_TECS_ALT_FRAME_GUARD=false

print_usage() {
    cat <<EOF
Usage: $0 [options] [install_path]

Normal use: no arguments, or --maintained (default; maintained X-Plane airframes).
Official baseline: --official (actual PX4/PX4-Autopilot main; no pending patches).
Developer validation: --validation (experimental TECS PR #27670 only).
Custom repository/branch overrides require omitting explicit source modes.

Options:
  --airframe NAME Select one of the five canonical xplane_* airframes without a menu.
  --list-airframes
                  List supported targets and exit without setup or network actions.
  --maintained    Use the maintained public PX4 fork (default).
  --sync          Sync the clean selected PX4 checkout to the configured remote
                  revision and update submodules. Preserve local branches and tuning.
  --no-sync       Advanced/offline mode: do not sync the PX4 checkout before
                  launching; no launcher download or dependency installer runs.
                  Requires existing checkout/build dependencies. The build itself
                  may need cached inputs. Saved per-airframe tuning is preserved.
  --update-launcher
                  Explicitly update the installed script/runner pair and exit.
                  Never overwrites the invoking repository source file.
  --repair        Run the full dependency/setup path and sync the selected PX4 checkout.
  --reset-ip      Ignore the saved PX4_SIM_HOSTNAME and choose auto/default IP again.
  --reset-config  Remove saved px4xplane CLI config and re-prompt all remembered values.
  --reset-params  Back up and reset only the selected airframe's isolated SITL
                  parameter store. Does not delete another checkout's parameters.
  --qgc-host IP   QGC IPv4 address (default: the selected X-Plane host).
  --with-mavlink-router
                  Use an already installed mavlink-routerd for a remote GCS.
                  Optional; direct QGC UDP is the default. No router installer runs.
  --px4-repo URL  Use a different PX4 remote for this run.
  --px4-branch NAME
                  Use a different PX4 branch for this run. Also available via
                  PX4XPLANE_PX4_BRANCH for a custom source.
  --px4-path PATH Use an existing PX4-Autopilot checkout instead of the
                  helper-managed checkout. The script adds/uses a separate
                  px4xplane remote and does not rewrite your origin remote.
  --restore-official
                  Restore the selected PX4 checkout to official PX4 main,
                  update submodules, preserve branches/tuning, and exit.
  --validation    Use official PX4 main plus the still-open experimental
                  fixed-wing TECS altitude reset PR #27670 only.
  --official      Use official PX4 main without experimental patches or custom overrides.
  --exact-pr      Backward-compatible alias for --official.
  --with-ekf-gsf-guard
                  Retired: fails because PR #27533 was closed without merging.
  --without-ekf-gsf-guard
                  Backward-compatible no-op; the guard is retired.
  --with-vtol-handoff-guard
                  Retired: fails because PR #27601 is already merged.
  --without-vtol-handoff-guard
                  Backward-compatible no-op; the guard is retired.
  --with-tailsitter-fw-frame-guard
                  Retired: fails because PR #27793 is already merged.
  --without-tailsitter-fw-frame-guard
                  Backward-compatible no-op; the guard is retired.
  --with-tecs-alt-guard
                  Developer validation: cherry-pick experimental TECS
                  altitude reference reset PR #27670 before launching.
  --without-tecs-alt-guard
                  Do not apply the pending fixed-wing TECS altitude guard.
  --uninstall     Remove the global px4xplane command and saved launcher config.
                  Preserve PX4 checkouts, tuning, logs and backups.
  --help          Show this help.

Environment:
  PX4XPLANE_LAUNCHER_BASE_URL
                  Raw-content setup directory containing setup_px4_sitl.sh and
                  launch_sitl.py. Remembered at bootstrap; use a commit URL to pin.
  PX4XPLANE_SKIP_LAUNCHER_UPDATE=1
                  Disable explicit launcher updates (normal runs never self-update).
EOF
}

select_launch_mode() {
    if [ -n "$SELECTED_MODE" ] && [ "$SELECTED_MODE" != "$1" ]; then
        echo "Error: conflicting source modes: $SELECTED_MODE and $1" >&2
        exit 1
    fi
    SELECTED_MODE="$1"
    LAUNCH_MODE="$1"
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
        "$NO_SYNC_MODE_FLAG")
            SYNC_MODE=false
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
        --reset-params)
            RESET_PARAMS_MODE=true
            shift
            ;;
        --airframe)
            if [ -z "${2:-}" ] || [[ "$2" == --* ]]; then
                echo "Missing value for --airframe" >&2
                exit 1
            fi
            AIRFRAME="$2"
            shift 2
            ;;
        --list-airframes)
            LIST_AIRFRAMES=true
            shift
            ;;
        --update-launcher)
            UPDATE_LAUNCHER=true
            shift
            ;;
        --maintained)
            select_launch_mode maintained
            shift
            ;;
        --qgc-host)
            if [ -z "${2:-}" ] || [[ "$2" == --* ]]; then
                echo "Missing value for --qgc-host"
                print_usage
                exit 1
            fi
            QGC_HOST="$2"
            shift 2
            ;;
        --with-mavlink-router)
            USE_MAVLINK_ROUTER=true
            shift
            ;;
        "$PX4_REPO_FLAG")
            if [ -z "${2:-}" ] || [[ "$2" == --* ]]; then
                echo "Missing value for $PX4_REPO_FLAG"
                print_usage
                exit 1
            fi
            REPO_URL="$2"
            PX4_REPO_EXPLICIT=true
            shift 2
            ;;
        "$PX4_BRANCH_FLAG")
            if [ -z "${2:-}" ] || [[ "$2" == --* ]]; then
                echo "Missing value for $PX4_BRANCH_FLAG"
                print_usage
                exit 1
            fi
            BRANCH_NAME="$2"
            PX4_BRANCH_EXPLICIT=true
            shift 2
            ;;
        "$PX4_PATH_FLAG")
            if [ -z "${2:-}" ] || [[ "$2" == --* ]]; then
                echo "Missing value for $PX4_PATH_FLAG"
                print_usage
                exit 1
            fi
            PX4_PATH_OVERRIDE="$2"
            shift 2
            ;;
        "$RESTORE_OFFICIAL_FLAG")
            RESTORE_OFFICIAL_MODE=true
            shift
            ;;
        "$VALIDATION_MODE_FLAG")
            select_launch_mode validation
            shift
            ;;
        "$OFFICIAL_MODE_FLAG"|"$EXACT_PR_MODE_FLAG")
            select_launch_mode official
            shift
            ;;
        "$EKF_GSF_GUARD_FLAG")
            echo "Error: $1 is retired: PX4 PR #27533 was closed without merging. Use --maintained, --official or --validation for TECS PR #27670 only." >&2
            exit 1
            ;;
        "$VTOL_HANDOFF_GUARD_FLAG")
            echo "Error: $1 is retired: PX4 PR #27601 is already merged. Use --maintained, --official or --validation for TECS PR #27670 only." >&2
            exit 1
            ;;
        "$TAILSITTER_FW_FRAME_GUARD_FLAG")
            echo "Error: $1 is retired: PX4 PR #27793 is already merged. Use --maintained, --official or --validation for TECS PR #27670 only." >&2
            exit 1
            ;;
        "$NO_EKF_GSF_GUARD_FLAG"|"$NO_VTOL_HANDOFF_GUARD_FLAG"|"$NO_TAILSITTER_FW_FRAME_GUARD_FLAG")
            # Keep old commands working without restoring obsolete patches.
            shift
            ;;
        "$TECS_ALT_FRAME_GUARD_FLAG")
            select_launch_mode validation
            shift
            ;;
        "$NO_TECS_ALT_FRAME_GUARD_FLAG")
            TECS_DISABLED=true
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

if [ "$LIST_AIRFRAMES" = true ]; then
    printf '%s\n' "${PLATFORM_CHOICES[@]}"
    exit 0
fi
if [ -n "$AIRFRAME" ]; then
    case "$AIRFRAME" in
        xplane_ehang184|xplane_alia250|xplane_cessna172|xplane_tb2|xplane_qtailsitter) ;;
        *) echo "Error: unsupported airframe: $AIRFRAME (see --list-airframes)" >&2; exit 1 ;;
    esac
fi
if [ "$#" -gt 1 ] || { [ "$#" -gt 0 ] && [ -n "$PX4_PATH_OVERRIDE" ]; }; then
    echo "Error: choose one install path or --px4-path, not both." >&2
    exit 1
fi
if [ -n "$SELECTED_MODE" ] || [ "$RESTORE_OFFICIAL_MODE" = true ]; then
    if [ "$PX4_REPO_EXPLICIT" = true ] || [ "$PX4_BRANCH_EXPLICIT" = true ]; then
        echo "Error: source modes cannot be combined with custom PX4 repo/branch overrides." >&2
        exit 1
    fi
    case "$LAUNCH_MODE" in
        official|validation) REPO_URL="$OFFICIAL_PX4_REPO_URL"; BRANCH_NAME="$OFFICIAL_PX4_BRANCH" ;;
        maintained) REPO_URL="$MAINTAINED_PX4_REPO_URL"; BRANCH_NAME="$MAINTAINED_PX4_BRANCH" ;;
    esac
elif [ "$PX4_REPO_EXPLICIT" = true ] || [ "$PX4_BRANCH_EXPLICIT" = true ] || [ -n "${PX4XPLANE_DEFAULT_PX4_BRANCH:-}" ]; then
    LAUNCH_MODE=custom
fi
if [ "$LAUNCH_MODE" = validation ]; then
    if [ "$TECS_DISABLED" = true ]; then
        echo "Error: --validation requires the TECS guard; use --official instead." >&2
        exit 1
    fi
    APPLY_TECS_ALT_FRAME_GUARD=true
fi
if [ "$RESTORE_OFFICIAL_MODE" = true ] && { [ "$REPAIR_MODE" = true ] || [ -n "$AIRFRAME" ] || [ "$RESET_PARAMS_MODE" = true ] || { [ -n "$SELECTED_MODE" ] && [ "$SELECTED_MODE" != official ]; }; }; then
    echo "Error: --restore-official cannot be combined with repair, launch, reset-params or another source mode." >&2
    exit 1
fi
if [ "$SYNC_MODE" = false ] && { [ "$REPAIR_MODE" = true ] || [ "$RESTORE_OFFICIAL_MODE" = true ] || [ "$APPLY_TECS_ALT_FRAME_GUARD" = true ] || [ "$UPDATE_LAUNCHER" = true ]; }; then
    echo "Error: --no-sync cannot be combined with --repair, --restore-official, --validation or --update-launcher." >&2
    exit 1
fi
if [ "$UPDATE_LAUNCHER" = true ] && { [ "$REPAIR_MODE" = true ] || [ "$RESTORE_OFFICIAL_MODE" = true ] || [ -n "$AIRFRAME" ] || [ "$RESET_PARAMS_MODE" = true ] || [ "$LAUNCH_MODE" = validation ]; }; then
    echo "Error: --update-launcher is a standalone maintenance operation." >&2
    exit 1
fi
if [ "$UPDATE_LAUNCHER" = true ] && [ "${PX4XPLANE_SKIP_LAUNCHER_UPDATE:-0}" = 1 ]; then
    echo "Error: launcher updates disabled by PX4XPLANE_SKIP_LAUNCHER_UPDATE." >&2
    exit 1
fi

print_launcher_selection() {
    echo "PX4-XPlane launcher $LAUNCHER_VERSION"
    if [ "$RESTORE_OFFICIAL_MODE" = true ]; then
        echo "  Mode: restore-official"
        echo "  Repository: $OFFICIAL_PX4_REPO_URL"
        echo "  Branch: $OFFICIAL_PX4_BRANCH"
    else
        echo "  Mode: $LAUNCH_MODE"
        echo "  Repository: $REPO_URL"
        echo "  Branch: $BRANCH_NAME"
        if [ "$APPLY_TECS_ALT_FRAME_GUARD" = true ]; then
            echo "  TECS PR #27670: enabled (experimental)"
        fi
    fi
    echo "  Checkout: ${CLONE_PATH:-${PX4_PATH_OVERRIDE:-not selected}}"
}

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

    # Checkouts, per-airframe state and backups may contain the user's work.
    echo "PX4 checkouts, saved tuning and parameter backups are preserved."
    if [ -e "$DEFAULT_CONFIG_FILE" ]; then
        rm "$DEFAULT_CONFIG_FILE"
        echo "Removed .px4sitl_config file."
    fi

    echo "Uninstallation complete."
    exit 0
fi

# === Check for Custom Installation Directory / PX4 Checkout Parameters ===
USING_EXTERNAL_PX4_PATH=false
AUTO_DETECTED_PX4_PATH=""

last_config_value() {
    local key="$1"
    local file="${2:-$CONFIG_FILE}"
    if [ -f "$file" ]; then
        grep -E "^${key}=" "$file" | tail -n 1 | cut -d= -f2-
    fi
}

CONFIG_FILE="$DEFAULT_CONFIG_FILE"
if [ "$RESET_CONFIG_MODE" = false ]; then
    if [ -z "$PX4_PATH_OVERRIDE" ] && [ "$#" -eq 0 ]; then
        PX4_PATH_OVERRIDE="$(last_config_value PX4_PATH)"
    fi
    LAUNCHER_BASE_URL="${LAUNCHER_BASE_URL:-$(last_config_value LAUNCHER_BASE_URL)}"
fi
LAUNCHER_BASE_URL="${LAUNCHER_BASE_URL:-https://raw.githubusercontent.com/alireza787b/px4xplane/master/setup}"
LAUNCHER_BASE_URL="${LAUNCHER_BASE_URL%/}"
SETUP_SCRIPT_URL="$LAUNCHER_BASE_URL/setup_px4_sitl.sh"
SETUP_RUNNER_URL="$LAUNCHER_BASE_URL/launch_sitl.py"

if [ -n "$PX4_PATH_OVERRIDE" ]; then
    CLONE_PATH="${PX4_PATH_OVERRIDE/#\~/$HOME}"
    INSTALL_PATH="$(dirname "$CLONE_PATH")"
    USING_EXTERNAL_PX4_PATH=true
elif [ "$#" -gt 0 ]; then
    INSTALL_PATH="$1"
    CLONE_PATH="$INSTALL_PATH/$MANAGED_CHECKOUT_NAME"
else
    INSTALL_PATH="$DEFAULT_CLONE_PATH"
    CLONE_PATH="$INSTALL_PATH/$MANAGED_CHECKOUT_NAME"
    CONFIG_FILE="$DEFAULT_CONFIG_FILE"

    # Reuse the legacy helper-managed fork checkout if an older installation
    # already has one. The remote is normalized during sync.
    if [ ! -e "$CLONE_PATH/.git" ] && [ -e "$HOME/$LEGACY_CHECKOUT_NAME/.git" ]; then
        AUTO_DETECTED_PX4_PATH="$HOME/$LEGACY_CHECKOUT_NAME"
        CLONE_PATH="$AUTO_DETECTED_PX4_PATH"
        USING_EXTERNAL_PX4_PATH=true
    fi
fi
CLONE_PATH="$(readlink -m -- "$CLONE_PATH")" || exit 1
INSTALL_PATH="$(dirname "$CLONE_PATH")"
if [ "$USING_EXTERNAL_PX4_PATH" = true ] && [ "$UPDATE_LAUNCHER" = false ]; then
    if [ "$(git -C "$CLONE_PATH" rev-parse --show-toplevel 2>/dev/null)" != "$CLONE_PATH" ] || \
        [ ! -f "$CLONE_PATH/ROMFS/px4fmu_common/init.d-posix/rcS" ]; then
        echo "Error: --px4-path must select the root of an existing PX4 checkout: $CLONE_PATH" >&2
        exit 1
    fi
fi

if [ "$USING_EXTERNAL_PX4_PATH" = true ] && [ "$PX4_REMOTE_NAME" = "origin" ]; then
    PX4_REMOTE_NAME="px4xplane"
fi

SOURCE_SCRIPT_DIR="$(dirname -- "$(readlink -f -- "$0")")"
SCRIPT_PATH="$HOME/.local/share/px4xplane/launcher/setup_px4_sitl.sh"
RUNNER_PATH="$HOME/.local/share/px4xplane/launcher/launch_sitl.py"


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

global_command_paths() {
    echo "$HOME/bin/$COMMAND_NAME"
    echo "$HOME/.local/bin/$COMMAND_NAME"
}

refresh_global_command() {
    local target
    local current_target
    local seen=":"

    while IFS= read -r target; do
        [ -n "$target" ] || continue
        case "$seen" in
            *":$target:"*) continue ;;
        esac
        seen="${seen}${target}:"

        if [ -e "$target" ] || [ -L "$target" ]; then
            mkdir -p "$(dirname "$target")" 2>/dev/null || true
            current_target="$(readlink -f "$target" 2>/dev/null || true)"

            if [ "$current_target" != "$SCRIPT_PATH" ]; then
                ln -sf "$SCRIPT_PATH" "$target" 2>/dev/null || cp "$SCRIPT_PATH" "$target" 2>/dev/null || true
            fi

            chmod +x "$target" 2>/dev/null || true
        fi
    done < <(global_command_paths)
}

is_valid_ipv4() {
    local ip="$1"
    local IFS=.
    local -a parts

    [[ "$ip" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || return 1
    read -r -a parts <<< "$ip"

    for part in "${parts[@]}"; do
        [[ "$part" =~ ^[0-9]+$ ]] || return 1
        ((10#$part >= 0 && 10#$part <= 255)) || return 1
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
    local networking_mode="${1:-unknown}"
    local detected

    if [ "$networking_mode" = "mirrored" ]; then
        echo "$DEFAULT_FALLBACK_IP"
        return 0
    fi

    detected="$(ip route 2>/dev/null | awk '/^default / {print $3; exit}')"

    if is_valid_ipv4 "$detected"; then
        echo "$detected"
    else
        echo "$DEFAULT_FALLBACK_IP"
    fi
}

detect_wsl_networking_mode() {
    local mode=""

    if command -v wslinfo >/dev/null 2>&1; then
        mode="$(wslinfo --networking-mode 2>/dev/null | head -n 1 | tr -d '\r' | tr '[:upper:]' '[:lower:]')"
    fi

    case "$mode" in
        mirrored|nat|virtioproxy)
            echo "$mode"
            ;;
        *)
            echo "unknown"
            ;;
    esac
}

choose_px4_sim_ip() {
    local environment="$1"
    local auto_ip="$2"
    local default_ip="$3"
    local saved_ip="$4"
    local saved_source="$5"
    local prompt_text="$6"
    local input
    local candidate=""
    local candidate_source="auto"
    local preferred_ip=""
    local preferred_source="auto"

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
        saved_source=""
    elif [ -n "$saved_ip" ]; then
        if is_valid_ipv4 "$saved_ip"; then
            if [ "$saved_source" = "manual" ]; then
                preferred_ip="$saved_ip"
                preferred_source="manual"
                info "Previous manual PX4_SIM_HOSTNAME found: $saved_ip"
            elif [ "$saved_ip" = "$auto_ip" ]; then
                preferred_ip="$saved_ip"
                preferred_source="auto"
            else
                warning "Saved auto IP '$saved_ip' no longer matches the current $environment IP '$auto_ip'; using current detection."
            fi
        else
            warning "Saved PX4_SIM_HOSTNAME '$saved_ip' is invalid and will be ignored."
            saved_ip=""
            saved_source=""
        fi
    fi

    if [ -z "$preferred_ip" ]; then
        preferred_ip="$auto_ip"
        preferred_source="auto"
    fi

    echo "$prompt_text"
    echo "  Current/default: $preferred_ip ($preferred_source)"
    echo "  Auto-detected:   $auto_ip"
    echo ""
    echo "Press Enter to use current/default, type 'a' for auto, 'r' to reset to default,"
    echo "or enter a valid IPv4 address."

    read -t 15 -r input
    input="$(normalize_ip_input "$input")"

    case "$input" in
        "")
            candidate="$preferred_ip"
            candidate_source="$preferred_source"
            ;;
        [Aa])
            candidate="$auto_ip"
            candidate_source="auto"
            ;;
        [Rr]|reset|RESET|default|DEFAULT)
            candidate="$default_ip"
            candidate_source="auto"
            ;;
        *)
            candidate="$input"
            candidate_source="manual"
            ;;
    esac

    if ! is_valid_ipv4 "$candidate"; then
        warning "Invalid IP '$candidate'. Falling back to $auto_ip."
        candidate="$auto_ip"
        candidate_source="auto"
    fi

    if ! is_valid_ipv4 "$candidate"; then
        warning "Fallback IP '$candidate' is invalid. Using $DEFAULT_FALLBACK_IP."
        candidate="$DEFAULT_FALLBACK_IP"
        candidate_source="auto"
    fi

    CHOSEN_PX4_SIM_IP="$candidate"
    CHOSEN_PX4_SIM_IP_SOURCE="$candidate_source"
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

require_clean_px4_checkout() {
    local status_output
    status_output="$(git status --porcelain --untracked-files=normal)" || return 1
    if [ -n "$status_output" ]; then
        warning "PX4 has local changes or mismatched submodules. Nothing was discarded."
        printf '%s\n' "$status_output"
        info "Commit/preserve your changes first, or deliberately use --no-sync."
        return 1
    fi
    # Root status does not always expose untracked files in nested submodules.
    git submodule foreach --quiet --recursive \
        'status=$(git status --porcelain --untracked-files=normal) && test -z "$status" && test "$(git rev-parse HEAD)" = "$sha1"' || {
        warning "A PX4 submodule has local changes; sync stopped without cleaning."
        return 1
    }
}

require_no_running_px4() {
    local status=0
    pgrep -x px4 >/dev/null 2>&1 || status=$?
    if [ "$status" -ne 1 ]; then
        warning "PX4 may already be running (or process detection failed). Stop it before setup/build."
        return 1
    fi
}

apply_guard_commits() {
    local base_ref="$1"
    local enabled="$2"
    local guard_local_ref="$3"
    local label="$4"
    local guard_ref="$PX4_REMOTE_NAME/$guard_local_ref"
    local merge_base
    local guard_commits
    local guard_commit

    if [ "$enabled" != true ]; then
        return 0
    fi

    merge_base="$(git merge-base "$base_ref" "$guard_ref" 2>/dev/null || true)"
    if [ -z "$merge_base" ]; then
        warning "Could not find a merge base for $label against $base_ref."
        return 1
    fi

    guard_commits="$(git rev-list --reverse --no-merges "$merge_base..$guard_ref" 2>/dev/null || true)"

    if [ -z "$guard_commits" ]; then
        warning "No $label commits found on $guard_ref."
        return 1
    fi

    progress "Applying $label locally for validation..."
    for guard_commit in $guard_commits; do
        if ! git cherry-pick -x "$guard_commit"; then
            if [ -z "$(git diff --name-only --diff-filter=U)" ] && git diff --quiet && git diff --cached --quiet; then
                git cherry-pick --skip || return 1
                info "$label commit $guard_commit was already present; skipped."
            else
                git cherry-pick --abort >/dev/null 2>&1 || true
                warning "Could not apply $label commit $guard_commit."
                return 1
            fi
        fi
    done

    success "Local validation branch includes $label: $(git rev-parse --short HEAD)."
}

fetch_guard_ref() {
    local enabled="$1"
    local source_ref="$2"
    local local_ref="$3"
    local label="$4"

    if [ "$enabled" != true ]; then
        return 0
    fi

    progress "Fetching $label for local validation..."
    if ! git fetch "$PX4_REMOTE_NAME" "+$source_ref:refs/remotes/$PX4_REMOTE_NAME/$local_ref"; then
        warning "Could not fetch $label from $source_ref."
        return 1
    fi
}

sync_px4_repo_to_remote() {
    local remote_ref="$PX4_REMOTE_NAME/$BRANCH_NAME"
    local current_remote_url

    cd "$CLONE_PATH" || return 1
    require_no_running_px4 || return 1
    require_clean_px4_checkout || return 1

    progress "Syncing PX4 checkout with $remote_ref..."

    if ! git remote get-url "$PX4_REMOTE_NAME" >/dev/null 2>&1; then
        git remote add "$PX4_REMOTE_NAME" "$REPO_URL" || return 1
        info "Added $PX4_REMOTE_NAME remote: $REPO_URL"
    else
        current_remote_url="$(git remote get-url "$PX4_REMOTE_NAME" 2>/dev/null || true)"
    fi

    if [ "$PX4_REMOTE_NAME" = origin ] && [ -n "$current_remote_url" ] && [ "$current_remote_url" != "$REPO_URL" ]; then
        warning "Refusing to rewrite origin. Use the default px4xplane remote."
        return 1
    fi
    if [ "$PX4_REMOTE_NAME" != origin ] && git remote get-url "$PX4_REMOTE_NAME" >/dev/null 2>&1; then
        git remote set-url "$PX4_REMOTE_NAME" "$REPO_URL" || return 1
        info "Using $PX4_REMOTE_NAME remote: $REPO_URL"
    fi

    if ! git fetch --no-recurse-submodules "$PX4_REMOTE_NAME" "+$BRANCH_NAME:refs/remotes/$PX4_REMOTE_NAME/$BRANCH_NAME"; then
        warning "Could not fetch $PX4_REMOTE_NAME/$BRANCH_NAME."
        return 1
    fi

    fetch_guard_ref "$APPLY_TECS_ALT_FRAME_GUARD" "$TECS_ALT_FRAME_GUARD_SOURCE_REF" "$TECS_ALT_FRAME_GUARD_LOCAL_REF" "fixed-wing TECS altitude guard" || return 1

    if ! git rev-parse --verify "$remote_ref" >/dev/null 2>&1; then
        warning "Remote branch $remote_ref was not found."
        return 1
    fi

    if [ "$APPLY_TECS_ALT_FRAME_GUARD" = true ] && ! git rev-parse --verify "$PX4_REMOTE_NAME/$TECS_ALT_FRAME_GUARD_LOCAL_REF" >/dev/null 2>&1; then
        warning "Remote guard ref $PX4_REMOTE_NAME/$TECS_ALT_FRAME_GUARD_LOCAL_REF was not found."
        return 1
    fi

    # Keep even detached local work reachable before changing revisions.
    local previous_head
    previous_head="$(git rev-parse HEAD)" || return 1
    git update-ref "refs/px4xplane/previous/$previous_head" "$previous_head" || return 1
    # Refuse collisions with ignored files as well as normal untracked files.
    if ! git checkout --no-overwrite-ignore --detach "$remote_ref"; then
        warning "Could not switch local branch to $remote_ref."
        return 1
    fi

    apply_guard_commits "$remote_ref" "$APPLY_TECS_ALT_FRAME_GUARD" "$TECS_ALT_FRAME_GUARD_LOCAL_REF" "fixed-wing TECS altitude guard" || return 1

    progress "Synchronizing submodules..."
    git submodule sync --recursive || return 1
    if git submodule update --init --recursive --checkout --quiet; then
        success "Submodules updated."
    else
        warning "Submodule update failed; no build or launch will run."
        return 1
    fi

    success "PX4 checkout synced to $(git rev-parse --short HEAD) (detached; local branches/tuning preserved)."
}

restore_px4_repo_to_official() {
    if [ ! -e "$CLONE_PATH/.git" ]; then
        warning "No PX4 git checkout found at $CLONE_PATH."
        return 1
    fi

    REPO_URL="$OFFICIAL_PX4_REPO_URL"
    BRANCH_NAME="$OFFICIAL_PX4_BRANCH"
    PX4_REMOTE_NAME=px4xplane
    APPLY_TECS_ALT_FRAME_GUARD=false
    sync_px4_repo_to_remote || return 1
    success "PX4 checkout restored to official $OFFICIAL_PX4_BRANCH at $(git rev-parse --short HEAD)."
    info "X-Plane SITL airframes are available in official PX4 main."
}

ensure_sitl_runner() {
    local install_dir="$HOME/.local/share/px4xplane"
    local candidate pair_id pair_dir
    mkdir -p "$install_dir" || return 1
    candidate="$(mktemp -d "$install_dir/.launcher.XXXXXXXX")" || return 1
    if [ "$UPDATE_LAUNCHER" = true ]; then
        if ! curl --connect-timeout 10 --max-time 60 -fsSL "$SETUP_SCRIPT_URL" -o "$candidate/setup_px4_sitl.sh" || \
            ! curl --connect-timeout 10 --max-time 60 -fsSL "$SETUP_RUNNER_URL" -o "$candidate/launch_sitl.py"; then
            rm -rf -- "$candidate"
            warning "Launcher pair download failed; the installed pair was not changed."
            return 1
        fi
    else
        cp "$(readlink -f -- "$0")" "$candidate/setup_px4_sitl.sh" || { rm -rf -- "$candidate"; return 1; }
        if [ -f "$SOURCE_SCRIPT_DIR/launch_sitl.py" ]; then
            cp "$SOURCE_SCRIPT_DIR/launch_sitl.py" "$candidate/launch_sitl.py" || { rm -rf -- "$candidate"; return 1; }
        elif [ "$SYNC_MODE" = true ]; then
            if ! curl --connect-timeout 10 --max-time 60 -fsSL "$SETUP_RUNNER_URL" -o "$candidate/launch_sitl.py"; then
                rm -rf -- "$candidate"
                warning "Could not download the launcher companion from $SETUP_RUNNER_URL."
                return 1
            fi
        elif [ -f "$RUNNER_PATH" ]; then
            cp "$RUNNER_PATH" "$candidate/launch_sitl.py" || { rm -rf -- "$candidate"; return 1; }
        else
            rm -rf -- "$candidate"
            warning "Offline companion is missing. Bootstrap once online or run from the source setup directory."
            return 1
        fi
    fi
    if ! bash -n "$candidate/setup_px4_sitl.sh" || \
        ! grep -qx "LAUNCHER_API_VERSION=$LAUNCHER_API_VERSION" "$candidate/setup_px4_sitl.sh" || \
        [ "$(python3 "$candidate/launch_sitl.py" --api-version)" != "$LAUNCHER_API_VERSION" ]; then
        rm -rf -- "$candidate"
        warning "Launcher pair validation failed; the installed pair was not changed."
        return 1
    fi
    # Install a complete immutable pair, then atomically switch the command's directory.
    pair_id="$(sha256sum "$candidate/setup_px4_sitl.sh" "$candidate/launch_sitl.py" | awk '{printf "%s", $1}' | sha256sum | cut -c1-32)"
    pair_dir="$install_dir/launcher-$pair_id"
    chmod +x "$candidate/setup_px4_sitl.sh" || { rm -rf -- "$candidate"; return 1; }
    if [ -d "$pair_dir" ]; then
        if ! cmp -s "$candidate/setup_px4_sitl.sh" "$pair_dir/setup_px4_sitl.sh" || \
            ! cmp -s "$candidate/launch_sitl.py" "$pair_dir/launch_sitl.py"; then
            rm -rf -- "$candidate"
            warning "Existing launcher pair is inconsistent; inspect $pair_dir."
            return 1
        fi
        rm -rf -- "$candidate"
    else
        mv -T -- "$candidate" "$pair_dir" || { rm -rf -- "$candidate"; return 1; }
    fi
    candidate="$(mktemp -d "$install_dir/.link.XXXXXXXX")" || return 1
    if ! ln -s "$pair_dir" "$candidate/launcher" || ! mv -Tf -- "$candidate/launcher" "$install_dir/launcher"; then
        rm -rf -- "$candidate"
        return 1
    fi
    rmdir "$candidate"
    SCRIPT_PATH="$pair_dir/setup_px4_sitl.sh"
    RUNNER_PATH="$pair_dir/launch_sitl.py"
    printf 'LAUNCHER_BASE_URL=%s\n' "$LAUNCHER_BASE_URL" >> "$CONFIG_FILE" || return 1
}

update_launcher_script_if_needed() {
    if [ "$UPDATE_LAUNCHER" = false ]; then
        return 0
    fi
    ensure_sitl_runner || exit 1
    refresh_global_command
    success "Installed launcher pair from $LAUNCHER_BASE_URL. Re-run the launch command."
    exit 0
}

if [ "$RESET_CONFIG_MODE" = true ] && [ -f "$CONFIG_FILE" ]; then
    rm -f "$CONFIG_FILE"
    success "Saved px4xplane CLI configuration reset: $CONFIG_FILE"
fi

if ! command -v python3 >/dev/null 2>&1; then
    warning "Python 3 is required. Install it before running setup."
    exit 1
fi
update_launcher_script_if_needed
if ! command -v pgrep >/dev/null 2>&1; then
    warning "Process checks require pgrep. Install procps before running setup."
    exit 1
fi
require_no_running_px4 || exit 1

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

if [ "$RESTORE_OFFICIAL_MODE" = true ]; then
    print_launcher_selection
    restore_px4_repo_to_official
    exit $?
fi

# === Introductory Information ===
print_launcher_selection
echo ""
echo "Requirements:"
echo "  - X-Plane 11 or 12"
echo "  - px4xplane bridge plugin (download from releases)"
echo "  - Ubuntu/Debian Linux (or WSL on Windows)"
echo ""
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
if ! command -v python3 >/dev/null 2>&1; then
    warning "Python 3 is required. Install it with: sudo apt-get install python3"
    exit 1
fi
# === Simplified Process for Subsequent Runs ===
if [ -e "$CLONE_PATH/.git" ] && [ "$REPAIR_MODE" = false ]; then
    info "Detected existing installation at: $CLONE_PATH"

    # === Auto-Sync Detection ===
    cd "$CLONE_PATH" || exit

    if [ "$SYNC_MODE" = true ]; then
        if sync_px4_repo_to_remote true; then
            LOCAL_HASH="$(git rev-parse HEAD 2>/dev/null)"
            REMOTE_HASH="$LOCAL_HASH"
        else
            warning "Sync failed. Stopping; --no-sync is available for intentional local runs."
            exit 1
        fi
    else
        highlight "Offline/no-sync mode"
        LOCAL_HASH="$(git rev-parse HEAD 2>/dev/null || true)"
        REMOTE_HASH="$LOCAL_HASH"
    fi

    if [ "$SYNC_MODE" = false ]; then
        info "Using local HEAD ${LOCAL_HASH:-unknown}; remote not checked (--no-sync)."
    elif [ -n "$LOCAL_HASH" ] && [ -n "$REMOTE_HASH" ]; then
        success "Your installation is up-to-date!"
    else
        warning "Could not check for updates (local or remote hash missing)."
        exit 1
    fi

    highlight "Skipping initial setup. Proceeding to build selection..."
else
    # === Full Repair Mode (If Flag is Set or Configured) ===
    if [ "$REPAIR_MODE" = true ]; then
        warning "Repair mode enabled. Running full setup and pulling the latest code..."
    fi

    # === Clone the Repository ===
    if [ ! -e "$CLONE_PATH/.git" ]; then
        if [ "$SYNC_MODE" = false ]; then
            warning "No existing checkout for --no-sync. Run once with sync or pass --px4-path."
            exit 1
        fi
        progress "Cloning the repository from $REPO_URL..."
        info "This may take a few minutes depending on your connection..."
        mkdir -p "$INSTALL_PATH" || exit 1
        git clone --branch "$BRANCH_NAME" -- "$REPO_URL" "$CLONE_PATH" || exit 1
        success "Repository cloned successfully!"
    else
        info "Repository already exists at $CLONE_PATH."
    fi

    cd "$CLONE_PATH" || exit

    if [ "$SYNC_MODE" = true ]; then
        sync_px4_repo_to_remote || exit 1
    fi

    # === Run the ubuntu.sh Script ===
    highlight "Installing PX4 dependencies..."
    warning "This step may prompt for sudo password and take several minutes."
    info "Installing SITL dependencies without the NuttX or Gazebo toolchains."
    echo ""

    if bash Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools; then
        success "Dependencies installed successfully!"
        info "For WSL users: It's recommended to restart WSL after first installation."
        echo "  Exit WSL, then restart it before running this script again."
    else
        warning "Dependency installation failed. Resolve the setup output before launching."
        exit 1
    fi

fi

info "Build cache and saved tuning are preserved; no automatic clean/distclean."

# === Network Setup for X-Plane ===
# Detect environment and configure IP address for X-Plane connection
if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
    # WSL Environment: X-Plane runs on Windows, PX4 on WSL
    highlight "WSL Environment Detected"
    echo "For X-Plane integration, PX4 needs to know your Windows IP address."
    echo ""

    # Try to auto-detect the Windows IP from WSL
    WSL_NETWORKING_MODE="$(detect_wsl_networking_mode)"
    AUTO_DETECTED_IP="$(detect_wsl_host_ip "$WSL_NETWORKING_MODE")"
    SAVED_IP="$(last_config_value PX4_SIM_HOSTNAME)"
    SAVED_IP_SOURCE="$(last_config_value PX4_SIM_HOST_SOURCE)"

    info "WSL networking mode: $WSL_NETWORKING_MODE"
    if [ "$WSL_NETWORKING_MODE" = "mirrored" ]; then
        info "Mirrored networking uses Windows localhost (127.0.0.1)."
    else
        info "NAT/compatible networking uses the current Windows host gateway from the WSL route table."
    fi
    echo ""
    highlight "PX4/X-Plane network target"
    choose_px4_sim_ip "WSL host" "$AUTO_DETECTED_IP" "$AUTO_DETECTED_IP" "$SAVED_IP" "$SAVED_IP_SOURCE" "Choose the Windows host IPv4 address for X-Plane."
    PX4_SIM_HOSTNAME="$CHOSEN_PX4_SIM_IP"
    info "Using PX4_SIM_HOSTNAME: $PX4_SIM_HOSTNAME"
else
    # Native Linux: Both X-Plane and PX4 run on the same machine
    highlight "Native Linux Environment Detected"
    echo "Running both X-Plane and PX4 SITL on the same Linux machine."
    echo ""

    # Check if user had previous configuration
    SAVED_IP="$(last_config_value PX4_SIM_HOSTNAME)"
    SAVED_IP_SOURCE="$(last_config_value PX4_SIM_HOST_SOURCE)"
    AUTO_DETECTED_IP="127.0.0.1"

    info "Auto-detected IP: $AUTO_DETECTED_IP (localhost)"
    echo ""
    echo "💡 Note: For native Linux, use 127.0.0.1 (localhost) since both"
    echo "   X-Plane and PX4 SITL run on the same machine."
    echo ""
    echo "   If X-Plane runs on a different machine, enter that machine's IP address."
    echo ""
    highlight "PX4/X-Plane network target"
    choose_px4_sim_ip "native Linux" "$AUTO_DETECTED_IP" "$DEFAULT_FALLBACK_IP" "$SAVED_IP" "$SAVED_IP_SOURCE" "Choose the X-Plane host IPv4 address."
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

if [ -z "$QGC_HOST" ] && [ "$RESET_CONFIG_MODE" = false ]; then
    QGC_HOST="$(last_config_value QGC_HOST)"
fi
SAVED_QGC_HOST="$QGC_HOST"
QGC_HOST="${QGC_HOST:-$PX4_SIM_HOSTNAME}"
QGC_HOST="$(normalize_ip_input "$QGC_HOST")"
if ! is_valid_ipv4 "$QGC_HOST"; then
    warning "Invalid --qgc-host IPv4 address: $QGC_HOST"
    exit 1
fi
{
    echo "PX4_PATH=$CLONE_PATH"
    echo "LAUNCHER_BASE_URL=$LAUNCHER_BASE_URL"
    echo "PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME"
    echo "PX4_SIM_HOST_SOURCE=${CHOSEN_PX4_SIM_IP_SOURCE:-auto}"
    echo "QGC_HOST=$SAVED_QGC_HOST"
} > "$CONFIG_FILE"
export PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME"
success "IP address configured: $PX4_SIM_HOSTNAME"
QGC_STARTUP_HOST="$QGC_HOST"
info "X-Plane TCP endpoint: $PX4_SIM_HOSTNAME:4560"
info "QGC UDP destination: $QGC_HOST:14550 (reception not yet verified)"

# === MAVLink Router Setup ===
if [ "$USE_MAVLINK_ROUTER" = true ]; then
    highlight "MAVLink Router Setup"
    if ! command -v mavlink-routerd >/dev/null 2>&1; then
        warning "mavlink-routerd is not installed. Install it separately or omit --with-mavlink-router."
        exit 1
    fi
    if [[ "$QGC_HOST" == 127.* ]]; then
        warning "Use direct QGC for localhost. Router mode requires a remote --qgc-host to avoid a loop/port conflict."
        exit 1
    fi
    mavlink-routerd -e "$QGC_HOST:14550" 127.0.0.1:14550 &
    MAVLINK_ROUTER_PID=$!
    sleep 1
    if kill -0 "$MAVLINK_ROUTER_PID" 2>/dev/null; then
        QGC_STARTUP_HOST=127.0.0.1
        info "Optional router forwards local UDP 14550 to $QGC_HOST:14550."
    else
        warning "MAVLink Router failed to start; no PX4 launch performed."
        MAVLINK_ROUTER_PID=""
        exit 1
    fi
fi

ensure_sitl_runner || exit 1
refresh_global_command

# === Global Access Setup (Optional) ===
if ! command -v "$COMMAND_NAME" &> /dev/null; then
    highlight "Global Command Setup"
    echo "Set up '$COMMAND_NAME' command for system-wide access?"
    echo "This allows you to run this script from any directory."
    echo ""
    echo "Press Enter to enable (default: yes) or type 'n' to skip."
    read -t 10 -r GLOBAL_SETUP

    if [[ -z "$GLOBAL_SETUP" || "$GLOBAL_SETUP" =~ ^[Yy]$ ]]; then
        if [ -d "$HOME/bin" ]; then
            COMMAND_DIR="$HOME/bin"
            ln -sf "$SCRIPT_PATH" "$HOME/bin/$COMMAND_NAME"
            success "Global command set up in: $HOME/bin/$COMMAND_NAME"
        elif [ -d "$HOME/.local/bin" ]; then
            COMMAND_DIR="$HOME/.local/bin"
            ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/$COMMAND_NAME"
            success "Global command set up in: $HOME/.local/bin/$COMMAND_NAME"
        else
            COMMAND_DIR="$HOME/.local/bin"
            mkdir -p "$HOME/.local/bin"
            ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/$COMMAND_NAME"
            success "Global command set up in: $HOME/.local/bin/$COMMAND_NAME"
        fi
        chmod +x "$SCRIPT_PATH"
        refresh_global_command
        case ":$PATH:" in
            *":$COMMAND_DIR:"*) ;;
            *)
                printf 'export PATH="%s:$PATH"\n' "$COMMAND_DIR" >> "$HOME/.bashrc"
                export PATH="$COMMAND_DIR:$PATH"
                info "Open a new WSL/terminal session to refresh the parent shell's command path."
                ;;
        esac
        info "You can now run '$COMMAND_NAME' from anywhere!"
    else
        info "Global access setup skipped."
    fi
fi

# === Platform Selection and Build ===
PLATFORM="$AIRFRAME"
if [ -z "$PLATFORM" ]; then
    highlight "Platform Selection"
    echo "Select the X-Plane airframe to build and launch:"
    select PLATFORM in "${PLATFORM_CHOICES[@]}" "Exit"; do
        if [[ "$PLATFORM" == "Exit" ]]; then
            info "Exiting without building..."
            exit 0
        elif [ -n "$PLATFORM" ]; then
            break
        fi
        warning "Invalid selection. Please choose a valid platform or 'Exit'."
    done
fi
if [ -n "$PLATFORM" ]; then
        highlight "Building: $PLATFORM"
        require_no_running_px4 || exit 1
        LAST_PLATFORM="$PLATFORM"
        info "PX4 target: $PLATFORM (SYS_AUTOSTART=$(sys_autostart_for_platform "$PLATFORM"))"
        info "Airframe file: $(airframe_file_for_platform "$PLATFORM")"
        warning "Confirm X-Plane is loaded with the matching aircraft/config before arming."

        cd "$CLONE_PATH" || exit
        CURRENT_AIRFRAME_HASH="$(airframe_hash_for_platform "$PLATFORM")"

        {
            echo "LAST_PLATFORM=$LAST_PLATFORM"
            if [ -n "$CURRENT_AIRFRAME_HASH" ]; then
                echo "LAST_AIRFRAME_HASH=$CURRENT_AIRFRAME_HASH"
            fi
        } >> "$CONFIG_FILE"

        info "Source commit: $(git rev-parse HEAD)"
        info "Saved tuning: isolated by checkout/source and airframe; preserved unless --reset-params."
        progress "Building with existing cache: make -j2 px4_sitl_default"
        echo ""

        if ! make -j2 px4_sitl_default; then
            warning "Build failed. No PX4 launch performed."
            exit 1
        fi
        RUNNER_ARGS=(--checkout "$CLONE_PATH" --target "$PLATFORM" --host "$PX4_SIM_HOSTNAME"
            --qgc-host "$QGC_STARTUP_HOST" --source-label "$REPO_URL@$BRANCH_NAME:$APPLY_TECS_ALT_FRAME_GUARD")
        [ "$RESET_PARAMS_MODE" = false ] || RUNNER_ARGS+=(--reset-params)
        python3 "$RUNNER_PATH" "${RUNNER_ARGS[@]}"
        PX4_EXIT_CODE=$?
        info "PX4 exited with status $PX4_EXIT_CODE. Saved tuning and logs remain in the reported runtime directory."
        exit "$PX4_EXIT_CODE"
fi

warning "No airframe selected; no SITL launch performed."
exit 1
