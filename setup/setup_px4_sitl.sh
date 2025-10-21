#!/bin/bash

# === PX4 X-Plane SITL Setup Script ===
#
# Author: Alireza Ghaderi
# LinkedIn: alireza787b
# Date: October 2024
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
# - Fork Branch: px4xplane-sitl (synced with feature/xplane-sitl-integration)
# - Target: Official PX4 repository (pending PR approval)
# - Progress: https://github.com/alireza787b/PX4-Autopilot-Me/tree/feature/xplane-sitl-integration
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
# - xplane_qtailsitter: Quantix (quad tailsitter VTOL)
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
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME"
DEFAULT_CONFIG_FILE="$HOME/.px4sitl_config"
DEFAULT_FALLBACK_IP="127.0.0.1"
SCRIPT_NAME="px4xplane_script.sh"
MAVLINK2REST_IP="127.0.0.1"
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2" "xplane_qtailsitter")

# === MAVLink Router Configuration ===
USE_MAVLINK_ROUTER=true  # Set to true to enable MAVLink Router installation and setup
MAVLINK_ROUTER_INSTALL_SCRIPT_URL="https://raw.githubusercontent.com/alireza787b/mavlink-anywhere/main/install_mavlink_router.sh"
MAVLINK_ROUTER_COMMAND="mavlink-routerd -e IP_PLACEHOLDER:14540 -e IP_PLACEHOLDER:14550 -e IP_PLACEHOLDER:14569 -e MAVLINK2REST_IP_PLACEHOLDE:14569 0.0.0.0:14550"

# === Repair Mode (Configurable) ===
REPAIR_MODE=false  # Set to true if you always want full repair mode

# === Flags and Arguments ===
REPAIR_MODE_FLAG="--repair"
UNINSTALL_FLAG="--uninstall"

if [[ "$1" == "$UNINSTALL_FLAG" ]]; then
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

if [[ "$1" == "$REPAIR_MODE_FLAG" ]]; then
    REPAIR_MODE=true  # Override repair mode from command line
    shift
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

# === Trap for Cleanup ===
cleanup() {
    if [[ -n "$MAVLINK_ROUTER_PID" ]]; then
        highlight "Stopping MAVLink Router..."
        kill "$MAVLINK_ROUTER_PID"
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
highlight "Updated: October 2024"
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
echo "  → Branch: px4xplane-sitl (synced with feature/xplane-sitl-integration)"
echo "  → PR Status: In preparation for submission"
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
echo "  • Quantix (quad tailsitter VTOL)"
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

# === Simplified Process for Subsequent Runs ===
if [ -d "$CLONE_PATH" ] && [ -f "$CONFIG_FILE" ] && [ "$REPAIR_MODE" = false ]; then
    info "Detected existing installation at: $CLONE_PATH"

    # === Auto-Sync Detection ===
    cd "$CLONE_PATH" || exit

    # Prompt user if they want to check for updates
    highlight "Check for updates from remote repository?"
    echo "Press 's' to skip (fast start) or any other key to check (default: check in 5 seconds)"
    read -t 5 -n 1 -r SKIP_UPDATE_CHECK
    echo ""

    if [[ "$SKIP_UPDATE_CHECK" =~ ^[Ss]$ ]]; then
        warning "Skipped update check. Using current local version."
        info "To check for updates later, run: $0 --repair"
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

        # Check if local branch is behind remote (with error handling)
        LOCAL_HASH=$(git rev-parse HEAD 2>/dev/null)
        REMOTE_HASH=$(git rev-parse origin/"$BRANCH_NAME" 2>/dev/null)
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
            progress "Pulling latest updates..."

            # Stash any local changes (shouldn't be any, but be safe)
            if ! git diff-index --quiet HEAD -- 2>/dev/null; then
                info "Stashing local changes..."
                git stash push -m "Auto-stash before pulling updates" 2>/dev/null || true
            fi

            # Reset to remote branch (force sync) with error handling
            if git reset --hard origin/"$BRANCH_NAME" 2>/dev/null; then
                success "Repository updated to latest version!"

                # Update submodules
                progress "Updating submodules..."
                if git submodule update --init --recursive --quiet 2>/dev/null; then
                    success "Submodules updated."
                else
                    warning "Submodule update had issues. Continuing anyway..."
                fi

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
                warning "Could not reset to remote branch. Continuing with local version..."
                info "You may need to run with --repair flag: $0 --repair"
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

    # === Fetch Latest Changes ===
    progress "Fetching the latest changes from the remote repository..."
    git fetch --all --quiet

    # Reset to match remote (in case user had local changes)
    info "Syncing with remote branch..."
    git reset --hard origin/"$BRANCH_NAME"
    success "Local branch synced with remote."

    # === Initialize and Update Submodules ===
    progress "Updating submodules (this may take a moment)..."
    git submodule update --init --recursive --quiet
    success "Submodules updated."
fi

# === Clean Build Options ===
highlight "Build Options"
echo "You can clean the build directory if you're experiencing build issues."
echo ""
info "Option 1: 'make clean' - Removes build artifacts (safe, fast)"
info "Option 2: 'make distclean' - Full reset including CMake cache (slower, thorough)"
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
    AUTO_DETECTED_IP=$(ip route | grep default | awk '{print $3}')

    if [ -f "$CONFIG_FILE" ]; then
        source "$CONFIG_FILE"
        info "Previous configuration found: $PX4_SIM_HOSTNAME"
        echo "Press Enter to keep this IP (default) or enter a new one:"
        echo "  Auto-detected: $AUTO_DETECTED_IP"
        echo ""
    else
        info "Auto-detected Windows IP: $AUTO_DETECTED_IP"
        PX4_SIM_HOSTNAME="$AUTO_DETECTED_IP"
    fi

    echo "💡 Tip: Find your Windows IP by running 'ipconfig' in PowerShell."
    echo "   Look for 'Ethernet adapter vEthernet (WSL)' or similar."
    echo ""
    highlight "Enter IP address (or press Enter for auto-detected: $AUTO_DETECTED_IP):"

    read -t 15 -r NEW_IP
    if [ -n "$NEW_IP" ]; then
        PX4_SIM_HOSTNAME="$NEW_IP"
        info "Using manual IP: $PX4_SIM_HOSTNAME"
    else
        PX4_SIM_HOSTNAME="${PX4_SIM_HOSTNAME:-$AUTO_DETECTED_IP}"
        info "Using auto-detected IP: $PX4_SIM_HOSTNAME"
    fi

    # If no IP is set, default to localhost
    if [ -z "$PX4_SIM_HOSTNAME" ]; then
        warning "No IP detected. Falling back to: $DEFAULT_FALLBACK_IP"
        PX4_SIM_HOSTNAME="$DEFAULT_FALLBACK_IP"
    fi
else
    # Native Linux: Both X-Plane and PX4 run on the same machine
    highlight "Native Linux Environment Detected"
    echo "Running both X-Plane and PX4 SITL on the same Linux machine."
    echo ""

    # Check if user had previous configuration
    if [ -f "$CONFIG_FILE" ]; then
        source "$CONFIG_FILE"
        info "Previous configuration found: $PX4_SIM_HOSTNAME"
        AUTO_DETECTED_IP="${PX4_SIM_HOSTNAME:-127.0.0.1}"
    else
        AUTO_DETECTED_IP="127.0.0.1"
        PX4_SIM_HOSTNAME="$AUTO_DETECTED_IP"
    fi

    info "Auto-detected IP: $AUTO_DETECTED_IP (localhost)"
    echo ""
    echo "💡 Note: For native Linux, use 127.0.0.1 (localhost) since both"
    echo "   X-Plane and PX4 SITL run on the same machine."
    echo ""
    echo "   If X-Plane runs on a different machine, enter that machine's IP address."
    echo ""
    highlight "Press Enter to use $AUTO_DETECTED_IP or enter different IP:"

    read -t 15 -r NEW_IP
    if [ -n "$NEW_IP" ]; then
        PX4_SIM_HOSTNAME="$NEW_IP"
        info "Using custom IP: $PX4_SIM_HOSTNAME"
    else
        PX4_SIM_HOSTNAME="$AUTO_DETECTED_IP"
        info "Using localhost: $PX4_SIM_HOSTNAME"
    fi
fi

# Ensure config directory exists and save the configuration
CONFIG_DIR=$(dirname "$CONFIG_FILE")
if [ ! -d "$CONFIG_DIR" ]; then
    mkdir -p "$CONFIG_DIR"
fi

# Save both the IP and the last selected platform in the config file
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
    MAVLINK_ROUTER_CMD=$(echo "$MAVLINK_ROUTER_COMMAND" | sed "s/IP_PLACEHOLDER/$PX4_SIM_HOSTNAME/g" | sed "s/MAVLINK2REST_IP_PLACEHOLDE/$MAVLINK2REST_IP/g")

    progress "Starting MAVLink Router..."
    info "Command: $MAVLINK_ROUTER_CMD"
    $MAVLINK_ROUTER_CMD &
    MAVLINK_ROUTER_PID=$!

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
    sleep 3
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
        echo "LAST_PLATFORM=$LAST_PLATFORM" >> "$CONFIG_FILE"

        cd "$CLONE_PATH" || exit

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
echo "  • For WSL users, verify Windows Firewall allows UDP traffic"
echo ""
highlight "Resources:"
echo "  • Video Tutorials: https://github.com/alireza787b/px4xplane"
echo "  • Documentation: https://github.com/alireza787b/px4xplane/wiki"
echo "  • Troubleshooting: https://github.com/alireza787b/px4xplane/issues"
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
