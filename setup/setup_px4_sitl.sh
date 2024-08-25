#!/bin/bash

# === Configurable Variables ===
REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
BRANCH_NAME="px4xplane-sitl"
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME/PX4-Autopilot-Me"
DEFAULT_FALLBACK_IP="127.0.0.1"  # Fallback IP if no IP is detected or entered
SCRIPT_NAME="px4xplane"  # Name for the global script

# === Airframe Options for SITL ===
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2")

# === Functions ===
uninstall_px4xplane() {
    echo "Uninstalling px4xplane..."

    # Remove the global script
    if [ -f "/usr/local/bin/$SCRIPT_NAME" ]; then
        sudo rm "/usr/local/bin/$SCRIPT_NAME"
        echo "Removed global command from /usr/local/bin."
    elif [ -f "$HOME/bin/$SCRIPT_NAME" ]; then
        rm "$HOME/bin/$SCRIPT_NAME"
        echo "Removed global command from $HOME/bin."
    fi

    # Remove the installation directory
    if [ -d "$CLONE_PATH" ]; then
        rm -rf "$CLONE_PATH"
        echo "Removed installation directory: $CLONE_PATH."
    fi

    echo "px4xplane uninstalled successfully."
    exit 0
}

# === Check for Uninstall Flag ===
if [[ "$1" == "--uninstall" ]]; then
    # Ensure the configuration file exists to find the installation path
    if [ -f "$DEFAULT_CLONE_PATH/px4xplane_config" ]; then
        source "$DEFAULT_CLONE_PATH/px4xplane_config"
        CLONE_PATH="${INSTALL_PATH:-$DEFAULT_CLONE_PATH}"
    else
        CLONE_PATH="$DEFAULT_CLONE_PATH"
    fi
    uninstall_px4xplane
fi

# === Check for Custom Installation Directory Parameter ===
if [ -n "$1" ] && [[ "$1" != "--uninstall" ]]; then
    CLONE_PATH="$1/PX4-Autopilot-Me"
    CONFIG_FILE="$1/px4xplane_config"
else
    CLONE_PATH="$DEFAULT_CLONE_PATH"
    CONFIG_FILE="$DEFAULT_CLONE_PATH/px4xplane_config"
fi

# === Create the Directory if it Doesn't Exist ===
if [ ! -d "$CLONE_PATH" ]; then
    echo "Directory $CLONE_PATH does not exist. Creating it..."
    mkdir -p "$CLONE_PATH"
fi

# === Introductory Information ===
echo "----------------------------------------------------------"
echo "PX4 X-Plane SITL Version 2.0 Setup Script"
echo "Author: Alireza Ghaderi"
echo "GitHub Repo: alireza787b/px4xplane"
echo "LinkedIn: alireza787b"
echo "Date: August 2024"
echo "----------------------------------------------------------"
echo "This script helps you set up PX4 SITL with X-Plane integration."
echo "It will clone the repository, install dependencies, and build SITL."
echo "You will need to download the PX4 X-Plane plugin from the release"
echo "section of this repository: https://github.com/alireza787b/px4xplane"
echo "Please ensure to follow the README instructions and video tutorials."
echo "This is a temporary setup until the integration is merged officially with PX4."
echo "----------------------------------------------------------"
echo "Press Enter to start the process (default: continue in 5 seconds)..."
read -t 5 -r

# === Prerequisites Check ===
echo "Checking for required tools..."
if ! command -v git &> /dev/null; then
    echo "Git is not installed. Please install Git and run the script again."
    exit 1
fi

# === Clone the Repository ===
if [ ! -d "$CLONE_PATH/.git" ]; then
    echo "Cloning the repository from $REPO_URL into $CLONE_PATH..."
    git clone --recursive "$REPO_URL" "$CLONE_PATH"
else
    echo "Repository already cloned at $CLONE_PATH."
fi

cd "$CLONE_PATH" || exit
git checkout "$BRANCH_NAME"

# === Add Upstream and Fetch Tags ===
echo "Adding upstream repository and fetching tags..."
git remote add upstream "$UPSTREAM_URL"
git fetch upstream
git fetch upstream --tags

# If tag issue persists, create a missing tag manually
if ! git tag | grep -q "v1.14.0-dev"; then
    echo "Creating missing tag v1.14.0-dev..."
    git tag v1.14.0-dev
fi

# === Run the ubuntu.sh Script ===
echo "Running setup script to install dependencies..."
bash Tools/setup/ubuntu.sh
echo "Setup complete. For WSL users, it's recommended to exit WSL, restart it, and rerun the script."

# === Fetch Latest Changes ===
echo "Fetching the latest changes from the remote repository..."
git fetch --all
git pull origin "$BRANCH_NAME"

# === Initialize and Update Submodules ===
echo "Updating submodules..."
git submodule update --init --recursive

# === Clean Build Options ===
echo "Do you want to run 'make clean' to clean the build directory? Press Enter to skip (default) or type 'y' to clean: "
read -t 5 -r CLEAN_BUILD

echo "Do you want to run 'make distclean' to reset the build directory? Press Enter to skip (default) or type 'y' to reset: "
read -t 5 -r DISTCLEAN_BUILD

if [[ "$CLEAN_BUILD" =~ ^[Yy]$ ]]; then
    make clean
fi

if [[ "$DISTCLEAN_BUILD" =~ ^[Yy]$ ]]; then
    make distclean
fi

# === Network Setup for X-Plane (WSL Users) ===
if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
    # Try to auto-detect the Windows IP from WSL
    AUTO_DETECTED_IP=$(ip route | grep default | awk '{print $3}')
    
    if [ -f "$CONFIG_FILE" ]; then
        source "$CONFIG_FILE"
        echo "Detected previous configuration. Use stored Windows IP ($PX4_SIM_HOSTNAME) or enter a new one (press Enter to keep default: $PX4_SIM_HOSTNAME in 10 seconds):"
    else
        echo "Auto-detected Windows IP: $AUTO_DETECTED_IP"
        PX4_SIM_HOSTNAME="$AUTO_DETECTED_IP"
    fi
    
    echo "Tip: You can find your Windows IP by opening PowerShell and typing 'ipconfig'. Look for the IP address under the section titled 'Ethernet adapter vEthernet (WSL)' or something similar."
    echo "Please enter the IP address if you want to change the detected one (press Enter to accept detected IP or enter a new one):"
    
    read -t 10 -r NEW_IP
    if [ -n "$NEW_IP" ]; then
        PX4_SIM_HOSTNAME="$NEW_IP"
    fi
    
    # If no IP is set, default to the predefined fallback IP (localhost or any other)
    if [ -z "$PX4_SIM_HOSTNAME" ]; then
        echo "No IP detected. Falling back to predefined IP: $DEFAULT_FALLBACK_IP (this only works if X-Plane is also running on Linux)."
        PX4_SIM_HOSTNAME="$DEFAULT_FALLBACK_IP"
    fi
    
    # Ensure config directory exists and save the configuration
    CONFIG_DIR=$(dirname "$CONFIG_FILE")
    if [ ! -d "$CONFIG_DIR" ]; then
        mkdir -p "$CONFIG_DIR"
    fi
    
    # Save both the IP and the last selected platform in the config file
    echo "PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME" > "$CONFIG_FILE"
    echo "INSTALL_PATH=$CLONE_PATH" >> "$CONFIG_FILE"
    echo "LAST_PLATFORM=${LAST_PLATFORM:-}" >> "$CONFIG_FILE"
    export PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME"
    echo "Exported PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME"
fi

# === Platform Selection and Build ===
echo "Please select the platform to build:"
select PLATFORM in "${PLATFORM_CHOICES[@]}"; do
    if [[ " ${PLATFORM_CHOICES[@]} " =~ " ${PLATFORM} " ]]; then
        echo "You have selected $PLATFORM. Building..."
        LAST_PLATFORM="$PLATFORM"
        echo "LAST_PLATFORM=$LAST_PLATFORM" >> "$CONFIG_FILE"
        make px4_sitl_default "$PLATFORM"
        break
    else
        echo "Invalid selection. Please choose a valid platform."
    fi
done

echo "Make sure that the selected airframe is defined and selected inside X-Plane's menu and loaded."
echo "Follow the video tutorials and instructions for more guidance."

# === Final Completion Message ===
echo "Script completed. Summary:"
echo "- Repository cloned: $CLONE_PATH"
echo "- Dependencies installed"
echo "- Submodules updated"
echo "- Platform built: $PLATFORM"
echo "You can now proceed with running the SITL simulation."

# === Install Script Globally ===
echo "Do you want to make the 'px4xplane' command globally available? (y/n)"
read -t 5 -r INSTALL_GLOBAL

if [[ "$INSTALL_GLOBAL" =~ ^[Yy]$ ]]; then
    echo "Installing script globally..."

    # Determine the destination for the global script
    if [ -d "/usr/local/bin" ]; then
        INSTALL_DIR="/usr/local/bin"
    elif [ -d "$HOME/bin" ]; then
        INSTALL_DIR="$HOME/bin"
    else
        echo "Could not find a suitable directory in PATH for installation."
        exit 1
    fi

    # Copy or symlink the script to the global directory
    cp "$0" "$INSTALL_DIR/$SCRIPT_NAME"
    chmod +x "$INSTALL_DIR/$SCRIPT_NAME"

    echo "'px4xplane' is now globally available. You can run it from anywhere."
else
    echo "Skipping global installation."
fi

# === End of Script ===
