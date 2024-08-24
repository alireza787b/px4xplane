#!/bin/bash

# === Configurable Variables ===
REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
BRANCH_NAME="px4xplane-sitl"
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME/PX4-Autopilot-Me"
DEFAULT_CONFIG_FILE="$HOME/.px4sitl_config"

# === Check for Custom Installation Directory Parameter ===
if [ -n "$1" ]; then
    CLONE_PATH="$1/PX4-Autopilot-Me"
    CONFIG_FILE="$1/.px4sitl_config"
else
    CLONE_PATH="$DEFAULT_CLONE_PATH"
    CONFIG_FILE="$DEFAULT_CONFIG_FILE"
fi

# === Create the Directory if it Doesn't Exist ===
if [ ! -d "$CLONE_PATH" ]; then
    echo "Directory $CLONE_PATH does not exist. Creating it..."
    mkdir -p "$CLONE_PATH"
fi

# === Introductory Information ===
echo "----------------------------------------------------------"
echo "PX4 X-Plane SITL Setup Script"
echo "Author: Alireza Ghaderi"
echo "GitHub Repo: alireza787b/px4xplane"
echo "LinkedIn: alireza787b"
echo "Date: August 2024"
echo "----------------------------------------------------------"
echo "This script helps you set up PX4 SITL with X-Plane integration."
echo "It will clone the repository, install dependencies, and build SITL."
echo "You will need to download the PX4 X-Plane plugin from the release"
echo "section of this repository: https://github.com/alireza787b/px4xplane"
echo "Please ensure to follow the README instructions."
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
        echo "Detected previous configuration. Use stored Windows IP ($PX4_SIM_HOSTNAME) or enter a new one (press Enter to keep default: $PX4_SIM_HOSTNAME in 5 seconds):"
    else
        echo "Auto-detected Windows IP: $AUTO_DETECTED_IP"
        PX4_SIM_HOSTNAME="$AUTO_DETECTED_IP"
    fi
    
    read -t 5 -r NEW_IP
    if [ -n "$NEW_IP" ]; then
        PX4_SIM_HOSTNAME="$NEW_IP"
    fi
    
    # If no IP is set, default to localhost
    if [ -z "$PX4_SIM_HOSTNAME" ]; then
        echo "No IP detected. Defaulting to localhost (this only works if X-Plane is also running on Linux)."
        PX4_SIM_HOSTNAME="localhost"
    fi
    
    # Ensure config directory exists and save the configuration
    CONFIG_DIR=$(dirname "$CONFIG_FILE")
    if [ ! -d "$CONFIG_DIR" ]; then
        mkdir -p "$CONFIG_DIR"
    fi
    
    echo "PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME" > "$CONFIG_FILE"
    export PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME"
    echo "Exported PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME"
fi

# === Platform Selection and Build ===
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2")
echo "Please select the platform to build (default: first option in 10 seconds):"
select PLATFORM in "${PLATFORM_CHOICES[@]}"; do
    if [[ " ${PLATFORM_CHOICES[@]} " =~ " ${PLATFORM} " ]]; then
        echo "You have selected $PLATFORM. Building..."
        echo "LAST_PLATFORM=$PLATFORM" > "$CONFIG_FILE"
        make px4_sitl_default "$PLATFORM"
        break
    else
        echo "Invalid selection. Please choose a valid platform."
    fi
done < <(sleep 10; echo 1)

# === Final Completion Message ===
echo "Script completed. Summary:"
echo "- Repository cloned: $CLONE_PATH"
echo "- Dependencies installed"
echo "- Submodules updated"
echo "- Platform built: $PLATFORM"
echo "You can now proceed with running the SITL simulation."

# === End of Script ===