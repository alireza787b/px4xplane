#!/bin/bash

# === Configurable Variables ===
REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
BRANCH_NAME="px4xplane-sitl"
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME/PX4-Autopilot-Me"
CONFIG_FILE="$HOME/.px4sitl_config"

# === Check for Custom Installation Directory Parameter ===
if [ -n "$1" ]; then
    CLONE_PATH="$1/PX4-Autopilot-Me"
else
    CLONE_PATH="$DEFAULT_CLONE_PATH"
fi

# === Create the Directory if it Doesn't Exist ===
if [ ! -d "$CLONE_PATH" ]; then
    echo "Directory $CLONE_PATH does not exist. Creating it..."
    mkdir -p "$CLONE_PATH"
fi

# === Initial Setup ===
echo "Welcome to the PX4 SITL Setup Script"
echo "This script will clone the repository, set up dependencies, and build the specified SITL platform."
echo "Make sure you have a stable internet connection."
echo "Installing to: $CLONE_PATH"
echo "Press Enter to continue or Ctrl+C to cancel."
read -r

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
echo "Do you want to run 'make clean' to clean the build directory? [y/N]: "
read -r CLEAN_BUILD

echo "Do you want to run 'make distclean' to reset the build directory? [y/N]: "
read -r DISTCLEAN_BUILD

if [[ "$CLEAN_BUILD" =~ ^[Yy]$ ]]; then
    make clean
fi

if [[ "$DISTCLEAN_BUILD" =~ ^[Yy]$ ]]; then
    make distclean
fi

# === Network Setup for X-Plane (WSL Users) ===
if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
    if [ -f "$CONFIG_FILE" ]; then
        source "$CONFIG_FILE"
        echo "Detected previous configuration. Use stored Windows IP ($PX4_SIM_HOSTNAME) or enter a new one:"
    else
        echo "Please enter the IP address of your Windows machine (WSL interface IP) for X-Plane:"
    fi
    read -r NEW_IP
    if [ -n "$NEW_IP" ]; then
        PX4_SIM_HOSTNAME="$NEW_IP"
        echo "PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME" > "$CONFIG_FILE"
    fi
    export PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME"
    echo "Exported PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME"
fi

# === Platform Selection and Build ===
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2")
echo "Please select the platform to build:"
select PLATFORM in "${PLATFORM_CHOICES[@]}"; do
    if [[ " ${PLATFORM_CHOICES[@]} " =~ " ${PLATFORM} " ]]; then
        echo "You have selected $PLATFORM. Building..."
        echo "LAST_PLATFORM=$PLATFORM" > "$CONFIG_FILE"
        make px4_sitl_default "$PLATFORM"
        break
    else
        echo "Invalid selection. Please choose a valid platform."
    fi
done

# === Final Completion Message ===
echo "Script completed. Summary:"
echo "- Repository cloned: $CLONE_PATH"
echo "- Dependencies installed"
echo "- Submodules updated"
echo "- Platform built: $PLATFORM"
echo "You can now proceed with running the SITL simulation."

# === End of Script ===
