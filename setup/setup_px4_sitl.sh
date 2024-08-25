#!/bin/bash

# === Configurable Variables ===
REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
BRANCH_NAME="px4xplane-sitl"
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME/testpx4"
DEFAULT_CONFIG_FILE="$HOME/.px4sitl_config"
DEFAULT_FALLBACK_IP="127.0.0.1"
SCRIPT_NAME="px4xplane_script.sh"
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2")
REPAIR_MODE_FLAG="--repair"

# === Flags and Arguments ===
REPAIR_MODE=false  # Default: Skip repair if not explicitly asked
if [[ "$1" == "$REPAIR_MODE_FLAG" ]]; then
    REPAIR_MODE=true
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

# === Introductory Information ===
highlight "----------------------------------------------------------"
highlight "PX4 X-Plane SITL Setup Script"
highlight "Author: Alireza Ghaderi"
highlight "GitHub Repo: alireza787b/px4xplane"
highlight "LinkedIn: alireza787b"
highlight "Date: August 2024"
highlight "----------------------------------------------------------"
highlight "This script helps you set up PX4 SITL with X-Plane integration."
highlight "You will need to download the PX4 X-Plane plugin from the release section of this repository: https://github.com/alireza787b/px4xplane"
highlight "This is a temporary setup until the integration is merged officially with PX4."
highlight "----------------------------------------------------------"
highlight "Press Enter to start the process (default: continue in 5 seconds)..."
read -t 5 -r

# === Prerequisites Check ===
highlight "Checking for required tools..."
if ! command -v git &> /dev/null; then
    echo "Git is not installed. Please install Git and run the script again."
    exit 1
fi

# === Simplified Process for Subsequent Runs ===
if [ -d "$CLONE_PATH" ] && [ -f "$CONFIG_FILE" ] && [ "$REPAIR_MODE" = false ]; then
    highlight "Detected existing configuration. Skipping setup and proceeding to build..."
else
    # === Full Repair Mode (If Flag is Set) ===
    if [ "$REPAIR_MODE" = true ]; then
        highlight "Repair mode enabled. Running full setup and pulling the latest code..."
    fi

    # === Clone the Repository ===
    if [ ! -d "$CLONE_PATH/.git" ]; then
        highlight "Cloning the repository from $REPO_URL into $CLONE_PATH..."
        git clone --recursive "$REPO_URL" "$CLONE_PATH"
    else
        echo "Repository already cloned at $CLONE_PATH."
    fi

    cd "$CLONE_PATH" || exit
    git checkout "$BRANCH_NAME"

    # === Add Upstream and Fetch Tags ===
    highlight "Adding upstream repository and fetching tags..."
    git remote add upstream "$UPSTREAM_URL"
    git fetch upstream
    git fetch upstream --tags

    # If tag issue persists, create a missing tag manually
    if ! git tag | grep -q "v1.14.0-dev"; then
        echo "Creating missing tag v1.14.0-dev..."
        git tag v1.14.0-dev
    fi

    # === Run the ubuntu.sh Script ===
    highlight "Running setup script to install dependencies..."
    bash Tools/setup/ubuntu.sh
    echo "Setup complete. For WSL users, it's recommended to exit WSL, restart it, and rerun the script."

    # === Fetch Latest Changes ===
    highlight "Fetching the latest changes from the remote repository..."
    git fetch --all
    git pull origin "$BRANCH_NAME"

    # === Initialize and Update Submodules ===
    highlight "Updating submodules..."
    git submodule update --init --recursive
fi

# === Clean Build Options ===
highlight "Do you want to run 'make clean' to clean the build directory? Press Enter to skip (default) or type 'y' to clean: "
read -t 5 -r CLEAN_BUILD

highlight "Do you want to run 'make distclean' to reset the build directory? Press Enter to skip (default) or type 'y' to reset: "
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
        highlight "Detected previous configuration. Use stored Windows IP ($PX4_SIM_HOSTNAME) or enter a new one (press Enter to keep default: $PX4_SIM_HOSTNAME in 10 seconds):"
    else
        highlight "Auto-detected Windows IP: $AUTO_DETECTED_IP"
        PX4_SIM_HOSTNAME="$AUTO_DETECTED_IP"
    fi
    
    echo "Tip: You can find your Windows IP by opening PowerShell and typing 'ipconfig'. Look for the IP address under the section titled 'Ethernet adapter vEthernet (WSL)' or something similar."
    highlight "Please enter the IP address if you want to change the detected one (press Enter to accept detected IP or enter a new one):"
    
    read -t 10 -r NEW_IP
    if [ -n "$NEW_IP" ]; then
        PX4_SIM_HOSTNAME="$NEW_IP"
    fi
    
    # If no IP is set, default to the predefined fallback IP (localhost or any other)
    if [ -z "$PX4_SIM_HOSTNAME" ]; then
        highlight "No IP detected. Falling back to predefined IP: $DEFAULT_FALLBACK_IP (this only works if X-Plane is also running on Linux)."
        PX4_SIM_HOSTNAME="$DEFAULT_FALLBACK_IP"
    fi
    
    # Ensure config directory exists and save the configuration
    CONFIG_DIR=$(dirname "$CONFIG_FILE")
    if [ ! -d "$CONFIG_DIR" ]; then
        mkdir -p "$CONFIG_DIR"
    fi
    
    # Save both the IP and the last selected platform in the config file
    echo "PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME" > "$CONFIG_FILE"
    export PX4_SIM_HOSTNAME="$PX4_SIM_HOSTNAME"
    echo "Exported PX4_SIM_HOSTNAME=$PX4_SIM_HOSTNAME"
fi

# === Global Access Setup (Optional) ===
if ! command -v px4xplane &> /dev/null; then
    highlight "Do you want to set up global access to the 'px4xplane' command so you can run this script from anywhere?"
    echo "Press Enter to confirm (default: yes) or type 'n' to skip."
    read -t 10 -r GLOBAL_SETUP
    if [[ -z "$GLOBAL_SETUP" || "$GLOBAL_SETUP" =~ ^[Yy]$ ]]; then
        if [ -d "$HOME/bin" ]; then
            ln -sf "$SCRIPT_PATH" "$HOME/bin/px4xplane"
        elif [ -d "$HOME/.local/bin" ]; then
            ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/px4xplane"
        else
            mkdir -p "$HOME/.local/bin"
            ln -sf "$SCRIPT_PATH" "$HOME/.local/bin/px4xplane"
            echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.bashrc"
            source "$HOME/.bashrc"
        fi
        chmod +x "$SCRIPT_PATH"
        highlight "Global access to 'px4xplane' command set up successfully."
        echo "You can now run 'px4xplane' from anywhere to execute this script."
    else
        echo "Global access setup skipped."
    fi
fi

# === Platform Selection and Build ===
highlight "Please select the platform to build:"
select PLATFORM in "${PLATFORM_CHOICES[@]}"; do
    if [[ " ${PLATFORM_CHOICES[@]} " =~ " ${PLATFORM} " ]]; then
        highlight "You have selected $PLATFORM. Building..."
        LAST_PLATFORM="$PLATFORM"
        echo "LAST_PLATFORM=$LAST_PLATFORM" >> "$CONFIG_FILE"
        make px4_sitl_default "$PLATFORM"
        break
    else
        highlight "Invalid selection. Please choose a valid platform."
    fi
done

highlight "Make sure that the selected airframe is defined and selected inside X-Plane's menu and loaded."
highlight "Follow the video tutorials and instructions for more guidance."

# === Final Completion Message ===
highlight "Script completed. Summary:"
highlight "- Repository cloned: $CLONE_PATH"
highlight "- Dependencies installed"
highlight "- Submodules updated"
highlight "- Platform built: $PLATFORM"
highlight "You can now proceed with running the SITL simulation."

# === End of Script ===
