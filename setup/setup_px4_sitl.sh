#!/bin/bash

# === PX4 X-Plane SITL Setup Script ===
#
# Author: Alireza Ghaderi
# LinkedIn: alireza787b
# Date: August 2024
# GitHub Repo: alireza787b/px4xplane
#
# === Purpose ===
# This script automates the setup of PX4 Software-In-The-Loop (SITL) integration with X-Plane.
# It is intended as a temporary solution until the necessary modifications are officially merged into the PX4 main repository.
# Currently, the script clones a forked version of PX4 (https://github.com/alireza787b/PX4-Autopilot-Me.git),
# which contains customized airframes specifically designed for X-Plane simulation.
#
# === Functionality ===
# The script performs the following tasks in a sequential manner:
# 1. Clones the PX4 repository from the forked repo (customized for X-Plane).
# 2. Sets up the necessary dependencies by running the provided setup script (`ubuntu.sh`).
# 3. Optionally sets up MAVLink Router, which routes MAVLink traffic to specified endpoints (typically for communication with X-Plane).
# 4. Optionally sets up global access to the `px4xplane` command, allowing the script to be run from anywhere.
# 5. Prompts the user to select a platform to build (e.g., specific airframes for X-Plane) and initiates the build process.
#
# === Key Considerations ===
# - **Temporary Fix**: This setup is a temporary fix until the official PX4 repository includes the necessary modifications for X-Plane support.
# - **Custom Fork**: The script clones a forked version of the PX4 repository (`alireza787b/PX4-Autopilot-Me`) that contains customized airframes for X-Plane.
# - **MAVLink Router**: If enabled, the script installs and runs MAVLink Router to facilitate communication between PX4 SITL and X-Plane over the network.
# - **Global Access**: The script offers the option to set up global access, making it easier to run the script from anywhere on the system by typing `px4xplane`.
# - **Repair Mode**: A repair mode is available to force a full setup process, including pulling the latest changes from the repository and reinstalling dependencies.
# - **Uninstall Option**: An uninstall option is provided to remove the global `px4xplane` command.
#
# === Usage ===
# - **Normal Run**: Run the script without any arguments to perform a normal setup or continue an existing setup.
#   Example: `./px4xplane_script.sh`
# - **Custom Installation Path**: You can specify a custom installation directory as an argument.
#   Example: `./px4xplane_script.sh ~/custom_install_path`
# - **Repair Mode**: Use the `--repair` flag to force the script to re-run the full setup process (useful if something went wrong or needs to be reset).
#   Example: `./px4xplane_script.sh --repair`
# - **Uninstall**: Use the `--uninstall` flag to remove the global `px4xplane` command from your system.
#   Example: `./px4xplane_script.sh --uninstall`
#
# === Customization Options ===
# The script includes several configurable variables at the top, allowing you to customize the behavior:
# - `REPO_URL`: URL of the forked PX4 repository to clone (currently set to `https://github.com/alireza787b/PX4-Autopilot-Me.git`).
# - `BRANCH_NAME`: The branch of the repository to checkout (currently set to `px4xplane-sitl`).
# - `UPSTREAM_URL`: URL of the upstream PX4 repository (used to fetch tags and pull updates).
# - `DEFAULT_CLONE_PATH`: The default installation directory if no custom path is provided.
# - `DEFAULT_CONFIG_FILE`: Path to the configuration file that stores user settings (such as IP address and selected platform).
# - `DEFAULT_FALLBACK_IP`: Fallback IP address (used if the script cannot detect an IP address).
# - `PLATFORM_CHOICES`: A list of platforms (airframes) to choose from for the SITL build.
# - **MAVLink Router Configuration**:
#   - `USE_MAVLINK_ROUTER`: Enable or disable MAVLink Router installation and setup.
#   - `MAVLINK_ROUTER_INSTALL_SCRIPT_URL`: URL of the installation script for MAVLink Router.
#   - `MAVLINK_ROUTER_COMMAND`: Command to run MAVLink Router (with IP placeholders).
#
# === Detailed Workflow ===
# 1. **Initial Setup**: The script checks if the PX4 repository is already cloned and if configuration files are present.
#    If not, it clones the repository, installs dependencies, and pulls the latest changes.
# 2. **MAVLink Router Setup (Optional)**: If enabled, the script installs and runs MAVLink Router, which routes MAVLink traffic
#    between PX4 SITL and the X-Plane simulator. The IP address detected by the script is used to configure the routing.
# 3. **Global Access Setup (Optional)**: The script can set up a global `px4xplane` command, allowing the user to run the script from any directory.
# 4. **Platform Selection and Build**: The script prompts the user to select a platform (airframe) to build and then initiates the build process for the selected platform.
# 5. **Repair Mode**: If `--repair` is passed, the script runs the full setup process again, ensuring that everything is up-to-date and reinstalled.
# 6. **Uninstall**: If `--uninstall` is passed, the script removes the global `px4xplane` command from the system.
#
# === Important Notes ===
# - This script is primarily designed for use on Linux, particularly for WSL users running X-Plane on Windows and PX4 SITL on WSL.
# - If MAVLink Router is enabled, the script will attempt to detect the Windows IP address on WSL, or prompt the user to enter it manually.
# - The `--repair` option can be useful if any part of the setup encounters issues, or if the repository needs to be updated to the latest version.
# - The script is structured to be modular, allowing for easy customization by modifying the configurable variables at the top of the script.
#
# === Future Considerations ===
# Once the customized airframes and modifications are officially merged into the PX4 main repository, this script will no longer be needed.
# The installation process would then revert to using the official PX4 repository without the need for the custom fork.


# === Configurable Variables ===
REPO_URL="https://github.com/alireza787b/PX4-Autopilot-Me.git"
BRANCH_NAME="px4xplane-sitl"
UPSTREAM_URL="https://github.com/PX4/PX4-Autopilot.git"
DEFAULT_CLONE_PATH="$HOME"
DEFAULT_CONFIG_FILE="$HOME/.px4sitl_config"
DEFAULT_FALLBACK_IP="127.0.0.1"
SCRIPT_NAME="px4xplane_script.sh"
MAVLINK2REST_IP="127.0.0.1"
PLATFORM_CHOICES=("xplane_ehang184" "xplane_alia250" "xplane_cessna172" "xplane_tb2" "xplane_qtailsitter)

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

# === Trap for Cleanup ===
cleanup() {
    if [[ -n "$MAVLINK_ROUTER_PID" ]]; then
        highlight "Stopping MAVLink Router..."
        kill "$MAVLINK_ROUTER_PID"
    fi
}
trap cleanup EXIT

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
    # === Full Repair Mode (If Flag is Set or Configured) ===
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
    cd "$CLONE_PATH" || exit  # Ensure we are in the repository directory before building
    make clean
fi

if [[ "$DISTCLEAN_BUILD" =~ ^[Yy]$ ]]; then
    cd "$CLONE_PATH" || exit  # Ensure we are in the repository directory before building
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

    # === MAVLink Router Setup ===
    if [ "$USE_MAVLINK_ROUTER" = true ]; then
        highlight "Setting up MAVLink Router..."
        MAVLINK_ROUTER_INSTALL_SCRIPT="$INSTALL_PATH/install_mavlink_router.sh"

        # Download the MAVLink Router installation script
        if [ ! -f "$MAVLINK_ROUTER_INSTALL_SCRIPT" ]; then
            curl -o "$MAVLINK_ROUTER_INSTALL_SCRIPT" "$MAVLINK_ROUTER_INSTALL_SCRIPT_URL"
            chmod +x "$MAVLINK_ROUTER_INSTALL_SCRIPT"
        fi

        # Run the MAVLink Router installation script
        bash "$MAVLINK_ROUTER_INSTALL_SCRIPT"

        # Replace specific IP placeholders in the MAVLink Router command
        MAVLINK_ROUTER_CMD=$(echo "$MAVLINK_ROUTER_COMMAND" | sed "s/IP_PLACEHOLDER/$PX4_SIM_HOSTNAME/g" | sed "s/MAVLINK2REST_IP_PLACEHOLDE/$MAVLINK2REST_IP/g")
        highlight "Running MAVLink Router: $MAVLINK_ROUTER_CMD"
        $MAVLINK_ROUTER_CMD &
        MAVLINK_ROUTER_PID=$!

        # Provide additional instructions and delay for better visibility
        highlight "MAVLink Router is now running. You can connect to the following endpoints:"
        echo " - $PX4_SIM_HOSTNAME:14540"
        echo " - $PX4_SIM_HOSTNAME:14550"
        echo " - $PX4_SIM_HOSTNAME:14569"
        echo " - $MAVLINK2REST_IP:14569"
        echo " "
        highlight "Make sure to configure these endpoints in your X-Plane setup as needed."
        sleep 2  # Allow the user time to read the information
    fi
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
highlight "Please select the platform to build (or enter 0 to Exit):"
select PLATFORM in "${PLATFORM_CHOICES[@]}" "Exit"; do
    if [[ "$PLATFORM" == "Exit" ]]; then
        highlight "Exiting without building..."
        exit 0
    elif [[ " ${PLATFORM_CHOICES[@]} " =~ " ${PLATFORM} " ]]; then
        highlight "You have selected $PLATFORM. Building..."
        LAST_PLATFORM="$PLATFORM"
        echo "LAST_PLATFORM=$LAST_PLATFORM" >> "$CONFIG_FILE"
        cd "$CLONE_PATH" || exit  # Ensure we are in the repository directory before building
        make px4_sitl_default "$PLATFORM"
        break
    else
        highlight "Invalid selection. Please choose a valid platform or enter 0 to exit."
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
