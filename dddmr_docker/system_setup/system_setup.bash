#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ==============================================================================
# 1. Firefox Installation
# ==============================================================================
read -r -p "Do you want to install Firefox? [y/N]: " install_firefox || install_firefox="n"

case "$install_firefox" in
    [yY]|[yY][eE][sS])
        echo "Checking Ubuntu version for Firefox installation..."
        if [ -f /etc/os-release ]; then
            . /etc/os-release
            OS_ID="$ID"
            OS_VERSION="$VERSION_ID"
        elif command -v lsb_release >/dev/null 2>&1; then
            OS_ID=$(lsb_release -si | tr '[:upper:]' '[:lower:]')
            OS_VERSION=$(lsb_release -sr)
        else
            echo "Error: Unable to detect operating system version." >&2
            exit 1
        fi

        echo "Detected OS: ${OS_ID} ${OS_VERSION}"

        case "$OS_VERSION" in
            22.04*|2204*)
                VERSION_FOLDER="2204"
                ;;
            24.04*|2404*)
                VERSION_FOLDER="2404"
                ;;
            *)
                echo "Error: Unsupported Ubuntu version (${OS_VERSION}). Supported versions: 22.04, 24.04." >&2
                exit 1
                ;;
        esac

        FIREFOX_SCRIPT="$SCRIPT_DIR/$VERSION_FOLDER/install_firefox.bash"
        if [ ! -f "$FIREFOX_SCRIPT" ]; then
            echo "Error: Installation script not found at '$FIREFOX_SCRIPT'." >&2
            exit 1
        fi

        echo "Running installer: $FIREFOX_SCRIPT"
        bash "$FIREFOX_SCRIPT"
        ;;
    *)
        echo "Firefox installation skipped."
        ;;
esac

echo ""

# ==============================================================================
# 2. Visual Studio Code Installation
# ==============================================================================
read -r -p "Do you want to install Visual Studio Code? [y/N]: " install_vscode || install_vscode="n"

case "$install_vscode" in
    [yY]|[yY][eE][sS])
        ARCH=$(uname -m)
        echo "Detected architecture: ${ARCH}"

        if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
            VSCODE_SCRIPT="$SCRIPT_DIR/aarch64/vscode_install_aarch64.bash"
            if [ ! -f "$VSCODE_SCRIPT" ]; then
                echo "Error: Installation script not found at '$VSCODE_SCRIPT'." >&2
                exit 1
            fi

            echo "Running installer: $VSCODE_SCRIPT"
            bash "$VSCODE_SCRIPT"
        else
            echo "Skipping VS Code installation: architecture is ${ARCH} (vscode_install_aarch64.bash is only for aarch64)."
        fi
        ;;
    *)
        echo "Visual Studio Code installation skipped."
        ;;
esac

echo ""

# ==============================================================================
# 3. L4T Clocks, Network Buffer & IP Query Script Generation
# ==============================================================================
read -r -p "Do you want to generate l4t clocks and enlarge network socket's buffer? [y/N]: " setup_l4t_and_network || setup_l4t_and_network="n"

case "$setup_l4t_and_network" in
    [yY]|[yY][eE][sS])
        echo "Available network interfaces:"
        if command -v ip >/dev/null 2>&1; then
            ip -br link show
        elif [ -d /sys/class/net ]; then
            ls -1 /sys/class/net
        else
            /sbin/ifconfig -a | grep '^[a-zA-Z0-9]' | cut -d: -f1
        fi

        echo ""
        read -r -p "Enter network interface to query [default: eno1]: " target_iface || target_iface=""
        target_iface="$(echo "$target_iface" | xargs)"
        target_iface="${target_iface:-eno1}"

        SYSTEM_SETUP_SCRIPT_PATH="/usr/local/bin/dddmr_l4t_system_setup.bash"
        echo "Generating $SYSTEM_SETUP_SCRIPT_PATH for interface '$target_iface'..."
        sudo tee "$SYSTEM_SETUP_SCRIPT_PATH" > /dev/null << EOF
#!/bin/bash

echo -e Get IP from $target_iface
ip="\$(/sbin/ifconfig $target_iface | grep 'inet ' | tr -s ' ' | cut -d" " -f3)"

while [ "\$ip" == "" ]; do
  ip="\$(/sbin/ifconfig $target_iface | grep 'inet ' | tr -s ' ' | cut -d" " -f3)"
  echo -e "Querying for ip address..."
  sleep 1
done

echo -e "Got IP: \$(/sbin/ifconfig $target_iface | grep 'inet ' | tr -s ' ' | cut -d" " -f3)"

sudo jetson_clocks
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.ipv4.ipfrag_time=3
EOF
        sudo chmod +x "$SYSTEM_SETUP_SCRIPT_PATH"
        echo "Generated $SYSTEM_SETUP_SCRIPT_PATH successfully."

        SERVICE_FILE="/etc/systemd/system/dddmr_l4t_system_setup.service"
        echo "Creating systemd service: $SERVICE_FILE..."
        sudo tee "$SERVICE_FILE" > /dev/null << EOF
[Unit]
Description=DDMR L4T System Setup
After=network.target

[Service]
Type=oneshot
ExecStart=$SYSTEM_SETUP_SCRIPT_PATH
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
        sudo systemctl daemon-reload
        echo "Created $SERVICE_FILE."

        read -r -p "Do you want to activate dddmr_l4t_system_setup.service? [y/N]: " activate_service || activate_service="n"
        case "$activate_service" in
            [yY]|[yY][eE][sS])
                sudo systemctl enable dddmr_l4t_system_setup.service
                sudo systemctl start dddmr_l4t_system_setup.service
                echo "Activated and enabled dddmr_l4t_system_setup.service successfully."
                ;;
            *)
                echo "dddmr_l4t_system_setup.service created but not activated."
                ;;
        esac
        ;;
    *)
        echo "Skipped generating l4t clocks and network buffer script."
        ;;
esac

echo ""

# ==============================================================================
# 4. jtop (jetson-stats) Installation
# ==============================================================================
read -r -p "Do you want to install jtop (jetson-stats)? [y/N]: " install_jtop || install_jtop="n"

case "$install_jtop" in
    [yY]|[yY][eE][sS])
        echo "Installing jtop (jetson-stats)..."
        sudo apt update
        sudo apt install -y python3-pip
        if pip3 install --help 2>&1 | grep -q -- '--break-system-packages'; then
            sudo pip3 install -U jetson-stats --break-system-packages
            sudo jtop --install-service
        else
            sudo pip3 install -U jetson-stats
        fi
        echo "jtop installed successfully."
        ;;
    *)
        echo "jtop installation skipped."
        ;;
esac

