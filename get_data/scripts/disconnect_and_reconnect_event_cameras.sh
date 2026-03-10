#!/bin/bash

# Script to disconnect and reconnect EVK4 cameras
# Automatically uses sudo when needed

# Function to run commands with sudo if not already root
run_with_sudo() {
    if [ "$EUID" -eq 0 ]; then
        # Already root, run directly
        "$@"
    else
        # Not root, use sudo
        sudo "$@"
    fi
}

echo "Searching for EVK4 cameras..."

# Find all EVK4 devices
evk4_paths=()
while read -r path; do
    if [[ -f "$path/idVendor" ]] && [[ -f "$path/idProduct" ]]; then
        vendor=$(cat "$path/idVendor" 2>/dev/null)
        product=$(cat "$path/idProduct" 2>/dev/null)
        
        if [[ "$vendor" == "04b4" ]] && [[ "$product" == "00f5" ]]; then
            evk4_paths+=("$path")
            echo "Found EVK4 at: $path"
        fi
    fi
done < <(find /sys/bus/usb/devices/ -maxdepth 1 -type l 2>/dev/null)

# Check if any cameras were found
if [ ${#evk4_paths[@]} -eq 0 ]; then
    echo "No EVK4 cameras found!"
    exit 1
fi

echo "Found ${#evk4_paths[@]} EVK4 camera(s)"
echo "Starting disconnect/reconnect cycle..."

# Disconnect all cameras
for path in "${evk4_paths[@]}"; do
    echo "Disconnecting: $path"
    echo '0' | run_with_sudo tee "$path/authorized" > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error: Failed to disconnect $path"
        exit 1
    fi
done

# Wait 1 second
echo "Waiting 1 second..."
sleep 1

# Reconnect all cameras
for path in "${evk4_paths[@]}"; do
    echo "Reconnecting: $path"
    echo '1' | run_with_sudo tee "$path/authorized" > /dev/null
    if [ $? -ne 0 ]; then
        echo "Error: Failed to reconnect $path"
        exit 1
    fi
done

echo "Done! EVK4 cameras have been reset."

# Optional: Show the devices after reset
echo ""
echo "Current EVK4 devices:"
run_with_sudo lsusb | grep "04b4:00f5" 2>/dev/null || lsusb | grep "04b4:00f5"