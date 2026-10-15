#!/bin/bash
# Reset the EVK4 event cameras (re-authorize them, so that they open reliably) and check that they
# are on USB3.
#
# At a cold boot the EVK4s power up together with their hub and miss the SuperSpeed link training:
# they enumerate at 480 Mbps on the USB2 side of the hub (lsusb -t: under a 480M bus) and stay
# there. Only a power cycle of the camera brings it to USB3: re-plug it. Software cannot do it on
# this hub (Genesys 05e3:0610/0626): tried on 2026-10-15, turning off the camera's port, and all the
# ports of the hub (ganged power switching), re-enumerated the cameras on USB2 again. A hub with
# per-port power switching (uhubctl) would allow it.
#
# Needs root. Run by the app before recording the event cameras (sudo -n: sudoers rule in the README).
# Exit code: 0 every EVK4 on USB3, 2 some EVK4 on USB2 (re-plug it), 1 no EVK4 / not root.

set -u
VENDOR=04b4
PRODUCT=00f5
DEVICES=/sys/bus/usb/devices

if [ "$EUID" -ne 0 ]; then
    echo "needs root: sudo $0" >&2
    exit 1
fi

cameras=()
for dev in "$DEVICES"/*; do
    if [ "$(cat "$dev/idVendor" 2>/dev/null)" = "$VENDOR" ] && [ "$(cat "$dev/idProduct" 2>/dev/null)" = "$PRODUCT" ]; then
        cameras+=("$(basename "$dev")")
    fi
done
if [ ${#cameras[@]} -eq 0 ]; then
    echo "No EVK4 found" >&2
    exit 1
fi

for cam in "${cameras[@]}"; do
    echo 0 > "$DEVICES/$cam/authorized"
done
sleep 1
for cam in "${cameras[@]}"; do
    echo 1 > "$DEVICES/$cam/authorized"
done

status=0
for cam in "${cameras[@]}"; do
    speed=$(cat "$DEVICES/$cam/speed")
    echo "$cam: $speed Mbps"
    if [ "$speed" -lt 5000 ]; then
        echo "EVK4 $cam on USB2 ($speed Mbps, it happens at a cold boot): re-plug it" >&2
        status=2
    fi
done
exit $status
