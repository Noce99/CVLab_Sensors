"""Prophesee EVK4 device settings shared by the recorder and the live calibration capture."""

import glob
import os

from cvlab_sensors import config

DELTA_T_US = 10_000   # events are read in batches of this duration


def mask_hot_pixels(device, serial):
    """Mask the EVENT_HOT_PIXELS of this camera in hardware; returns the configured list."""
    hot_pixels = config.EVENT_HOT_PIXELS.get(serial, [])
    if hot_pixels:
        masks = device.get_i_digital_event_mask().get_pixel_masks()
        if len(hot_pixels) > len(masks):
            print(f"WARNING: {len(hot_pixels)} hot pixels but only {len(masks)} hardware masks")
        for mask, (x, y) in zip(masks, hot_pixels):
            mask.set_mask(x, y, True)
        print(f"Masked hot pixels: {hot_pixels[:len(masks)]}")
    return hot_pixels


def limit_event_rate(device, rate):
    """Cap the event rate in hardware (ERC), in events/s; None disables the cap."""
    erc = device.get_i_erc_module()
    if erc is None:
        print("WARNING: no ERC module, event rate not capped")
        return
    if rate is not None:
        erc.set_cd_event_rate(int(rate))
    erc.enable(rate is not None)
    print(f"Event rate cap: {f'{rate / 1e6:.0f} Mev/s' if rate is not None else 'off'}")


def evk4_usb_speeds_mbps():
    """USB link speed of every connected EVK4 (they expose no serial in sysfs)."""
    speeds = []
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                vendor = f.read().strip()
            with open(os.path.join(dev, "idProduct")) as f:
                product = f.read().strip()
            if (vendor, product) == ("04b4", "00f5"):
                with open(os.path.join(dev, "speed")) as f:
                    speeds.append(int(f.read().strip()))
        except (OSError, ValueError):
            pass
    return speeds
