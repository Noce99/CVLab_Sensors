"""Prophesee EVK4 device settings (from the Settings page) shared by the recorder and the live tools."""

import glob
import os

from cvlab_sensors import config
from cvlab_sensors.settings import store

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


def configure(device, serial, values, max_rate_mev):
    """Configure an opened EVK4 with the "event" settings (cvlab_sensors.settings.store), before
    streaming. max_rate_mev: the cap of this use (recording or live tools). Returns the
    applied report: the values, what the camera reports back and what it could not set."""
    import metavision_hal

    notes, readback = [], {}
    readback["masked_pixels"] = mask_hot_pixels(device, serial)
    limit_event_rate(device, None if max_rate_mev is None else max_rate_mev * 1e6)
    erc = device.get_i_erc_module()
    if erc is not None:
        readback["max_rate_mev"] = erc.get_cd_event_rate() / 1e6 if erc.is_enabled() else None

    biases = device.get_i_ll_biases()
    if biases is None:
        notes.append("no bias interface: biases left at the camera defaults")
    else:
        readback["bias_ranges"] = {}
        for name in store.EVENT_BIASES:
            value = values[name]
            try:
                if value is not None and not biases.set(name, int(value)):
                    notes.append(f"{name} {value} rejected by the camera")
            except Exception as e:
                notes.append(f"{name} {value} rejected by the camera: {e}")
            try:
                readback[name] = biases.get(name)
                readback["bias_ranges"][name] = list(biases.get_bias_info(name).get_bias_allowed_range())
            except Exception:
                pass

    def module(getter, label, wanted):
        found = getter()
        if found is None and wanted:
            notes.append(f"no {label} on this camera: not applied")
        return found

    afk = module(device.get_i_antiflicker_module, "anti-flicker module", values["antiflicker"])
    if afk is not None:
        try:
            if values["antiflicker"]:
                afk.set_filtering_mode(metavision_hal.I_AntiFlickerModule.AntiFlickerMode.__members__[values["antiflicker_mode"]])
                afk.set_frequency_band(values["antiflicker_low_hz"], values["antiflicker_high_hz"])
            afk.enable(values["antiflicker"])
            readback["antiflicker"] = afk.is_enabled()
            readback["antiflicker_low_hz"] = afk.get_band_low_frequency()
            readback["antiflicker_high_hz"] = afk.get_band_high_frequency()
        except Exception as e:
            notes.append(f"anti-flicker not applied: {e}")

    trail = module(device.get_i_event_trail_filter_module, "trail filter", values["trail_filter"])
    if trail is not None:
        try:
            if values["trail_filter"]:
                trail.set_type(metavision_hal.I_EventTrailFilterModule.Type.__members__[values["trail_type"]])
                trail.set_threshold(values["trail_threshold_us"])
            trail.enable(values["trail_filter"])
            readback["trail_filter"] = trail.is_enabled()
            readback["trail_threshold_us"] = trail.get_threshold()
        except Exception as e:
            notes.append(f"trail filter not applied: {e}")

    roi = module(device.get_i_roi, "hardware ROI", values["roi"])
    if roi is not None:
        try:
            if values["roi"]:
                roi.set_window(metavision_hal.I_ROI.Window(values["roi_x"], values["roi_y"],
                                                           values["roi_width"], values["roi_height"]))
            roi.enable(values["roi"])
            readback["roi"] = values["roi"]
        except Exception as e:
            notes.append(f"ROI not applied: {e}")

    if values["trigger_in"]:
        device.get_i_trigger_in().enable(metavision_hal.I_TriggerIn.Channel.MAIN)
        print("Trigger input MAIN enabled")
    readback["trigger_in"] = values["trigger_in"]

    for note in notes:
        print(f"WARNING event camera {serial}: {note}")
    return store.applied_report("event", values, readback, notes)


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
