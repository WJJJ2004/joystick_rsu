# Copyright 2025 DeepMind Technologies Limited
# (Adapted for Linux evdev backend)
#
# Logitech F710 Gamepad reader using evdev (/dev/input/event*).
# Keeps the same public API as the hid-based version:
# - Gamepad.vx, Gamepad.vy, Gamepad.wz
# - Gamepad.is_running
# - Gamepad.get_command()
# - Gamepad.stop()
#
# Notes:
# - This reads from kernel input events (Handlers=eventX jsY).
# - It does NOT use hidraw/hidapi, so it avoids open() / open_path() issues.

import threading
import time
from typing import Optional, Dict

import numpy as np

try:
    from evdev import InputDevice, list_devices, ecodes
except ImportError as e:
    raise ImportError(
        "evdev is required. Install with: pip install evdev\n"
        "If you are in a venv, make sure you installed it inside the venv."
    ) from e


def _interpolate(value, old_max, new_scale, deadzone=0.01):
    # value is already in [-old_max, old_max]
    ret = value * new_scale / old_max
    if abs(ret) < deadzone:
        return 0.0
    return ret


def _normalize_abs(value: int, absinfo) -> float:
    """
    Normalize an absolute axis value to [-1, 1] using evdev AbsInfo.
    """
    # absinfo: (value, min, max, fuzz, flat, resolution) or AbsInfo object
    # evdev provides .min, .max
    mn = absinfo.min
    mx = absinfo.max
    if mx == mn:
        return 0.0
    # center: many gamepads are centered around mid, but not always symmetric.
    # Map linearly to [-1, 1] around center.
    center = (mx + mn) / 2.0
    half_range = (mx - mn) / 2.0
    return float((value - center) / half_range)


class Gamepad:
    """
    Gamepad class that reads from a Logitech F710 gamepad via evdev.

    Keeps API compatibility with the original hid-based implementation.
    """

    def __init__(
        self,
        vendor_id: int = 0x046D,
        product_id: int = 0xC219,  # DirectInput mode for F710 in your setup
        vel_scale_x: float = 0.4,
        vel_scale_y: float = 0.4,
        vel_scale_rot: float = 1.0,
        device_path: Optional[str] = None,
        prefer_name_contains: str = "Logitech",  # optional hint
    ):
        self._vendor_id = vendor_id
        self._product_id = product_id
        self._vel_scale_x = vel_scale_x
        self._vel_scale_y = vel_scale_y
        self._vel_scale_rot = vel_scale_rot

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.is_running = True

        self._device: Optional[InputDevice] = None
        self._device_path = device_path
        self._prefer_name_contains = prefer_name_contains

        # Axis code mapping (filled after connecting)
        self._axis_codes: Dict[str, int] = {}
        self._absinfo: Dict[int, object] = {}

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def _find_device_path(self) -> Optional[str]:
        """
        Find an event device path for the given vid/pid, falling back to name match.
        """
        candidates = []
        for path in list_devices():
            try:
                dev = InputDevice(path)
                # evdev gives vendor/product on the input device info
                vid = getattr(dev.info, "vendor", None)
                pid = getattr(dev.info, "product", None)
                name = (dev.name or "")
                if vid == self._vendor_id and pid == self._product_id:
                    candidates.append((path, name, True))
                elif self._prefer_name_contains and (self._prefer_name_contains.lower() in name.lower()):
                    candidates.append((path, name, False))
            except Exception:
                continue

        if not candidates:
            return None

        # Prefer exact vid/pid matches first, then name-only matches
        candidates.sort(key=lambda x: (not x[2], x[0]))
        return candidates[0][0]

    def _setup_axis_mapping(self):
        """
        Determine which ABS_* codes correspond to:
        - left stick x/y
        - right stick x (rotation)
        """
        assert self._device is not None

        caps = self._device.capabilities(absinfo=True)
        abs_caps = caps.get(ecodes.EV_ABS, [])
        # abs_caps is list of tuples: (code, absinfo)
        absinfo_by_code = {}
        for item in abs_caps:
            if isinstance(item, tuple) and len(item) == 2:
                code, info = item
                absinfo_by_code[code] = info

        self._absinfo = absinfo_by_code

        # Common mappings:
        # Left stick: ABS_X, ABS_Y
        # Right stick X: ABS_RX or ABS_Z (depends on driver/mode)
        # We'll pick the first available among candidates.
        def pick_first(*codes):
            for c in codes:
                if c in absinfo_by_code:
                    return c
            return None

        left_x = pick_first(ecodes.ABS_X)
        left_y = pick_first(ecodes.ABS_Y)

        # Try common right-x candidates in order
        right_x = pick_first(ecodes.ABS_RX, ecodes.ABS_Z, ecodes.ABS_RZ)

        if left_x is None or left_y is None:
            raise OSError(
                f"Could not find required ABS axes. Found ABS codes: {list(absinfo_by_code.keys())}"
            )
        if right_x is None:
            # If no obvious right-x, fall back to ABS_RX missing case:
            # allow operation without rotation, but log.
            right_x = -1  # sentinel

        self._axis_codes = {"left_x": left_x, "left_y": left_y, "right_x": right_x}

    def _connect_device(self) -> bool:
        try:
            path = self._device_path or self._find_device_path()
            if not path:
                raise OSError(
                    f"No evdev device found for {self._vendor_id:04x}:{self._product_id:04x}. "
                    f"Check /proc/bus/input/devices for event handler (e.g., event10)."
                )

            self._device = InputDevice(path)
            self._device.grab()  # prevent other consumers from interfering (optional, but stabilizes)
            self._setup_axis_mapping()

            print(f"Connected to evdev device: {path}")
            print(f"  Name: {self._device.name}")
            print(f"  VID:PID = {self._device.info.vendor:04x}:{self._device.info.product:04x}")
            print(f"  Axis mapping: {self._axis_codes}")
            return True

        except PermissionError as e:
            print(
                "PermissionError: cannot open /dev/input/event*. "
                "Try running with sudo or add a udev rule for your device."
            )
            print(f"Error connecting to device: {e}")
            return False
        except OSError as e:
            print(f"Error connecting to device: {e}")
            return False
        except Exception as e:
            print(f"Unexpected error connecting to device: {e}")
            return False

    def read_loop(self):
        if not self._connect_device():
            self.is_running = False
            return

        assert self._device is not None

        # Track latest normalized axis values
        lx = 0.0
        ly = 0.0
        rx = 0.0

        try:
            for event in self._device.read_loop():
                if not self.is_running:
                    break

                if event.type != ecodes.EV_ABS:
                    continue

                code = event.code
                value = event.value

                # Normalize if we have absinfo
                info = self._absinfo.get(code)
                if info is None:
                    continue

                n = _normalize_abs(value, info)  # [-1, 1]

                if code == self._axis_codes["left_x"]:
                    lx = n
                elif code == self._axis_codes["left_y"]:
                    ly = n
                elif self._axis_codes["right_x"] != -1 and code == self._axis_codes["right_x"]:
                    rx = n

                # Match original sign conventions:
                # original HID code:
                # left_x = -(data[1] - 128) / 128.0
                # left_y = -(data[2] - 128) / 128.0
                # right_x = -(data[3] - 128) / 128.0
                # => negate all
                left_x = -lx
                left_y = -ly
                right_x = -rx

                # Original mapping:
                # vx <- left_y, vy <- left_x, wz <- right_x
                self.vx = _interpolate(left_y, 1.0, self._vel_scale_x)
                self.vy = _interpolate(left_x, 1.0, self._vel_scale_y)
                self.wz = _interpolate(right_x, 1.0, self._vel_scale_rot)

        except OSError as e:
            # If device disconnects or read fails
            print(f"Error reading from device: {e}")
        except Exception as e:
            print(f"Unexpected error reading from device: {e}")
        finally:
            try:
                # release grab if possible
                self._device.ungrab()
            except Exception:
                pass
            try:
                self._device.close()
            except Exception:
                pass
            self.is_running = False

    def get_command(self):
        return np.array([self.vx, self.vy, self.wz], dtype=np.float32)

    def stop(self):
        self.is_running = False


if __name__ == "__main__":
    gamepad = Gamepad()
    while gamepad.is_running:
        print(gamepad.get_command())
        time.sleep(0.1)
    print("Gamepad not running (connection failed or disconnected).")
