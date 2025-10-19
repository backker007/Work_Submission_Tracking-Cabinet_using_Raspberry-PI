# shared/hardware_helpers.py
# -*- coding: utf-8 -*-
"""
Hardware helpers for Smart Locker
- I2C bus (Blinka)
- VL53L0X (multi-sensor via XSHUT)
- PCA9685 servos
- MCP23017 (relays + door switches + status LEDs)
- Smoothing / outlier rejection for distance reads
- LED helpers set_slot_led_ready / set_slot_led_error for main_controller

export: init_mcp, init_sensors, read_sensor, move_servo_180,
        is_door_reliably_closed, mcp_pins, relay_pins, CHANGE_THRESHOLD,
        is_slot_full, mcp, set_slot_led_ready, set_slot_led_error,
        set_slot_led_off, vl53_address_map, internet_ok, power_down_sensors,
        ensure_sensors_ready, reset_sensor_filter, read_sensor_fresh
"""

from __future__ import annotations

import os
import time
import json
import logging
import statistics
import socket
import threading
from collections import deque
from typing import Dict, List, Optional

import board
import busio
import digitalio
from digitalio import Direction, Pull
from adafruit_pca9685 import PCA9685
from adafruit_mcp230xx.mcp23017 import MCP23017

from .vl53l0x_init import (
    init_vl53x_four,
    read_mm,
    debug_summary,
    SensorHandle,
    _wait_for_addr as _vl53_wait_for_addr,
    _raw_set_address_confirm as _vl53_set_addr,
    _open_reader_with_retries as _vl53_open_with_retries,
)
from .topics import SLOT_IDS

log = logging.getLogger("hw")

# =============================================================================
# I2C & Peripherals
# =============================================================================
shared_i2c = busio.I2C(board.SCL, board.SDA)
_sensors_powered = False
_pm_lock = threading.RLock()

# --- PCA9685 (Servo) ---
pca = PCA9685(shared_i2c)
pca.frequency = 50  # 50Hz

# --- MCP23017 ---
mcp: Optional[MCP23017] = None
mcp_pins: List = []
relay_pins: List = []

# =============================================================================
# ENV Config
# =============================================================================
TARGET_MIN_MM = int(os.getenv("TARGET_MIN_MM", "80"))
TARGET_MAX_MM = int(os.getenv("TARGET_MAX_MM", "300"))

TIMING_BUDGET_US = int(os.getenv("VL53_BUDGET_US", "20000"))
VL53_BOOT_DELAY_S = float(os.getenv("VL53_BOOT_DELAY_S", "0.35"))
VL53_BOOT_TIMEOUT_S = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))
VL53_ADDR_SET_RETRIES = int(os.getenv("VL53_ADDR_SET_RETRIES", "4"))
VL53_ADDR_SET_GAP_S = float(os.getenv("VL53_ADDR_SET_GAP_S", "0.08"))
VL53_ALLOW_ADAFRUIT = os.getenv("VL53_ALLOW_ADAFRUIT", "1").lower() in ("1", "true", "yes")

SMOOTH_WINDOW = int(os.getenv("VL53_SMOOTH_WINDOW", "5"))
OUTLIER_MM = int(os.getenv("VL53_OUTLIER_MM", "15"))
CHANGE_THRESHOLD = int(os.getenv("CHANGE_THRESHOLD", "5"))

VL53_CONTINUOUS = os.getenv("VL53_CONTINUOUS", "0").lower() in ("1", "true", "yes")
INTER_SENSOR_DELAY_S = float(os.getenv("VL53_INTER_DELAY_S", "0.015"))

ADDRESS_BASE = int(os.getenv("VL53_BASE_ADDR", "0x30"), 16)

try:
    _OFFSETS = json.loads(os.getenv("VL53_OFFSETS", "{}"))
except Exception:
    _OFFSETS = {}

_xshut_env = os.getenv("VL53_XSHUT_PINS", "17")
_xshut_gpio = [int(p.strip()) for p in _xshut_env.split(",") if p.strip()]
XSHUT_PINS = [digitalio.DigitalInOut(getattr(board, f"D{gpio}")) for gpio in _xshut_gpio]

INIT_ALL_LOW = os.getenv("VL53_INIT_ALL_LOW", "1").lower() in ("1", "true", "yes")
BUS_MODE = os.getenv("VL53_BUS_MODE", "multi").strip().lower()

DOOR_SAMPLES = int(os.getenv("DOOR_SAMPLES", "20"))
DOOR_SAMPLE_INTERVAL_S = float(os.getenv("DOOR_SAMPLE_INTERVAL_S", "0.03"))

DOOR_WIRING = os.getenv("DOOR_WIRING", "GND").strip().upper()  # "GND" | "VCC"
DOOR_USE_PULLUP = os.getenv("DOOR_USE_PULLUP", "1").lower() in ("1", "true", "yes")
DOOR_SENSOR_INVERT = os.getenv("DOOR_SENSOR_INVERT", "0").lower() in ("1", "true", "yes")

SERVO_SETTLE_S = float(os.getenv("SERVO_SETTLE_S", "0.7"))

DEFAULT_FULL_THRESHOLD_MM = int(os.getenv("DEFAULT_FULL_THRESHOLD_MM", "40"))
SLOT_FULL_THRESHOLD_MM = {sid: DEFAULT_FULL_THRESHOLD_MM for sid in SLOT_IDS}

def is_slot_full(slot_id: str, distance_mm: Optional[float]) -> bool:
    if distance_mm is None:
        return False
    thr = SLOT_FULL_THRESHOLD_MM.get(slot_id, DEFAULT_FULL_THRESHOLD_MM)
    return distance_mm <= thr

NET_WATCHDOG = os.getenv("NET_WATCHDOG", "1").lower() in ("1", "true", "yes")
NET_CHECK_HOST = os.getenv("NET_CHECK_HOST") or os.getenv("MQTT_HOST") or "8.8.8.8"
NET_CHECK_PORT = int(os.getenv("NET_CHECK_PORT") or os.getenv("MQTT_PORT") or "53")
NET_CHECK_INTERVAL_S = float(os.getenv("NET_CHECK_INTERVAL_S", "3.0"))
NET_CONNECT_TIMEOUT_S = float(os.getenv("NET_CONNECT_TIMEOUT_S", "1.2"))
LED_BOOT_DEFAULT = os.getenv("LED_BOOT_DEFAULT", "ready").strip().lower()
LED_ONLINE_DEFAULT_READY = os.getenv("LED_ONLINE_DEFAULT_READY", "1").lower() in ("1", "true", "yes")
NET_OFFLINE_BLINK = os.getenv("NET_OFFLINE_BLINK", "1").lower() in ("1", "true", "yes")
NET_OFFLINE_BLINK_S = float(os.getenv("NET_OFFLINE_BLINK_S", "0.6"))

print(f"🔌 XSHUT GPIO pins from .env: {_xshut_gpio}")

# =============================================================================
# VL53L0X Backends
# =============================================================================
_handles_guard = threading.RLock()
_vl53_handles: Dict[int, SensorHandle] = {}
buffers: List[deque] = []
last_values: List[Optional[int]] = []
_last_read_ts: List[float] = []

try:
    import adafruit_vl53l0x  # type: ignore
except Exception:
    adafruit_vl53l0x = None

def _make_read(sensor):
    def _read():
        try:
            v = sensor.range
            if isinstance(v, (int, float)):
                return int(v)
        except Exception:
            return None
        return None
    return _read

def _open_reader(addr: int):
    if adafruit_vl53l0x is None:
        raise RuntimeError("adafruit_vl53l0x not installed")
    s = adafruit_vl53l0x.VL53L0X(shared_i2c, address=addr)
    if hasattr(s, "measurement_timing_budget"):
        try:
            s.measurement_timing_budget = TIMING_BUDGET_US
        except Exception:
            pass
    if VL53_CONTINUOUS:
        for m in ("start_continuous", "startContinuous"):
            if hasattr(s, m):
                try:
                    getattr(s, m)()
                except Exception:
                    pass
                break
    return s

def _expected_addr_for(index: int) -> int:
    return (ADDRESS_BASE + index) & 0x7F

def _recover_index_by_xshut(index: int) -> bool:
    if index >= len(XSHUT_PINS):
        log.warning(f"recover: no XSHUT pin for index {index}")
        return False

    target = _expected_addr_for(index)
    try:
        XSHUT_PINS[index].switch_to_output(value=False)
        time.sleep(max(0.05, float(os.getenv("VL53_HOLD_LOW_S", "0.20"))))
        XSHUT_PINS[index].value = True

        boot_to = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))

        if not _vl53_wait_for_addr(shared_i2c, 0x29, timeout_s=max(1.0, boot_to)):
            if not _vl53_wait_for_addr(shared_i2c, target, timeout_s=0.6):
                log.warning(f"recover: neither 0x29 nor target 0x{target:02X} present")
                return False
        else:
            _ok = _vl53_set_addr(
                shared_i2c, target,
                retries=int(os.getenv("VL53_ADDR_SET_RETRIES", "6")),
                gap_s=float(os.getenv("VL53_ADDR_SET_GAP_S", "0.10")),
            )
            if not _ok:
                log.warning("recover: set addr 0x%02X failed", target)
                return False

        s = _vl53_open_with_retries(
            shared_i2c, target, TIMING_BUDGET_US,
            retries=int(os.getenv("VL53_OPEN_RETRIES", "8")),
            delay_s=float(os.getenv("VL53_OPEN_DELAY_S", "0.30")),
        )
        h = SensorHandle(idx=index, addr=target, backend="adafruit",
                         handle=s, read=_make_read(s))
        with _handles_guard:
            _vl53_handles[index] = h
        log.warning(f"⚠️ Recovered index {index} @0x{target:02X} by XSHUT")
        return True

    except Exception as e:
        log.warning(f"recover index {index} failed: {e}")
        return False

def _reopen_handle(index: int) -> bool:
    addr = _expected_addr_for(index)
    try:
        s = _open_reader(addr)
        h = SensorHandle(idx=index, addr=addr, backend="adafruit", handle=s, read=_make_read(s))
        with _handles_guard:
            _vl53_handles[index] = h
        log.warning(f"⚠️ Reopened VL53L0X index {index} @0x{addr:02X}")
        return True
    except Exception as e:
        log.warning(f"❌ Soft reopen failed @0x{addr:02X}: {e}; try XSHUT...")
        return _recover_index_by_xshut(index)

# =============================================================================
# SERVO CONTROL
# =============================================================================
def angle_to_duty_cycle(angle: float) -> int:
    pulse_us = 500 + (angle / 180.0) * 2000
    return int((pulse_us / 20000.0) * 65535)

def move_servo_180(channel: int, angle: int) -> None:
    angle = max(0, min(180, int(angle)))
    angle = 180 - angle  # invert ทิศให้เหมาะกับกลไก
    duty = angle_to_duty_cycle(angle)
    log.debug(f"Servo CH{channel} → {angle}° (duty={duty})")
    pca.channels[channel].duty_cycle = duty
    time.sleep(max(0.2, SERVO_SETTLE_S))
    pca.channels[channel].duty_cycle = 0

# =============================================================================
# DOOR SENSOR (MC-38) via MCP23017
# =============================================================================
def _raw_closed(pin) -> bool:
    if DOOR_WIRING == "VCC":
        closed = (pin.value is True)
    else:
        closed = (pin.value is False)
    return (not closed) if DOOR_SENSOR_INVERT else closed

def is_door_reliably_closed(index: int, samples: Optional[int] = None,
                            interval: Optional[float] = None) -> bool:
    if samples is None:
        samples = DOOR_SAMPLES
    if interval is None:
        interval = DOOR_SAMPLE_INTERVAL_S

    pos = 8 + index  # 8..11
    if pos >= len(mcp_pins):
        log.error(f"Door sensor index {index} is out of range.")
        return False

    pin = mcp_pins[pos]
    for _ in range(max(1, samples)):
        if not _raw_closed(pin):
            return False
        time.sleep(max(0.001, interval))
    return True

# =============================================================================
# LED helpers
# =============================================================================
_LED_READY_PINS = [int(x) for x in os.getenv("SLOT_LED_READY_PINS", "0,1,2,3").split(",") if x.strip()]
_LED_ERROR_PINS = [int(x) for x in os.getenv("SLOT_LED_ERROR_PINS", "4,5,6,7").split(",") if x.strip()]
_LED_ACTIVE_HIGH = os.getenv("SLOT_LED_ACTIVE_HIGH", "1").lower() in ("1", "true", "yes")

_led_ready_pinobjs: List = []
_led_error_pinobjs: List = []

def _led_write(pin, on: bool) -> None:
    if pin is None:
        return
    pin.value = on if _LED_ACTIVE_HIGH else (not on)

def _set_led_pair(index: int, ready_on: bool, error_on: bool) -> None:
    try:
        pr = _led_ready_pinobjs[index] if index < len(_led_ready_pinobjs) else None
        pe = _led_error_pinobjs[index] if index < len(_led_error_pinobjs) else None
        _led_write(pr, ready_on)
        _led_write(pe, error_on)
    except Exception as e:
        log.debug(f"_set_led_pair({index}) ignored: {e}")

def _set_all_leds(ready_on: bool, error_on: bool) -> None:
    for i in range(len(_led_ready_pinobjs)):
        _set_led_pair(i, ready_on, error_on)

def set_slot_led_off(mcp_obj: Optional[MCP23017], index: int) -> None:
    try:
        _set_led_pair(index, False, False)
    except Exception as e:
        log.debug(f"set_slot_led_off({index}) ignored: {e}")

def set_slot_led_ready(mcp_obj: Optional[MCP23017], index: int) -> None:
    try:
        if not _network_online:
            _set_led_pair(index, False, True)
            return
        pin_ready = _led_ready_pinobjs[index] if index < len(_led_ready_pinobjs) else None
        pin_error = _led_error_pinobjs[index] if index < len(_led_error_pinobjs) else None
        _led_write(pin_ready, True)
        _led_write(pin_error, False)
    except Exception as e:
        log.debug(f"set_slot_led_ready({index}) ignored: {e}")

def set_slot_led_error(mcp_obj: Optional[MCP23017], index: int) -> None:
    try:
        pin_ready = _led_ready_pinobjs[index] if index < len(_led_ready_pinobjs) else None
        pin_error = _led_error_pinobjs[index] if index < len(_led_error_pinobjs) else None
        _led_write(pin_ready, False)
        _led_write(pin_error, True)
    except Exception as e:
        log.debug(f"set_slot_led_error({index}) ignored: {e}")

def _led_boot_default() -> None:
    if _network_online:
        if LED_BOOT_DEFAULT == "ready":
            _set_all_leds(True, False)
        elif LED_BOOT_DEFAULT == "error":
            _set_all_leds(False, True)
        else:
            _set_all_leds(False, False)
    else:
        _set_all_leds(False, True)

# =============================================================================
# Network watchdog
# =============================================================================
_network_online = True
_net_thread_started = False

def internet_ok() -> bool:
    return _network_online

def _tcp_connect_ok(host: str, port: int, timeout: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

def _apply_network_led_state(online: bool) -> None:
    if not online:
        _set_all_leds(False, True)
    else:
        _set_all_leds(LED_ONLINE_DEFAULT_READY, False)

def _net_watchdog_loop():
    global _network_online
    blink = False
    while True:
        ok = _tcp_connect_ok(NET_CHECK_HOST, NET_CHECK_PORT, NET_CONNECT_TIMEOUT_S)
        if ok != _network_online:
            _network_online = ok
            log.info("🌐 Network state changed: %s", "ONLINE" if ok else "OFFLINE")
            _apply_network_led_state(ok)
            blink = (NET_OFFLINE_BLINK and not ok)
        if blink:
            _set_all_leds(False, True)
            time.sleep(NET_OFFLINE_BLINK_S)
            _set_all_leds(False, False)
            time.sleep(NET_OFFLINE_BLINK_S)
        else:
            time.sleep(NET_CHECK_INTERVAL_S)

def _maybe_start_network_watchdog():
    global _net_thread_started
    if _net_thread_started:
        return
    t = threading.Thread(target=_net_watchdog_loop, daemon=True, name="net-watchdog")
    t.start()
    _net_thread_started = True

def _refresh_network_state_once():
    global _network_online
    ok = _tcp_connect_ok(NET_CHECK_HOST, NET_CHECK_PORT, NET_CONNECT_TIMEOUT_S)
    _network_online = ok
    log.info("🌐 Network initial: %s (%s:%s)", "ONLINE" if ok else "OFFLINE", NET_CHECK_HOST, NET_CHECK_PORT)

# =============================================================================
# MCP23017 init
# =============================================================================
def init_mcp() -> None:
    global mcp, mcp_pins, relay_pins, _led_ready_pinobjs, _led_error_pinobjs

    mcp = MCP23017(shared_i2c)

    mcp_pins.clear()
    mcp_pins.extend(mcp.get_pin(i) for i in range(16))

    relay_pins.clear()
    for pin_num in (12, 13, 14, 15):
        pin = mcp_pins[pin_num]
        pin.direction = Direction.OUTPUT
        pin.value = False
        relay_pins.append(pin)
        print(f"  ✅ Relay pin {pin_num} initialized (OFF)")

    for pin_num in (8, 9, 10, 11):
        pin = mcp_pins[pin_num]
        pin.direction = Direction.INPUT
        try:
            if DOOR_WIRING == "GND" and DOOR_USE_PULLUP:
                pin.pull = Pull.UP
            else:
                try:
                    pin.pull = None
                except Exception:
                    try:
                        pin.pullup = False
                    except Exception:
                        pass
        except Exception:
            try:
                pin.pullup = (DOOR_WIRING == "GND" and DOOR_USE_PULLUP)
            except Exception:
                pass
        print(f"  ✅ Door switch pin {pin_num} initialized (wiring={DOOR_WIRING}, pullup={'ON' if (DOOR_WIRING=='GND' and DOOR_USE_PULLUP) else 'OFF'})")

    _led_ready_pinobjs = []
    _led_error_pinobjs = []
    n = len(SLOT_IDS)
    for i in range(n):
        pr = _LED_READY_PINS[i] if i < len(_LED_READY_PINS) else None
        pe = _LED_ERROR_PINS[i] if i < len(_LED_ERROR_PINS) else None

        if pr is not None and 0 <= pr < 16:
            rp = mcp_pins[pr]
            rp.direction = Direction.OUTPUT
            _led_write(rp, False)
        else:
            rp = None

        if pe is not None and 0 <= pe < 16:
            ep = mcp_pins[pe]
            ep.direction = Direction.OUTPUT
            _led_write(ep, False)
        else:
            ep = None

        _led_ready_pinobjs.append(rp)
        _led_error_pinobjs.append(ep)

    print(f"✅ MCP23017 initialized: {len(relay_pins)} relays, 4 door switches, {len(_led_ready_pinobjs)} LED-ready, {len(_led_error_pinobjs)} LED-error")

    _refresh_network_state_once()
    _led_boot_default()

    if NET_WATCHDOG:
        _maybe_start_network_watchdog()

# =============================================================================
# XSHUT helpers
# =============================================================================
class _IdxDriver:
    def __init__(self, pins: List[digitalio.DigitalInOut]) -> None:
        self._pins = pins
        for p in self._pins:
            p.switch_to_output(value=False)

    @property
    def pins(self) -> List[int]:
        return list(range(len(self._pins)))

    def all_low(self) -> None:
        for p in self._pins:
            p.value = False

    def one_high(self, index: int) -> None:
        self._pins[index].value = True

    def set_low(self, k: int) -> None:
        self._pins[k].value = False

    def set_high(self, k: int) -> None:
        self._pins[k].value = True

    def all_high(self) -> None:
        for p in self._pins:
            p.value = True

def _make_xshut_driver() -> _IdxDriver:
    return _IdxDriver(XSHUT_PINS)

def init_xshuts() -> None:
    pins_str = ",".join(str(p) for p in _xshut_gpio)
    if BUS_MODE == "mux":
        print("✅ MUX mode: skip XSHUT (handled by TCA9548A)")
        return
    for x in XSHUT_PINS:
        x.switch_to_output(value=False)
    time.sleep(0.3)
    print(f"✅ XSHUT pins initialized (all LOW) → pins={pins_str}")
    print("🔧 INLINE BOOT ACTIVE (multi-device)" if BUS_MODE == "multi" else "🧰 SINGLE-ACTIVE mode")

# =============================================================================
# VL53 INIT + READ
# =============================================================================
def init_sensors() -> None:
    global _vl53_handles, buffers, last_values, _last_read_ts, _sensors_powered

    buffers.clear()
    last_values.clear()
    _last_read_ts.clear()

    init_xshuts()
    if BUS_MODE != "multi":
        print(f"⚠️ BUS_MODE={BUS_MODE} (ชุดนี้รองรับ multi/XSHUT เป็นหลัก)")

    driver = _make_xshut_driver()
    sensor_count = min(len(SLOT_IDS), len(XSHUT_PINS))
    new_addrs = [ADDRESS_BASE + i for i in range(sensor_count)]

    _vl53_handles = init_vl53x_four(
        xshut=driver,
        i2c=shared_i2c,
        new_addrs=new_addrs,
        bus=1,
        timing_budget_us=TIMING_BUDGET_US,
        allow_fallback=VL53_ALLOW_ADAFRUIT,
        extra_boot_delay_s=VL53_BOOT_DELAY_S,
        probe_retries=4,
        probe_interval_s=0.15,
        boot_timeout_s=VL53_BOOT_TIMEOUT_S,
    )

    try:
        driver.all_high()
        print("✅ All VL53L0X sensors are ENABLED (XSHUT=HIGH).")
    except Exception:
        for p in XSHUT_PINS:
            p.value = True

    for _ in range(len(_vl53_handles)):
        buffers.append(deque(maxlen=SMOOTH_WINDOW))
        last_values.append(None)
        _last_read_ts.append(0.0)

    print("VL53 summary:", debug_summary(_vl53_handles))
    print(f"✅ เริ่มต้นเซ็นเซอร์สำเร็จ: {len(_vl53_handles)}/{sensor_count} ตัว (pins={_xshut_gpio})")

    if not _vl53_handles and sensor_count > 0:
        print("⚠️ ยังไม่พบเซ็นเซอร์ → ตรวจสาย/ไฟ/ที่อยู่ I2C")

    _sensors_powered = True

def ensure_sensors_ready() -> None:
    global _sensors_powered
    with _pm_lock:
        if _sensors_powered and _vl53_handles:
            return
        init_sensors()
        _sensors_powered = True

def _apply_outlier_reject(sensor_index: int, mm_value: int) -> int:
    if sensor_index >= len(buffers):
        return mm_value
    buf = buffers[sensor_index]
    if len(buf) >= 3:
        m = statistics.median(buf)
        if abs(mm_value - m) > OUTLIER_MM:
            return int(m + (OUTLIER_MM if mm_value > m else -OUTLIER_MM))
    return mm_value

def _smooth_and_stabilize(sensor_index: int, mm_value: int) -> int:
    if sensor_index >= len(buffers):
        return mm_value
    buffers[sensor_index].append(mm_value)
    stable = int(statistics.median(buffers[sensor_index]))
    if sensor_index >= len(last_values):
        return stable
    if last_values[sensor_index] is None or abs(stable - (last_values[sensor_index] or 0)) >= CHANGE_THRESHOLD:
        last_values[sensor_index] = stable
    return last_values[sensor_index]

def _apply_offset_by_slot_index(sensor_index: int, mm_value: int) -> int:
    try:
        slot_id = SLOT_IDS[sensor_index]
        off = int(_OFFSETS.get(slot_id, 0))
    except Exception:
        off = 0
    return int(mm_value) - off

def _clamp_to_range(mm_value: int) -> int:
    if mm_value < TARGET_MIN_MM:
        return 0
    return int(mm_value)

def reset_sensor_filter(index: int | None = None) -> None:
    if index is None:
        for buf in buffers:
            buf.clear()
        for i in range(len(last_values)):
            last_values[i] = None
        return
    if 0 <= index < len(buffers):
        buffers[index].clear()
    if 0 <= index < len(last_values):
        last_values[index] = None

def read_sensor(sensor_index: int, *, use_filter: bool = True, reset_before: bool = False) -> int:
    ensure_sensors_ready()

    if sensor_index < len(_last_read_ts):
        dt = time.monotonic() - _last_read_ts[sensor_index]
        if dt < INTER_SENSOR_DELAY_S:
            time.sleep(INTER_SENSOR_DELAY_S - dt)

    if reset_before:
        reset_sensor_filter(sensor_index)

    with _handles_guard:
        h_exists = sensor_index in _vl53_handles
    if not h_exists:
        if not _reopen_handle(sensor_index):
            if sensor_index < len(_last_read_ts):
                _last_read_ts[sensor_index] = time.monotonic()
            return -1

    def _read_raw_once() -> int:
        try:
            raw = read_mm(_vl53_handles, sensor_index)
            if raw is None or raw <= 0 or raw > 2000:
                if _reopen_handle(sensor_index):
                    raw = read_mm(_vl53_handles, sensor_index)
            if raw is None or raw <= 0 or raw > 2000:
                return -1
            val = _apply_offset_by_slot_index(sensor_index, int(raw))
            val = _clamp_to_range(val)
            return val
        except Exception:
            return -1

    if not use_filter:
        v = _read_raw_once()
        if sensor_index < len(_last_read_ts):
            _last_read_ts[sensor_index] = time.monotonic()
        return v

    try:
        raw = read_mm(_vl53_handles, sensor_index)
        if raw is None or raw <= 0 or raw > 2000:
            if _reopen_handle(sensor_index):
                raw = read_mm(_vl53_handles, sensor_index)
        if raw is None or raw <= 0 or raw > 2000:
            if sensor_index < len(_last_read_ts):
                _last_read_ts[sensor_index] = time.monotonic()
            return -1

        filtered = _apply_outlier_reject(sensor_index, int(raw))
        stable = _smooth_and_stabilize(sensor_index, filtered)
        stable = _apply_offset_by_slot_index(sensor_index, stable)
        val = _clamp_to_range(stable)

        if sensor_index < len(_last_read_ts):
            _last_read_ts[sensor_index] = time.monotonic()
        return val

    except Exception:
        if sensor_index < len(_last_read_ts):
            _last_read_ts[sensor_index] = time.monotonic()
        return -1

def read_sensor_fresh(sensor_index: int, samples: int = 3, gap_s: float = 0.02) -> int:
    reset_sensor_filter(sensor_index)
    vals = []
    for _ in range(max(1, samples)):
        v = read_sensor(sensor_index, use_filter=False)
        if v != -1:
            vals.append(v)
        time.sleep(gap_s)
    return int(statistics.median(vals)) if vals else -1

# =============================================================================
# Utilities
# =============================================================================
def sensor_addr(index: int) -> Optional[int]:
    h = _vl53_handles.get(index)
    return h.addr if h else None

def vl53_address_map() -> Dict[int, tuple[int, str]]:
    return {i: (h.addr, h.backend) for i, h in _vl53_handles.items()}

def _set_all_xshut_low():
    try:
        for p in XSHUT_PINS:
            p.switch_to_output(value=False)
    except Exception:
        pass

def power_down_sensors() -> None:
    global _sensors_powered, _vl53_handles
    with _pm_lock:
        try:
            for h in list(_vl53_handles.values()):
                dev = getattr(h, "handle", None)
                if dev is None:
                    continue
                for m in ("stop_continuous", "stopContinuous"):
                    if hasattr(dev, m):
                        try:
                            getattr(dev, m)()
                        except Exception:
                            pass
        except Exception:
            pass

        _set_all_xshut_low()

        _vl53_handles.clear()
        buffers.clear()
        last_values.clear()
        _last_read_ts.clear()

        _sensors_powered = False
        print("🔻 VL53 sensors powered down (XSHUT=LOW)")

def diagnose_sensor(index: int, samples: int = 10) -> dict:
    results = []
    failures = 0

    print(f"\n🔍 Diagnosing sensor {index} ({samples} samples)...")

    for i in range(samples):
        try:
            val = read_sensor(index)
            results.append(val)

            if val == -1:
                failures += 1
                print(f"  [{i+1:2d}] ❌ FAIL")
            else:
                print(f"  [{i+1:2d}] ✅ {val:3d}mm")

            time.sleep(0.1)

        except Exception as e:
            failures += 1
            print(f"  [{i+1:2d}] ❌ ERROR: {e}")

    valid = [v for v in results if v != -1]

    report = {
        "sensor_index": index,
        "total_samples": samples,
        "successful": len(valid),
        "failed": failures,
        "success_rate": f"{(len(valid)/samples)*100:.1f}%",
        "values": valid,
    }

    if valid:
        report.update({
            "min": min(valid),
            "max": max(valid),
            "mean": sum(valid) / len(valid),
            "median": statistics.median(valid),
            "std_dev": statistics.stdev(valid) if len(valid) > 1 else 0,
        })

    print(f"\n📊 Summary:")
    print(f"  Success: {report['successful']}/{samples} ({report['success_rate']})")
    if valid:
        print(f"  Range: {report['min']}-{report['max']}mm")
        print(f"  Median: {report['median']:.1f}mm")
        print(f"  Std Dev: {report['std_dev']:.1f}mm")

    return report
