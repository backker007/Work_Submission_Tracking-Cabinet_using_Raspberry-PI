# shared/hardware_helpers.py
# 🔧 รวมฟังก์ชัน/ตัวช่วยที่เกี่ยวกับ I2C, Servo, VL53L0X, MCP23017
from __future__ import annotations
import os, time, statistics, logging
import board, busio, digitalio
from collections import deque
from digitalio import Direction
from adafruit_pca9685 import PCA9685
from adafruit_mcp230xx.mcp23017 import MCP23017

from .vl53l0x_init import (
    XshutDriver, init_vl53x_four, read_mm, debug_summary, SensorHandle
)

log = logging.getLogger("hw")

# =============================================================================
# I2C & Peripherals
# =============================================================================
shared_i2c = busio.I2C(board.SCL, board.SDA)

# Servo board
pca = PCA9685(shared_i2c)
pca.frequency = 50

# Globals
_vl53_handles: dict[int, SensorHandle] = {}
buffers: list[deque] = []
last_values: list[int | None] = []
mcp = None
mcp_pins: list = []
relay_pins: list = []

# =============================================================================
# ENV Config (ปรับจูนได้จาก .env)
# =============================================================================
TARGET_MIN_MM = int(os.getenv("TARGET_MIN_MM", "20"))
TARGET_MAX_MM = int(os.getenv("TARGET_MAX_MM", "30"))

TIMING_BUDGET_US     = int(os.getenv("VL53_BUDGET_US", "20000"))
VL53_BOOT_DELAY_S    = float(os.getenv("VL53_BOOT_DELAY_S", "0.35"))   # เติมเพิ่มจาก 0.35 ได้
VL53_BOOT_TIMEOUT_S  = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))
VL53_ADDR_SET_RETRIES= int(os.getenv("VL53_ADDR_SET_RETRIES", "4"))
VL53_ADDR_SET_GAP_S  = float(os.getenv("VL53_ADDR_SET_GAP_S", "0.08"))
VL53_ALLOW_ADAFRUIT  = os.getenv("VL53_ALLOW_ADAFRUIT", "1").lower() in ("1","true","yes")

SMOOTH_WINDOW  = int(os.getenv("VL53_SMOOTH_WINDOW", "5"))
OUTLIER_MM     = int(os.getenv("VL53_OUTLIER_MM", "15"))
CHANGE_THRESHOLD = int(os.getenv("CHANGE_THRESHOLD", "5"))

ADDRESS_BASE = int(os.getenv("VL53_BASE_ADDR", "0x30"), 16)            # 0x30..33
_xshut_env   = os.getenv("VL53_XSHUT_PINS", "17,27,22,5")
_xshut_gpio  = [int(p.strip()) for p in _xshut_env.split(",") if p.strip()]
XSHUT_PINS   = [digitalio.DigitalInOut(getattr(board, f"D{gpio}")) for gpio in _xshut_gpio]

INIT_ALL_LOW = os.getenv("VL53_INIT_ALL_LOW", "1").lower() in ("1","true","yes")
BUS_MODE     = os.getenv("VL53_BUS_MODE", "multi").strip().lower()     # multi|mux|single

print(f"🔌 XSHUT GPIO pins from .env: {_xshut_gpio}")

# =============================================================================
# SERVO CONTROL
# =============================================================================
def angle_to_duty_cycle(angle: float) -> int:
    pulse_us = 500 + (angle / 180.0) * 2000
    return int((pulse_us / 20000.0) * 65535)

def move_servo_180(channel: int, angle: int):
    duty_cycle = angle_to_duty_cycle(angle)
    log.debug(f"Servo CH{channel} → {angle}° (duty: {duty_cycle})")
    pca.channels[channel].duty_cycle = duty_cycle
    time.sleep(0.7)
    pca.channels[channel].duty_cycle = 0

# =============================================================================
# DOOR SENSOR (MC-38) via MCP23017
# =============================================================================
def is_door_reliably_closed(index: int, samples=20, interval=0.03) -> bool:
    """True = ปิดสนิท (อ่าน HIGH ทั้งหมด)"""
    results = []
    pin = mcp_pins[8 + index]
    for _ in range(samples):
        results.append(pin.value)
        time.sleep(interval)
    return results.count(False) == 0

def init_mcp():
    """กำหนดรีเลย์/สวิตช์ประตู"""
    global mcp, mcp_pins, relay_pins
    mcp = MCP23017(shared_i2c)
    relay_pin_nums = [12, 13, 14, 15]
    door_switch_pins = [8, 9, 10, 11]

    mcp_pins.clear()
    relay_pins.clear()

    for pin_num in range(16):
        pin = mcp.get_pin(pin_num)
        if pin_num in relay_pin_nums:
            pin.direction = Direction.OUTPUT
            pin.value = False
            relay_pins.append(pin)
            print(f"  ✅ Relay pin {pin_num} initialized (OFF)")
        elif pin_num in door_switch_pins:
            pin.direction = Direction.INPUT
            pin.pull_up = False
            print(f"  ✅ Door switch pin {pin_num} initialized")
        else:
            pin.direction = Direction.OUTPUT
        mcp_pins.append(pin)

    print(f"✅ MCP23017 initialized: {len(relay_pins)} relays, {len(door_switch_pins)} door switches")

# =============================================================================
# XSHUT helpers
# =============================================================================
def init_xshuts():
    pins_str = ",".join(str(p) for p in _xshut_gpio)
    if BUS_MODE == "mux":
        print("✅ MUX mode: skip XSHUT (handled by TCA9548A)")
        return

    for x in XSHUT_PINS:
        x.direction = digitalio.Direction.OUTPUT

    if INIT_ALL_LOW:
        for x in XSHUT_PINS: x.value = False
        time.sleep(0.3)
        print(f"✅ XSHUT pins initialized (all LOW) → pins={pins_str}")
    else:
        for x in XSHUT_PINS: x.value = True
        time.sleep(0.05)
        print(f"✅ XSHUT pins initialized (NO RESET, all HIGH) → pins={pins_str}")

    print("🔧 INLINE BOOT ACTIVE (multi-device)" if BUS_MODE == "multi" else "🧰 SINGLE-ACTIVE mode")

def _make_xshut_driver():
    """Driver แบบ index-based: เปิดเฉพาะตัวที่กำลังตั้ง ไม่ไปยุ่งตัวก่อนหน้า"""
    class _IdxDriver:
        def __init__(self, pins):
            self._pins = pins
            for p in self._pins: p.direction = digitalio.Direction.OUTPUT
        @property
        def pins(self): return list(range(len(self._pins)))
        def all_low(self):  [setattr(p, "value", False) for p in self._pins]
        def one_high(self, index: int): self._pins[index].value = True
        def set_low(self, k: int):  self._pins[k].value = False
        def set_high(self, k: int): self._pins[k].value = True
        def all_high(self): [setattr(p, "value", True) for p in self._pins]
    return _IdxDriver(XSHUT_PINS)

# =============================================================================
# VL53 INIT + READ
# =============================================================================
def init_sensors():
    """เริ่มต้น VL53L0X ทั้งหมด พร้อมตั้ง I2C address 0x30.. ตามลำดับ XSHUT"""
    global _vl53_handles, buffers, last_values
    buffers.clear(); last_values.clear()
    init_xshuts()
    if BUS_MODE != "multi":
        print(f"⚠️ BUS_MODE={BUS_MODE} (ตัวอย่างนี้รองรับ multi/XSHUT เป็นหลัก)")

    driver = _make_xshut_driver()
    new_addrs = [ADDRESS_BASE + i for i in range(len(XSHUT_PINS))]

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

    # คง HIGH ทุกตัว กัน "ลืม addr"
    try: driver.all_high()
    except Exception:
        for p in XSHUT_PINS: p.value = True

    for _ in _vl53_handles:  # init buffers
        buffers.append(deque(maxlen=SMOOTH_WINDOW))
        last_values.append(None)

    print("VL53 summary:", debug_summary(_vl53_handles))
    print(f"✅ เริ่มต้นเซ็นเซอร์สำเร็จ: {len(_vl53_handles)}/{len(_xshut_gpio)} ตัว")

    if not _vl53_handles:
        print("⚠️ ยังไม่พบเซ็นเซอร์เลย → reset I2C แล้วลองใหม่")
        reset_i2c_bus()
        time.sleep(0.3)
        return init_sensors()

def reset_i2c_bus():
    print("🔄 กำลัง reset I2C bus...")
    os.system("sudo i2cdetect -y 1 > /dev/null 2>&1")
    time.sleep(0.5)
    print("✅ I2C bus reset complete")

def _apply_outlier_reject(sensor_index, mm_value):
    if sensor_index >= len(buffers): return mm_value
    buf = buffers[sensor_index]
    if len(buf) >= 3:
        m = statistics.median(buf)
        if abs(mm_value - m) > OUTLIER_MM:
            return int(m + (OUTLIER_MM if mm_value > m else -OUTLIER_MM))
    return mm_value

def _smooth_and_stabilize(sensor_index, mm_value):
    if sensor_index >= len(buffers): return mm_value
    buffers[sensor_index].append(mm_value)
    stable = int(statistics.median(buffers[sensor_index]))
    if sensor_index >= len(last_values): return stable
    if last_values[sensor_index] is None or abs(stable - (last_values[sensor_index] or 0)) >= CHANGE_THRESHOLD:
        last_values[sensor_index] = stable
    return last_values[sensor_index]  # type: ignore

def _clamp_to_range(mm_value: int) -> int:
    if TARGET_MIN_MM <= mm_value <= TARGET_MAX_MM:
        return int(mm_value - ((TARGET_MIN_MM + TARGET_MAX_MM) // 2))
    if mm_value < TARGET_MIN_MM: return 0
    return int(mm_value)

def read_sensor(sensor_index: int) -> int:
    try:
        if sensor_index not in _vl53_handles:
            print(f"⚠️ Sensor index {sensor_index} not initialized")
            return -1
        raw = read_mm(_vl53_handles, sensor_index)
        if raw is None or raw <= 0 or raw > 2000: return -1
        filtered = _apply_outlier_reject(sensor_index, raw)
        stable = _smooth_and_stabilize(sensor_index, filtered)
        return _clamp_to_range(stable)
    except Exception as e:
        print(f"⚠️ Error reading sensor {sensor_index}: {e}")
        return -1

# Utilities for mapping/diagnostics
def sensor_addr(index: int) -> int | None:
    h = _vl53_handles.get(index)
    return h.addr if h else None

def vl53_address_map() -> dict[int, tuple[int, str]]:
    return {i: (h.addr, h.backend) for i, h in _vl53_handles.items()}
