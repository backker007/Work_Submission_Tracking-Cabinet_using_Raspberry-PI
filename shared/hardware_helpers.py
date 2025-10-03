# 🔧 รวมฟังก์ชัน/ตัวช่วยที่เกี่ยวกับ I2C, Servo, VL53L0X, MCP23017
from __future__ import annotations
import os, time, statistics, logging
import board, busio, digitalio
from collections import deque
from digitalio import Direction, Pull
from adafruit_pca9685 import PCA9685
from adafruit_mcp230xx.mcp23017 import MCP23017

# ✅ นำเข้าให้ครบตามที่ใช้งานจริง (ไม่เปลี่ยน logic)
from .vl53l0x_init import (
    init_vl53x_four, read_mm, debug_summary, SensorHandle,
    _wait_for_addr as _vl53_wait_for_addr,
    _raw_set_address_confirm as _vl53_set_addr,
    _open_reader_with_retries as _vl53_open_with_retries,
)
from .topics import SLOT_IDS  # ใช้กำหนดจำนวนเซ็นเซอร์ตามจำนวน slot

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
VL53_BOOT_DELAY_S    = float(os.getenv("VL53_BOOT_DELAY_S", "0.35"))
VL53_BOOT_TIMEOUT_S  = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))
VL53_ADDR_SET_RETRIES= int(os.getenv("VL53_ADDR_SET_RETRIES", "4"))
VL53_ADDR_SET_GAP_S  = float(os.getenv("VL53_ADDR_SET_GAP_S", "0.08"))
VL53_ALLOW_ADAFRUIT  = os.getenv("VL53_ALLOW_ADAFRUIT", "1").lower() in ("1","true","yes")

SMOOTH_WINDOW  = int(os.getenv("VL53_SMOOTH_WINDOW", "5"))
OUTLIER_MM     = int(os.getenv("VL53_OUTLIER_MM", "15"))
CHANGE_THRESHOLD = int(os.getenv("CHANGE_THRESHOLD", "5"))

ADDRESS_BASE = int(os.getenv("VL53_BASE_ADDR", "0x30"), 16)
_xshut_env   = os.getenv("VL53_XSHUT_PINS", "17")
_xshut_gpio  = [int(p.strip()) for p in _xshut_env.split(",") if p.strip()]
XSHUT_PINS   = [digitalio.DigitalInOut(getattr(board, f"D{gpio}")) for gpio in _xshut_gpio]

INIT_ALL_LOW = os.getenv("VL53_INIT_ALL_LOW", "1").lower() in ("1","true","yes")
BUS_MODE     = os.getenv("VL53_BUS_MODE", "multi").strip().lower()

DOOR_SAMPLES = int(os.getenv("DOOR_SAMPLES", "20"))
DOOR_SAMPLE_INTERVAL_S = float(os.getenv("DOOR_SAMPLE_INTERVAL_S", "0.03"))
DOOR_SENSOR_INVERT = os.getenv("DOOR_SENSOR_INVERT", "0").lower() in ("1","true","yes")

print(f"🔌 XSHUT GPIO pins from .env: {_xshut_gpio}")

import threading
try:
    import adafruit_vl53l0x
except Exception:
    adafruit_vl53l0x = None

_handles_guard = threading.RLock()

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
        try: s.measurement_timing_budget = TIMING_BUDGET_US
        except Exception: pass
    for m in ("start_continuous", "startContinuous"):
        if hasattr(s, m):
            try: getattr(s, m)()
            except Exception: pass
            break
    return s

def _expected_addr_for(index: int) -> int:
    return (ADDRESS_BASE + index) & 0x7F

def _recover_index_by_xshut(index: int) -> bool:
    """Power-cycle ผ่าน XSHUT แล้วบังคับตั้งแอดเดรสเป็นค่าเป้าหมาย จากนั้นเปิด reader"""
    if index >= len(XSHUT_PINS):
        log.warning(f"recover: no XSHUT pin for index {index}")
        return False

    target = _expected_addr_for(index)
    try:
        # Power-cycle
        XSHUT_PINS[index].switch_to_output(value=False)
        time.sleep(max(0.05, float(os.getenv("VL53_HOLD_LOW_S", "0.20"))))
        XSHUT_PINS[index].value = True

        boot_to = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))

        # ถ้า default 0x29 โผล่ → ตั้ง addr ใหม่, ไม่งั้นลอง adopt ถ้ามี target อยู่แล้ว
        if not _vl53_wait_for_addr(shared_i2c, 0x29, timeout_s=max(1.0, boot_to)):
            if not _vl53_wait_for_addr(shared_i2c, target, timeout_s=0.6):
                log.warning(f"recover: neither default 0x29 nor target 0x{target:02X} present")
                return False
        else:
            _ok = _vl53_set_addr(
                shared_i2c, target,
                retries=int(os.getenv("VL53_ADDR_SET_RETRIES", "6")),
                gap_s=float(os.getenv("VL53_ADDR_SET_GAP_S", "0.10"))
            )
            if not _ok:
                log.warning("recover: set addr 0x%02X failed", target)
                return False

        # เปิด reader ด้วย retries
        s = _vl53_open_with_retries(
            shared_i2c, target, TIMING_BUDGET_US,
            retries=int(os.getenv("VL53_OPEN_RETRIES", "8")),
            delay_s=float(os.getenv("VL53_OPEN_DELAY_S", "0.30"))
        )
        h = SensorHandle(idx=index, addr=target, backend="adafruit",
                         handle=s, read=_make_read(s))
        with _handles_guard:
            _vl53_handles[index] = h
        log.warning(f"⚠️ Recovered index {index} @0x{target:02X} by XSHUT power-cycle")
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
        log.warning(f"❌ Soft reopen failed @0x{addr:02X}: {e}; trying XSHUT recovery...")
        return _recover_index_by_xshut(index)

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
def _raw_closed(pin) -> bool:
    # Pull.UP: LOW(False)=CLOSED, HIGH(True)=OPEN
    closed = (pin.value is False)
    return (not closed) if DOOR_SENSOR_INVERT else closed

def is_door_reliably_closed(index: int, samples: int = None, interval: float = None) -> bool:
    """Legacy debounce: ต้องอ่าน 'ปิด' ติดกันทุกครั้งถึงจะถือว่าปิดจริง"""
    if samples is None: samples = DOOR_SAMPLES
    if interval is None: interval = DOOR_SAMPLE_INTERVAL_S

    pos = 8 + index
    if pos >= len(mcp_pins):
        log.error(f"Door sensor index {index} is out of range.")
        return False

    pin = mcp_pins[pos]
    for _ in range(max(1, samples)):
        if not _raw_closed(pin):
            return False
        time.sleep(max(0.001, interval))
    return True

def init_mcp():
    """กำหนดรีเลย์/สวิตช์ประตู"""
    global mcp, mcp_pins, relay_pins
    mcp = MCP23017(shared_i2c)
    relay_pin_nums = [12, 13, 14, 15]
    door_switch_pins = [8, 9, 10, 11]

    mcp_pins = [mcp.get_pin(i) for i in range(16)]
    relay_pins.clear()

    for pin_num in relay_pin_nums:
        pin = mcp_pins[pin_num]
        pin.direction = Direction.OUTPUT
        pin.value = False
        relay_pins.append(pin)
        print(f"  ✅ Relay pin {pin_num} initialized (OFF)")

    for pin_num in door_switch_pins:
        pin = mcp_pins[pin_num]
        pin.direction = Direction.INPUT
        pin.pull = Pull.UP
        print(f"  ✅ Door switch pin {pin_num} initialized with Pull-up")

    print(f"✅ MCP23017 initialized: {len(relay_pins)} relays, {len(door_switch_pins)} door switches")

# =============================================================================
# XSHUT helpers
# =============================================================================
def init_xshuts():
    pins_str = ",".join(str(p) for p in _xshut_gpio)
    if BUS_MODE == "mux":
        print("✅ MUX mode: skip XSHUT (handled by TCA9548A)")
        return

    # ดึงลง LOW ทั้งหมด (เตรียม assign address ทีละตัว)
    for x in XSHUT_PINS:
        x.switch_to_output(value=False)
    time.sleep(0.3)
    print(f"✅ XSHUT pins initialized (all LOW) → pins={pins_str}")
    print("🔧 INLINE BOOT ACTIVE (multi-device)" if BUS_MODE == "multi" else "🧰 SINGLE-ACTIVE mode")

class _IdxDriver:
    """ไดรเวอร์ XSHUT แบบเรียบง่ายให้กับ init_vl53x_four ใช้"""
    def __init__(self, pins: list[digitalio.DigitalInOut]):
        self._pins = pins
        for p in self._pins:
            p.switch_to_output(value=False)
    @property
    def pins(self): return list(range(len(self._pins)))
    def all_low(self):  [setattr(p, "value", False) for p in self._pins]
    def one_high(self, index: int): self._pins[index].value = True
    def set_low(self, k: int):  self._pins[k].value = False
    def set_high(self, k: int): self._pins[k].value = True
    def all_high(self): [setattr(p, "value", True) for p in self._pins]

def _make_xshut_driver():
    return _IdxDriver(XSHUT_PINS)

# =============================================================================
# VL53 INIT + READ
# =============================================================================
def init_sensors():
    """
    เริ่มต้น VL53L0X ทั้งหมด:
    - ใช้ XSHUT เปิดทีละตัว ตั้ง address เป็น ADDRESS_BASE + index
    - เปิดใช้งานทุกตัวค้างไว้ (XSHUT = HIGH) เพื่อไม่ให้ address หาย
    - ตั้งค่า timing budget / ช่วงวัด ตาม .env
    """
    global _vl53_handles, buffers, last_values
    buffers.clear(); last_values.clear()

    init_xshuts()
    if BUS_MODE != "multi":
        print(f"⚠️ BUS_MODE={BUS_MODE} (ชุดนี้รองรับ multi/XSHUT เป็นหลัก)")

    driver = _make_xshut_driver()

    # จำนวนจริงที่เราจะตั้ง address = ตามจำนวน slot และจำนวนขา XSHUT
    sensor_count = min(len(SLOT_IDS), len(XSHUT_PINS))
    new_addrs = [ADDRESS_BASE + i for i in range(sensor_count)]

    # ให้ไลบรารีของคุณทำขั้นตอน assign address + เปิดอินสแตนซ์คืนมา
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

    # ❗สำคัญ: เก็บ XSHUT = HIGH ค้างไว้ ไม่ปิดกลับ LOW
    try:
        driver.all_high()
        print("✅ All VL53L0X sensors are ENABLED (XSHUT=HIGH).")
    except Exception:
        for p in XSHUT_PINS: p.value = True

    # เตรียมบัฟเฟอร์กรองสัญญาณ
    for _ in range(len(_vl53_handles)):
        buffers.append(deque(maxlen=SMOOTH_WINDOW))
        last_values.append(None)

    print("VL53 summary:", debug_summary(_vl53_handles))
    print(f"✅ เริ่มต้นเซ็นเซอร์สำเร็จ: {len(_vl53_handles)}/{sensor_count} ตัว (pins={_xshut_gpio})")

    if not _vl53_handles and sensor_count > 0:
        print("⚠️ ยังไม่พบเซ็นเซอร์เลย → ตรวจสอบการเชื่อมต่อ Hardware และสายไฟ")

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
    return last_values[sensor_index]

def _clamp_to_range(mm_value: int) -> int:
    # ถ้าต่ำกว่า TARGET_MIN_MM ให้คืน 0 (ถือว่าแนบชิด/เต็ม)
    # ค่าที่สูงกว่า MAX ไม่ clamp เพราะบางโมดูลต้องใช้จริง (แต่คุณจะ map เป็นเปอร์เซ็นต์ภายหลัง)
    if mm_value < TARGET_MIN_MM:
        return 0
    return int(mm_value)

def read_sensor(sensor_index: int) -> int:
    """
    อ่านค่าแบบคง XSHUT=HIGH; ถ้า handle หายหรืออ่านพัง จะพยายาม reopen 1 ครั้ง
    คืนค่าเป็น mm หรือ -1 เมื่ออ่านไม่ได้
    """
    with _handles_guard:
        h_exists = sensor_index in _vl53_handles
    if not h_exists:
        # พยายามเปิดใหม่ (เช่นกรณีรีเซ็ต/เธรดอื่นยังไม่พร้อม)
        if not _reopen_handle(sensor_index):
            log.warning(f"Sensor index {sensor_index} not initialized (reopen failed)")
            return -1

    try:
        raw = read_mm(_vl53_handles, sensor_index)
        if raw is None or raw <= 0 or raw > 2000:
            # ลอง restart reader แล้วอ่านอีกครั้ง
            if _reopen_handle(sensor_index):
                raw = read_mm(_vl53_handles, sensor_index)
        if raw is None or raw <= 0 or raw > 2000:
            return -1

        filtered = _apply_outlier_reject(sensor_index, int(raw))
        stable = _smooth_and_stabilize(sensor_index, filtered)
        return _clamp_to_range(stable)
    except Exception as e:
        log.error(f"Error reading sensor {sensor_index}: {e}")
        return -1

# Utilities for mapping/diagnostics
def sensor_addr(index: int) -> int | None:
    h = _vl53_handles.get(index)
    return h.addr if h else None

def vl53_address_map() -> dict[int, tuple[int, str]]:
    # ✅ ใช้ _vl53_handles ให้ถูกต้อง
    return {i: (h.addr, h.backend) for i, h in _vl53_handles.items()}
