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

# ---- HW imports (Blinka) ----------------------------------------------------
import board
import busio
import digitalio
from digitalio import Direction, Pull
from adafruit_pca9685 import PCA9685
from adafruit_mcp230xx.mcp23017 import MCP23017

# NOTE: relative import because this file is under 'shared'
from .vl53l0x_init import (
    init_vl53x_four,
    read_mm,
    debug_summary,
    SensorHandle,
    _wait_for_addr as _vl53_wait_for_addr,
    _raw_set_address_confirm as _vl53_set_addr,
    _open_reader_with_retries as _vl53_open_with_retries,
)
from .topics import SLOT_IDS  # จำนวนช่องใช้งาน

log = logging.getLogger("hw")

# =============================================================================
# I2C & Peripherals
# =============================================================================
shared_i2c = busio.I2C(board.SCL, board.SDA)

# --- PCA9685 (Servo) ---
pca = PCA9685(shared_i2c)
pca.frequency = 50  # SG90/STD servo 50Hz

# --- MCP23017 (รีเลย์ / สวิทช์ประตู / LED) ---
mcp: Optional[MCP23017] = None
mcp_pins: List = []     # index 0..15 -> Pin objects
relay_pins: List = []   # เก็บเฉพาะพินที่เป็นรีเลย์

# =============================================================================
# ENV Config (จูนได้จาก .env)
# =============================================================================
# ระยะใช้งานจริงของตู้ (ค่า default จูนให้ตรง use-case; ปรับได้ใน .env)
TARGET_MIN_MM = int(os.getenv("TARGET_MIN_MM", "80"))   # <80 => ตีเป็น 'แนบชิด/เต็ม' = 0
TARGET_MAX_MM = int(os.getenv("TARGET_MAX_MM", "300"))

# ✅ FIX: เพิ่ม timing budget เป็น 60ms (จาก 20ms) เพื่อความแม่นยำสูงขึ้น
TIMING_BUDGET_US = int(os.getenv("VL53_BUDGET_US", "60000"))  # เดิม 20000
VL53_BOOT_DELAY_S = float(os.getenv("VL53_BOOT_DELAY_S", "0.35"))
VL53_BOOT_TIMEOUT_S = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))
VL53_ADDR_SET_RETRIES = int(os.getenv("VL53_ADDR_SET_RETRIES", "4"))
VL53_ADDR_SET_GAP_S = float(os.getenv("VL53_ADDR_SET_GAP_S", "0.08"))
VL53_ALLOW_ADAFRUIT = os.getenv("VL53_ALLOW_ADAFRUIT", "1").lower() in ("1", "true", "yes")

# ฟิลเตอร์
SMOOTH_WINDOW = int(os.getenv("VL53_SMOOTH_WINDOW", "5"))
OUTLIER_MM = int(os.getenv("VL53_OUTLIER_MM", "15"))

# ✅ FIX: ลด threshold เหลือ 3mm (จาก 5mm) เพื่อให้ค่าเปลี่ยนเร็วขึ้น
CHANGE_THRESHOLD = int(os.getenv("CHANGE_THRESHOLD", "3"))  # เดิม 5

# โหมดอ่านต่อเนื่อง / ดีเลย์ระหว่างเซ็นเซอร์
VL53_CONTINUOUS = os.getenv("VL53_CONTINUOUS", "0").lower() in ("1", "true", "yes")

# ✅ FIX: เพิ่ม inter-sensor delay ให้เท่ากับ timing budget (60ms)
INTER_SENSOR_DELAY_S = float(os.getenv("VL53_INTER_DELAY_S", "0.070"))  # เดิม 0.015

ADDRESS_BASE = int(os.getenv("VL53_BASE_ADDR", "0x30"), 16)

# ออฟเซ็ตต่อช่อง (JSON จาก .env) เช่น {"SC001":47,"SC002":0,"SC003":68,"SC004":25}
try:
    _OFFSETS = json.loads(os.getenv("VL53_OFFSETS", "{}"))
except Exception:
    _OFFSETS = {}

# XSHUT GPIO พิมพ์เลขขา GPIO ตามบอร์ด (เช่น 17 หมายถึง board.D17)
_xshut_env = os.getenv("VL53_XSHUT_PINS", "17")
_xshut_gpio = [int(p.strip()) for p in _xshut_env.split(",") if p.strip()]
XSHUT_PINS = [digitalio.DigitalInOut(getattr(board, f"D{gpio}")) for gpio in _xshut_gpio]

INIT_ALL_LOW = os.getenv("VL53_INIT_ALL_LOW", "1").lower() in ("1", "true", "yes")
BUS_MODE = os.getenv("VL53_BUS_MODE", "multi").strip().lower()  # multi | mux | single

DOOR_SAMPLES = int(os.getenv("DOOR_SAMPLES", "20"))
DOOR_SAMPLE_INTERVAL_S = float(os.getenv("DOOR_SAMPLE_INTERVAL_S", "0.03"))

# การอ่านค่าประตู (เดิม): ปิด=LOW (GND wiring + internal pull-up)
# เพิ่มโหมด VCC wiring: ปิด=HIGH (ต้องมี external pull-down)
DOOR_WIRING = os.getenv("DOOR_WIRING", "GND").strip().upper()  # "GND" | "VCC"
DOOR_USE_PULLUP = os.getenv("DOOR_USE_PULLUP", "1").lower() in ("1", "true", "yes")
DOOR_SENSOR_INVERT = os.getenv("DOOR_SENSOR_INVERT", "0").lower() in ("1", "true", "yes")

# Servo dwell / auto-off
SERVO_SETTLE_S = float(os.getenv("SERVO_SETTLE_S", "0.7"))  # เวลารอให้หมุนจบก่อนตัด PWM

# ===== Distance-based Slot 'Full' detection =====
DEFAULT_FULL_THRESHOLD_MM = int(os.getenv("DEFAULT_FULL_THRESHOLD_MM", "40"))
SLOT_FULL_THRESHOLD_MM = {sid: DEFAULT_FULL_THRESHOLD_MM for sid in SLOT_IDS}

def is_slot_full(slot_id: str, distance_mm: Optional[float]) -> bool:
    """True ถ้าวัดได้ใกล้กว่า threshold ของช่องนั้น (ถือว่า 'เต็ม')."""
    if distance_mm is None:
        return False
    thr = SLOT_FULL_THRESHOLD_MM.get(slot_id, DEFAULT_FULL_THRESHOLD_MM)
    return distance_mm <= thr

# ===== Network watchdog / LED policy =====
NET_WATCHDOG = os.getenv("NET_WATCHDOG", "1").lower() in ("1", "true", "yes")
NET_CHECK_HOST = os.getenv("NET_CHECK_HOST") or os.getenv("MQTT_HOST") or "8.8.8.8"
NET_CHECK_PORT = int(os.getenv("NET_CHECK_PORT") or os.getenv("MQTT_PORT") or "53")
NET_CHECK_INTERVAL_S = float(os.getenv("NET_CHECK_INTERVAL_S", "3.0"))
NET_CONNECT_TIMEOUT_S = float(os.getenv("NET_CONNECT_TIMEOUT_S", "1.2"))
LED_BOOT_DEFAULT = os.getenv("LED_BOOT_DEFAULT", "ready").strip().lower()  # ready|error|off
LED_ONLINE_DEFAULT_READY = os.getenv("LED_ONLINE_DEFAULT_READY", "1").lower() in ("1", "true", "yes")
NET_OFFLINE_BLINK = os.getenv("NET_OFFLINE_BLINK", "1").lower() in ("1", "true", "yes")
NET_OFFLINE_BLINK_S = float(os.getenv("NET_OFFLINE_BLINK_S", "0.6"))

# ✅ NEW: Health check configuration
HEALTH_CHECK_ENABLED = os.getenv("VL53_HEALTH_CHECK", "1").lower() in ("1", "true", "yes")
HEALTH_CHECK_INTERVAL_S = float(os.getenv("VL53_HEALTH_CHECK_INTERVAL_S", "60.0"))
HEALTH_CHECK_TIMEOUT_S = float(os.getenv("VL53_HEALTH_CHECK_TIMEOUT_S", "2.0"))

# ✅ NEW: Cross-talk mitigation
XSHUT_SEQUENTIAL_READ = os.getenv("VL53_XSHUT_SEQUENTIAL", "1").lower() in ("1", "true", "yes")
XSHUT_OFF_DELAY_MS = float(os.getenv("VL53_XSHUT_OFF_DELAY_MS", "5.0"))  # หน่วง ms ก่อนปิด XSHUT

print(f"🔌 XSHUT GPIO pins from .env: {_xshut_gpio}")
print(f"⚙️  TIMING_BUDGET_US: {TIMING_BUDGET_US} µs")
print(f"⚙️  INTER_SENSOR_DELAY_S: {INTER_SENSOR_DELAY_S} s")
print(f"⚙️  CHANGE_THRESHOLD: {CHANGE_THRESHOLD} mm")
print(f"⚙️  XSHUT_SEQUENTIAL_READ: {XSHUT_SEQUENTIAL_READ}")

# =============================================================================
# VL53L0X Backends
# =============================================================================
_handles_guard = threading.RLock()
_vl53_handles: Dict[int, SensorHandle] = {}  # index -> handle
buffers: List[deque] = []
last_values: List[Optional[int]] = []
_last_read_ts: List[float] = []  # สำหรับ inter-sensor delay

# ✅ NEW: Fail counter สำหรับ auto-recovery
_sensor_fail_counts: Dict[int, int] = {}
SENSOR_FAIL_THRESHOLD = int(os.getenv("SENSOR_FAIL_THRESHOLD", "3"))

# ✅ NEW: Recovery lock เพื่อป้องกัน recovery พร้อมกัน
_recovery_locks: Dict[int, threading.Lock] = {}

try:
    import adafruit_vl53l0x  # type: ignore
except Exception:
    adafruit_vl53l0x = None

def _make_read(sensor):
    """Wrap .range เป็นฟังก์ชันอ่านค่าแบบ int|None เพื่อใช้กับ SensorHandle."""
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
    """เปิดออบเจกต์ VL53L0X ที่ address ระบุ (ใช้ไลบรารี Adafruit)."""
    if adafruit_vl53l0x is None:
        raise RuntimeError("adafruit_vl53l0x not installed")
    s = adafruit_vl53l0x.VL53L0X(shared_i2c, address=addr)
    if hasattr(s, "measurement_timing_budget"):
        try:
            s.measurement_timing_budget = TIMING_BUDGET_US
        except Exception:
            pass
    # เริ่ม continuous เฉพาะเมื่อเปิดใช้ผ่าน .env
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

# ✅ NEW: ปรับปรุง recovery mechanism
def _recover_index_by_xshut(index: int) -> bool:
    """
    Power-cycle ผ่าน XSHUT แล้วพยายามตั้ง address เป้าหมาย → เปิด reader สำเร็จคืน True
    ✅ เพิ่ม: Thread-safe, retry logic, logging
    """
    if index >= len(XSHUT_PINS):
        log.warning(f"recover: no XSHUT pin for index {index}")
        return False

    # ป้องกัน concurrent recovery
    if index not in _recovery_locks:
        _recovery_locks[index] = threading.Lock()
    
    if not _recovery_locks[index].acquire(blocking=False):
        log.warning(f"recover: index {index} already in recovery, skipping")
        return False

    try:
        target = _expected_addr_for(index)
        log.warning(f"🔄 Starting recovery for sensor index {index} (target=0x{target:02X})")
        
        # 1) Power-cycle
        XSHUT_PINS[index].switch_to_output(value=False)
        time.sleep(max(0.05, float(os.getenv("VL53_HOLD_LOW_S", "0.20"))))
        XSHUT_PINS[index].value = True

        boot_to = float(os.getenv("VL53_BOOT_TIMEOUT_S", "1.2"))

        # 2) set addr ถ้าเห็น default, ไม่งั้น adopt ถ้าเห็น target อยู่แล้ว
        if not _vl53_wait_for_addr(shared_i2c, 0x29, timeout_s=max(1.0, boot_to)):
            if not _vl53_wait_for_addr(shared_i2c, target, timeout_s=0.6):
                log.warning(f"recover: neither 0x29 nor target 0x{target:02X} present")
                return False
            log.info(f"recover: adopting existing address 0x{target:02X}")
        else:
            _ok = _vl53_set_addr(
                shared_i2c, target,
                retries=int(os.getenv("VL53_ADDR_SET_RETRIES", "6")),
                gap_s=float(os.getenv("VL53_ADDR_SET_GAP_S", "0.10")),
            )
            if not _ok:
                log.warning("recover: set addr 0x%02X failed", target)
                return False

        # 3) open with retries
        s = _vl53_open_with_retries(
            shared_i2c, target, TIMING_BUDGET_US,
            retries=int(os.getenv("VL53_OPEN_RETRIES", "8")),
            delay_s=float(os.getenv("VL53_OPEN_DELAY_S", "0.30")),
        )
        h = SensorHandle(idx=index, addr=target, backend="adafruit",
                         handle=s, read=_make_read(s))
        with _handles_guard:
            _vl53_handles[index] = h
        
        # รีเซ็ต fail counter
        _sensor_fail_counts[index] = 0
        
        log.warning(f"✅ Recovered index {index} @0x{target:02X} by XSHUT")
        return True

    except Exception as e:
        log.error(f"❌ recover index {index} failed: {e}")
        return False
    finally:
        _recovery_locks[index].release()

def _reopen_handle(index: int) -> bool:
    """พยายามเปิด reader ใหม่แบบ soft ก่อน ถ้าไม่ได้ค่อย XSHUT recovery."""
    addr = _expected_addr_for(index)
    try:
        s = _open_reader(addr)
        h = SensorHandle(idx=index, addr=addr, backend="adafruit", handle=s, read=_make_read(s))
        with _handles_guard:
            _vl53_handles[index] = h
        _sensor_fail_counts[index] = 0
        log.warning(f"⚠️ Reopened VL53L0X index {index} @0x{addr:02X}")
        return True
    except Exception as e:
        log.warning(f"❌ Soft reopen failed @0x{addr:02X}: {e}; try XSHUT...")
        return _recover_index_by_xshut(index)

# ✅ NEW: Auto-recovery wrapper
def _auto_recover_if_needed(index: int) -> bool:
    """
    ตรวจสอบ fail counter และทำ auto-recovery ถ้าจำเป็น
    คืน True ถ้าฟื้นสำเร็จหรือยังไม่ถึง threshold
    """
    if index not in _sensor_fail_counts:
        _sensor_fail_counts[index] = 0
    
    if _sensor_fail_counts[index] < SENSOR_FAIL_THRESHOLD:
        return True
    
    log.warning(f"⚠️ Sensor {index} failed {SENSOR_FAIL_THRESHOLD} times, attempting auto-recovery...")
    
    if _reopen_handle(index):
        log.info(f"✅ Auto-recovery successful for sensor {index}")
        return True
    else:
        log.error(f"❌ Auto-recovery failed for sensor {index}")
        return False

# =============================================================================
# SERVO CONTROL
# =============================================================================
def angle_to_duty_cycle(angle: float) -> int:
    """แปลงองศา 0..180 → duty_cycle (0..65535) สำหรับ PCA9685 ที่ 50Hz."""
    pulse_us = 500 + (angle / 180.0) * 2000  # 0°≈500µs, 180°≈2500µs
    return int((pulse_us / 20000.0) * 65535)  # 20ms period @50Hz

def move_servo_180(channel: int, angle: int) -> None:
    """ขยับเซอร์โวแล้วตัด PWM เพื่อลดความร้อน (hold by gear)."""
    angle = max(0, min(180, int(angle)))

    # 🔁 กลับทิศทาง (สลับ 0↔180) ให้เข้ากับกลไกที่ติดตั้ง
    angle = 180 - angle

    duty = angle_to_duty_cycle(angle)
    log.debug(f"Servo CH{channel} → {angle}° (duty={duty})")
    pca.channels[channel].duty_cycle = duty
    time.sleep(max(0.2, SERVO_SETTLE_S))
    pca.channels[channel].duty_cycle = 0

# =============================================================================
# DOOR SENSOR (MC-38) via MCP23017
# =============================================================================
def _raw_closed(pin) -> bool:
    """
    อ่านสถานะ 'ปิด' ของสวิตช์ประตูตามรูปแบบเดินสาย
    - โหมด GND (ค่าเดิม): ใช้ Pull-up ภายใน → ปิด=LOW(False)
    - โหมด VCC: ต้องมี external pull-down → ปิด=HIGH(True)
    DOOR_SENSOR_INVERT=1 จะกลับด้านผลลัพธ์อีกชั้น
    """
    if DOOR_WIRING == "VCC":
        closed = (pin.value is True)   # ปิด=HIGH
    else:
        closed = (pin.value is False)  # ปิด=LOW (GND wiring + pull-up)
    return (not closed) if DOOR_SENSOR_INVERT else closed

def is_door_reliably_closed(index: int, samples: Optional[int] = None,
                            interval: Optional[float] = None) -> bool:
    """
    Legacy debounce: ต้องอ่าน 'ปิด' ติดกันทุกครั้งถึงจะถือว่าปิดจริง
    door switch ใช้ที่พิน 8..11 ตามดีไซน์เริ่มต้น
    """
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
# LED helpers (READY/ERROR) on MCP23017
# =============================================================================
# ปักหมุดพินผ่าน .env (คอมมาแยกรายการตามจำนวนช่อง), ค่าเริ่มต้น: READY=0..3, ERROR=4..7
_LED_READY_PINS = [int(x) for x in os.getenv("SLOT_LED_READY_PINS", "0,1,2,3").split(",") if x.strip()]
_LED_ERROR_PINS = [int(x) for x in os.getenv("SLOT_LED_ERROR_PINS", "4,5,6,7").split(",") if x.strip()]
_LED_ACTIVE_HIGH = os.getenv("SLOT_LED_ACTIVE_HIGH", "1").lower() in ("1", "true", "yes")

_led_ready_pinobjs: List = []  # list of MCP pins for ready (by index)
_led_error_pinobjs: List = []  # list of MCP pins for error (by index)

def _led_write(pin, on: bool) -> None:
    """เขียนค่าสำหรับ LED โดยคำนึง active-high/low."""
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

def set_slot_led_ready(mcp_obj: Optional[MCP23017], index: int) -> None:
    """
    เปิดไฟ 'พร้อมใช้งาน' ของช่อง index และปิดไฟ 'ผิดพลาด'
    หมายเหตุ: ถ้า watchdog จับว่าออฟไลน์ ไฟจะถูก override เป็น Error
    """
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
    """เปิดไฟ 'ผิดพลาด' และปิดไฟ 'พร้อมใช้งาน' ของช่อง index."""
    try:
        pin_ready = _led_ready_pinobjs[index] if index < len(_led_ready_pinobjs) else None
        pin_error = _led_error_pinobjs[index] if index < len(_led_error_pinobjs) else None
        _led_write(pin_ready, False)
        _led_write(pin_error, True)
    except Exception as e:
        log.debug(f"set_slot_led_error({index}) ignored: {e}")

def _led_boot_default() -> None:
    """
    กำหนดสถานะไฟทันทีหลังบูต:
      - ถ้าออนไลน์: ทำตาม LED_BOOT_DEFAULT (ready|error|off)
      - ถ้าออฟไลน์: ไฟแดง (หรือปล่อยให้ watchdog กระพริบ)
    """
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
# Network/MQTT watchdog (ภายใน hardware_helpers)
# =============================================================================
_network_online = True
_net_thread_started = False

def internet_ok() -> bool:
    """คืน True ถ้าตรวจเจอว่าออนไลน์ (โฮสต์/พอร์ตปลายทางต่อได้)."""
    return _network_online

def _tcp_connect_ok(host: str, port: int, timeout: float) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

def _apply_network_led_state(online: bool) -> None:
    """บังคับ LED รวมทั้งตู้เมื่อสถานะเน็ตเปลี่ยน."""
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
        # กระพริบไฟแดงเมื่อออฟไลน์
        if blink:
            _set_all_leds(False, True)   # ON
            time.sleep(NET_OFFLINE_BLINK_S)
            _set_all_leds(False, False)  # OFF
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
    """ตรวจครั้งเดียว เพื่อให้สถานะเริ่มต้นตรงความจริงก่อนสตาร์ทเธรด."""
    global _network_online
    ok = _tcp_connect_ok(NET_CHECK_HOST, NET_CHECK_PORT, NET_CONNECT_TIMEOUT_S)
    _network_online = ok
    log.info("🌐 Network initial: %s (%s:%s)", "ONLINE" if ok else "OFFLINE", NET_CHECK_HOST, NET_CHECK_PORT)

# =============================================================================
# ✅ NEW: Health Check Thread
# =============================================================================
_health_thread_started = False

def _health_check_loop():
    """
    เธรดตรวจสอบสุขภาพของ sensor ทั้งหมดเป็นระยะ
    - อ่านค่าทดสอบแต่ละตัว
    - ถ้าอ่านไม่ได้ติดต่อกันหลายครั้ง → trigger recovery
    """
    log.info("✅ Health check thread started (interval=%ds)", HEALTH_CHECK_INTERVAL_S)
    
    while True:
        time.sleep(HEALTH_CHECK_INTERVAL_S)
        
        for index in range(len(_vl53_handles)):
            try:
                # ทดสอบอ่านค่า
                test_val = read_mm(_vl53_handles, index)
                
                if test_val is None or test_val <= 0 or test_val > 2000:
                    _sensor_fail_counts[index] = _sensor_fail_counts.get(index, 0) + 1
                    log.warning(f"Health check: sensor {index} returned invalid value (fail_count={_sensor_fail_counts[index]})")
                    
                    # ถ้าเกิน threshold → recovery
                    if _sensor_fail_counts[index] >= SENSOR_FAIL_THRESHOLD:
                        log.warning(f"🔧 Health check triggering recovery for sensor {index}")
                        _auto_recover_if_needed(index)
                else:
                    # อ่านได้ปกติ → รีเซ็ต counter
                    _sensor_fail_counts[index] = 0
                    
            except Exception as e:
                log.error(f"Health check error for sensor {index}: {e}")
                _sensor_fail_counts[index] = _sensor_fail_counts.get(index, 0) + 1

def _maybe_start_health_check():
    global _health_thread_started
    if _health_thread_started or not HEALTH_CHECK_ENABLED:
        return
    t = threading.Thread(target=_health_check_loop, daemon=True, name="health-check")
    t.start()
    _health_thread_started = True
    log.info("🏥 Health check monitoring enabled")

# =============================================================================
# MCP23017 init
# =============================================================================
def init_mcp() -> None:
    """
    สร้างออบเจกต์ MCP23017 และตั้งพินใช้งาน
    - รีเลย์: 12, 13, 14, 15 (OUTPUT, OFF)
    - สวิทช์ประตู: 8, 9, 10, 11 (INPUT, Pull ตาม DOOR_WIRING/DOOR_USE_PULLUP)
    - LED: จาก .env SLOT_LED_READY_PINS / SLOT_LED_ERROR_PINS (OUTPUT, OFF)
    """
    global mcp, mcp_pins, relay_pins, _led_ready_pinobjs, _led_error_pinobjs

    mcp = MCP23017(shared_i2c)

    # เตรียมพินทั้งหมด
    mcp_pins.clear()
    mcp_pins.extend(mcp.get_pin(i) for i in range(16))

    # รีเลย์ 4 ช่อง (12..15)
    relay_pins.clear()
    for pin_num in (12, 13, 14, 15):
        pin = mcp_pins[pin_num]
        pin.direction = Direction.OUTPUT
        pin.value = False
        relay_pins.append(pin)
        print(f"  ✅ Relay pin {pin_num} initialized (OFF)")

    # Door switches (8..11) → INPUT + (Pull ตาม config)
    for pin_num in (8, 9, 10, 11):
        pin = mcp_pins[pin_num]
        pin.direction = Direction.INPUT
        # ตั้งค่า Pull:
        # - ถ้า DOOR_WIRING=GND และเปิดใช้ pull-up → เปิด Pull.UP ภายใน
        # - ถ้า DOOR_WIRING=VCC → ปิด pull-up ภายใน (ต้องมี external pull-down)
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
            # fallback ไลบรารีรุ่นเก่า
            try:
                pin.pullup = (DOOR_WIRING == "GND" and DOOR_USE_PULLUP)
            except Exception:
                pass
        print(f"  ✅ Door switch pin {pin_num} initialized (wiring={DOOR_WIRING}, pullup={'ON' if (DOOR_WIRING=='GND' and DOOR_USE_PULLUP) else 'OFF'})")

    # Status LEDs (READY/ERROR) -> OUTPUT, OFF
    _led_ready_pinobjs = []
    _led_error_pinobjs = []
    # map ตามจำนวนช่องใช้งานจริง
    n = len(SLOT_IDS)
    for i in range(n):
        # fallback: ถ้าพินน้อยกว่า index จะใส่ None
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

    # ===== อัปเดตสถานะเน็ตครั้งแรก + ตั้งไฟเริ่มต้นหลังบูต =====
    _refresh_network_state_once()
    _led_boot_default()

    # ===== เริ่ม network watchdog (จะคุมไฟเมื่อออฟไลน์) =====
    if NET_WATCHDOG:
        _maybe_start_network_watchdog()

# =============================================================================
# XSHUT helpers
# =============================================================================
class _IdxDriver:
    """ไดรเวอร์ XSHUT แบบเรียบง่าย ให้ init_vl53x_four ใช้งาน."""
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
    """เตรียม XSHUT ทั้งหมดเป็น LOW (multi-device) หรือข้ามเมื่อใช้ MUX)."""
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
    """
    เริ่มต้น VL53L0X ทั้งหมด:
    - เปิด XSHUT ทีละตัว → ตั้ง address = ADDRESS_BASE + index
    - เปิดทุกตัวค้างไว้ (XSHUT=HIGH) เพื่อให้ address ไม่หาย
    - ตั้ง timing budget ตาม .env
    """
    global _vl53_handles, buffers, last_values, _last_read_ts, _sensor_fail_counts

    buffers.clear()
    last_values.clear()
    _last_read_ts.clear()
    _sensor_fail_counts.clear()

    init_xshuts()
    if BUS_MODE != "multi":
        print(f"⚠️ BUS_MODE={BUS_MODE} (ชุดนี้รองรับ multi/XSHUT เป็นหลัก)")

    driver = _make_xshut_driver()

    # จำนวนจริงตาม SLOT_IDS และจำนวน XSHUT
    sensor_count = min(len(SLOT_IDS), len(XSHUT_PINS))
    new_addrs = [ADDRESS_BASE + i for i in range(sensor_count)]

    # ไลบรารีส่วนกลาง: assign address + เปิด instance คืนมา
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

    # ค้าง XSHUT=HIGH
    try:
        driver.all_high()
        print("✅ All VL53L0X sensors are ENABLED (XSHUT=HIGH).")
    except Exception:
        for p in XSHUT_PINS:
            p.value = True

    # เตรียมบัฟเฟอร์กรอง + ประทับเวลาอ่าน (สำหรับ inter-sensor delay)
    for i in range(len(_vl53_handles)):
        buffers.append(deque(maxlen=SMOOTH_WINDOW))
        last_values.append(None)
        _last_read_ts.append(0.0)
        _sensor_fail_counts[i] = 0
        _recovery_locks[i] = threading.Lock()

    print("VL53 summary:", debug_summary(_vl53_handles))
    print(f"✅ เริ่มต้นเซ็นเซอร์สำเร็จ: {len(_vl53_handles)}/{sensor_count} ตัว (pins={_xshut_gpio})")

    if not _vl53_handles and sensor_count > 0:
        print("⚠️ ยังไม่พบเซ็นเซอร์ → ตรวจสาย/ไฟ/ที่อยู่ I2C")
    
    # ✅ เริ่ม health check thread
    _maybe_start_health_check()

def _apply_outlier_reject(sensor_index: int, mm_value: int) -> int:
    """ปัด outlier เทียบ median ล่าสุดในบัฟเฟอร์ (±OUTLIER_MM)."""
    if sensor_index >= len(buffers):
        return mm_value
    buf = buffers[sensor_index]
    if len(buf) >= 3:
        m = statistics.median(buf)
        if abs(mm_value - m) > OUTLIER_MM:
            return int(m + (OUTLIER_MM if mm_value > m else -OUTLIER_MM))
    return mm_value

# ✅ FIX: ลด hysteresis (optional: สามารถปิดได้ด้วยการ comment if statement)
def _smooth_and_stabilize(sensor_index: int, mm_value: int) -> int:
    """
    median smoothing + state hold (เปลี่ยนเมื่อ Δ ≥ CHANGE_THRESHOLD)
    ✅ ปรับ: ลด threshold เหลือ 3mm และเพิ่มโหมด "no-hysteresis" (ถ้าต้องการ real-time)
    """
    if sensor_index >= len(buffers):
        return mm_value
    buffers[sensor_index].append(mm_value)
    stable = int(statistics.median(buffers[sensor_index]))
    
    if sensor_index >= len(last_values):
        return stable
    
    # ✅ ถ้าต้องการ real-time (ไม่มี hysteresis) ให้ uncomment บรรทัดนี้:
    # last_values[sensor_index] = stable
    # return stable
    
    # ✅ โหมด hysteresis (threshold = 3mm)
    if last_values[sensor_index] is None or abs(stable - (last_values[sensor_index] or 0)) >= CHANGE_THRESHOLD:
        last_values[sensor_index] = stable
    return last_values[sensor_index]

def _apply_offset_by_slot_index(sensor_index: int, mm_value: int) -> int:
    """หักออฟเซ็ตตาม slot_id จาก .env (VL53_OFFSETS)"""
    try:
        slot_id = SLOT_IDS[sensor_index]
        off = int(_OFFSETS.get(slot_id, 0))
    except Exception:
        off = 0
    return int(mm_value) - off

def _clamp_to_range(mm_value: int) -> int:
    """
    ถ้าต่ำกว่า TARGET_MIN_MM คืน 0 (ถือว่าแนบชิด/เต็ม)
    ค่าสูงกว่า MAX ไม่ clamp (ไว้ไป map % ด้านนอก)
    """
    if mm_value < TARGET_MIN_MM:
        return 0
    return int(mm_value)

# ✅ NEW: Sequential read with XSHUT control
def read_sensor_sequential(sensor_index: int) -> int:
    """
    อ่านแบบ Sequential: ปิด XSHUT อื่นทั้งหมดก่อนอ่าน เพื่อลด cross-talk
    ⚠️ ช้ากว่าแบบปกติ แต่แม่นยำกว่า
    """
    if not XSHUT_SEQUENTIAL_READ:
        return read_sensor(sensor_index)
    
    try:
        # 1) ปิด XSHUT อื่นทั้งหมด
        for i, pin in enumerate(XSHUT_PINS):
            if i != sensor_index:
                pin.value = False
        
        time.sleep(XSHUT_OFF_DELAY_MS / 1000.0)
        
        # 2) อ่านเฉพาะตัวที่ต้องการ
        result = read_sensor(sensor_index)
        
        # 3) เปิด XSHUT กลับคืน
        for pin in XSHUT_PINS:
            pin.value = True
        
        time.sleep(0.01)  # รอให้ sensor กลับมา
        
        return result
        
    except Exception as e:
        log.error(f"Sequential read error for sensor {sensor_index}: {e}")
        # กู้คืน XSHUT state
        for pin in XSHUT_PINS:
            try:
                pin.value = True
            except:
                pass
        return -1

# Global flag สำหรับควบคุม recovery
_disable_recovery_during_door_operation = False

def set_recovery_enabled(enabled: bool) -> None:
    """
    เปิด/ปิด auto-recovery ชั่วคราว
    ใช้ตอนเปิดประตูเพื่อป้องกัน recovery block ลูปหลัก
    """
    global _disable_recovery_during_door_operation
    _disable_recovery_during_door_operation = not enabled
    if not enabled:
        log.debug("🔒 Auto-recovery disabled temporarily")
    else:
        log.debug("🔓 Auto-recovery enabled")


def read_sensor(sensor_index: int) -> int:
    """
    🔧 FIX: เพิ่ม retry mechanism + timeout protection
    """
    global _disable_recovery_during_door_operation
    
    # Inter-sensor delay
    if sensor_index < len(_last_read_ts):
        dt = time.monotonic() - _last_read_ts[sensor_index]
        if dt < INTER_SENSOR_DELAY_S:
            time.sleep(INTER_SENSOR_DELAY_S - dt)

    with _handles_guard:
        h_exists = sensor_index in _vl53_handles
    
    if not h_exists:
        if _disable_recovery_during_door_operation:
            log.debug(f"Sensor {sensor_index} not init (recovery disabled)")
            return -1
        
        if not _reopen_handle(sensor_index):
            log.warning(f"Sensor index {sensor_index} not initialized")
            return -1

    # 🔧 FIX: เพิ่ม retry loop (3 ครั้ง) ก่อน fail
    MAX_RETRIES = 3
    for attempt in range(MAX_RETRIES):
        try:
            raw = read_mm(_vl53_handles, sensor_index)
            
            # ตรวจสอบค่าที่ได้
            if raw is not None and raw > 0 and raw <= 2000:
                # ✅ อ่านสำเร็จ → รีเซ็ต counter
                _sensor_fail_counts[sensor_index] = 0
                
                # Apply filters
                filtered = _apply_outlier_reject(sensor_index, int(raw))
                stable = _smooth_and_stabilize(sensor_index, filtered)
                stable = _apply_offset_by_slot_index(sensor_index, stable)
                val = _clamp_to_range(stable)
                
                if sensor_index < len(_last_read_ts):
                    _last_read_ts[sensor_index] = time.monotonic()
                
                return val
            
            # ค่าไม่ OK → retry
            if attempt < MAX_RETRIES - 1:
                log.debug(f"Sensor {sensor_index} retry {attempt+1}/{MAX_RETRIES}")
                time.sleep(0.05)  # หน่วงสั้นๆ ก่อน retry
                
        except Exception as e:
            log.debug(f"Read error sensor {sensor_index} attempt {attempt+1}: {e}")
            if attempt < MAX_RETRIES - 1:
                time.sleep(0.05)
    
    # ❌ Fail ทุก retry
    _sensor_fail_counts[sensor_index] = _sensor_fail_counts.get(sensor_index, 0) + 1
    
    if not _disable_recovery_during_door_operation:
        if _sensor_fail_counts[sensor_index] >= SENSOR_FAIL_THRESHOLD:
            log.debug(f"Triggering recovery for sensor {sensor_index}")
            _auto_recover_if_needed(sensor_index)
    else:
        if _sensor_fail_counts[sensor_index] >= SENSOR_FAIL_THRESHOLD:
            log.debug(f"Sensor {sensor_index} needs recovery (deferred)")
    
    if sensor_index < len(_last_read_ts):
        _last_read_ts[sensor_index] = time.monotonic()
    
    return -1

# =============================================================================
# 🔧 ฟังก์ชันสำหรับ trigger recovery แบบ manual
# =============================================================================

def trigger_deferred_recovery() -> None:
    """
    ทำ recovery ให้ sensor ที่มี fail count สูง (เรียกหลังปิดประตูแล้ว)
    """
    for idx in range(len(SLOT_IDS)):
        if _sensor_fail_counts.get(idx, 0) >= SENSOR_FAIL_THRESHOLD:
            log.info(f"🔄 Triggering deferred recovery for sensor {idx}")
            _auto_recover_if_needed(idx)
# =============================================================================
# Utilities for diagnostics
# =============================================================================
def sensor_addr(index: int) -> Optional[int]:
    h = _vl53_handles.get(index)
    return h.addr if h else None

def vl53_address_map() -> Dict[int, tuple[int, str]]:
    """คืน mapping {index: (i2c_addr, backend)} สำหรับ debug."""
    return {i: (h.addr, h.backend) for i, h in _vl53_handles.items()}

def get_sensor_health() -> Dict[int, dict]:
    """
    คืนสถานะสุขภาพของ sensor ทั้งหมด
    ใช้สำหรับ monitoring/debugging
    """
    health = {}
    for index in range(len(SLOT_IDS)):
        health[index] = {
            "fail_count": _sensor_fail_counts.get(index, 0),
            "initialized": index in _vl53_handles,
            "address": sensor_addr(index),
            "last_read_ts": _last_read_ts[index] if index < len(_last_read_ts) else 0
        }
    return health

def diagnose_sensor(index: int, samples: int = 10) -> dict:
    """
    ฟังก์ชัน debug: อ่าน sensor หลายครั้งแล้วสรุปผล
    
    Usage:
        from shared.hardware_helpers import diagnose_sensor
        result = diagnose_sensor(0, samples=20)
        print(result)
    """
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
