# controller/main_controller.py
# -*- coding: utf-8 -*-
# =============================================================================
# Smart Locker Main Controller
# - MQTT command handler
# - Slot/Door workers
# - Periodic status publisher
# - I2C/Servo/Door helpers are in shared.hardware_helpers
# =============================================================================

from __future__ import annotations

import os
import sys
import json
import time
import threading
import ssl
import logging
import statistics
from pathlib import Path
from queue import Queue, Full

# --- Path & .env ต้องมาก่อน imports ในแพ็กเกจภายใน ---
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dotenv import load_dotenv
load_dotenv(ROOT / ".env")

# ==== [Env Validation & Tunables] ============================================
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s [%(threadName)s] %(message)s",
)
log = logging.getLogger("smartlocker")

# กุญแจ .env ที่ต้องมี
_REQUIRED_VARS = ("CUPBOARD_ID", "MQTT_HOST", "MQTT_PORT")
_missing = [k for k in _REQUIRED_VARS if not os.getenv(k)]
if _missing:
    raise RuntimeError(f"Missing required .env keys: {_missing}")

# Exponential backoff ตอน connect MQTT
RECONNECT_BASE_S = float(os.getenv("MQTT_RECONNECT_BASE_S", "1.0"))
RECONNECT_MAX_S = float(os.getenv("MQTT_RECONNECT_MAX_S", "32.0"))

# ขนาดคิวงานต่อช่อง (ตั้งค่า SLOT_QUEUE_MAXSIZE ก่อน ถ้าไม่ตั้งจะใช้ QUEUE_MAXSIZE)
SLOT_QUEUE_MAXSIZE = int(os.getenv("SLOT_QUEUE_MAXSIZE", os.getenv("QUEUE_MAXSIZE", "200")))

# --- MQTT client ---
import paho.mqtt.client as mqtt  # type: ignore

# --- Project helpers (topics/publish) ---
from shared.topics import (  # type: ignore
    CUPBOARD_ID, SLOT_IDS, SLOT_TO_INDEX, INDEX_TO_SLOT, BASE,
    get_subscriptions, publish_status, publish_warning, topic_status,
)

# --- Hardware helpers ---
# NOTE: มี internet_ok / vl53_address_map / ensure_sensors_ready สำหรับ debug และการปลุกเซนเซอร์
from shared.hardware_helpers import (  # type: ignore
    init_mcp, init_sensors,
    read_sensor, move_servo_180, is_door_reliably_closed,
    mcp_pins, relay_pins, CHANGE_THRESHOLD, is_slot_full,
    mcp,
    set_slot_led_ready, set_slot_led_error, set_slot_led_off,
    vl53_address_map, internet_ok, power_down_sensors, ensure_sensors_ready,
    reset_sensor_filter, read_sensor_fresh
)

AUTO_POWERDOWN = os.getenv("VL53_AUTO_POWERDOWN", "1").lower() in ("1", "true", "yes")

# --- Role helpers ---
from shared.role_helpers import can_open_slot, can_open_door, is_valid_role  # type: ignore

# =============================================================================
# CONFIG (.env)
# =============================================================================
ZERO_THRESHOLD = int(os.getenv("ZERO_THRESHOLD", "70"))
ACTIVE_CHECK_INTERVAL = float(os.getenv("ACTIVE_CHECK_INTERVAL", "0.5"))

# โซลินอยด์/รีดสวิตช์
DOOR_UNLOCK_WINDOW_S = int(os.getenv("DOOR_UNLOCK_WINDOW_S", "10"))     # เวลารอเปิดจริง
DOOR_DEBOUNCE_OPEN_S = float(os.getenv("DOOR_DEBOUNCE_OPEN_S", "0.5"))
DOOR_DEBOUNCE_CLOSE_S = float(os.getenv("DOOR_DEBOUNCE_CLOSE_S", "0.6"))
SOLENOID_PULSE_MS = int(os.getenv("SOLENOID_PULSE_MS", "0"))            # >0 = จ่ายพัลส์แล้วตัด
SOLENOID_KEEP_ON_WHILE_OPEN = os.getenv("SOLENOID_KEEP_ON_WHILE_OPEN", "0").lower() in ("1", "true", "yes")

# พารามิเตอร์เสริม
MOTION_TIMEOUT = float(os.getenv("MOTION_TIMEOUT", "30"))
MOTION_INACTIVE_BEFORE_WARN = float(os.getenv("MOTION_INACTIVE_BEFORE_WARN", "5"))
TIME_REPEAT_WARNING = float(os.getenv("TIME_REPEAT_WARNING", "10"))
SENSOR_STABLE_DURATION = float(os.getenv("SENSOR_STABLE_DURATION", "1.5"))
SENSOR_CHECK_INTERVAL = float(os.getenv("SENSOR_CHECK_INTERVAL", "0.2"))
_SMT = (os.getenv("SENSOR_MOTION_THRESHOLD", "").strip())
SENSOR_MOTION_THRESHOLD = int(_SMT) if _SMT.isdigit() else CHANGE_THRESHOLD

# ค่าช่วง mm → %
EMPTY_MM = float(os.getenv("EMPTY_MM", "200"))
FULL_MM  = float(os.getenv("FULL_MM",  "80"))

def mm_to_percent(mm_corr: int | None) -> int | None:
    # ค่า <= 0 ให้เป็น None (invalid) แทนที่จะกลายเป็น 100%
    if mm_corr is None or mm_corr <= 0:
        return None
    denom = max(1.0, (EMPTY_MM - FULL_MM))
    pct = round(((EMPTY_MM - float(mm_corr)) / denom) * 100.0)
    return max(0, min(100, pct))

# =============================================================================
# LOG HELPERS
# =============================================================================
def log_event(msg: str) -> None:
    log.info(msg)

def log_dbg(msg: str) -> None:
    log.debug(msg)

def i2c_pulse_relay(index: int, ms: int) -> None:
    """จ่ายไฟรีเลย์เป็นระยะเวลาสั้นๆ (ms) แล้วตัด"""
    ms = max(10, int(ms))
    i2c_set_relay(index, True)
    log_event(f"⚡ Relay CH{index} PULSE {ms} ms")
    time.sleep(ms / 1000.0)
    i2c_set_relay(index, False)

# =============================================================================
# GLOBAL STATE
# =============================================================================
mqtt_client: mqtt.Client | None = None

# โครงสร้างสถานะของแต่ละ slot (index ตาม SLOT_IDS)
slot_status = [{"capacity_mm": 0, "capacity_percent": None, "connection_status": True, "is_open": False} for _ in SLOT_IDS]

# ให้การเข้าถึง I2C/Servo/Door เป็น short critical section
i2c_lock = threading.RLock()

# คิวคำสั่งต่อ slot
slot_queues: dict[str, Queue] = {sid: Queue(maxsize=SLOT_QUEUE_MAXSIZE) for sid in SLOT_IDS}

_i2c_fail_counts = {sid: 0 for sid in SLOT_IDS}
FAIL_THRESHOLD = 3

_blink_stop_events: dict[int, threading.Event] = {}
_blink_threads: dict[int, threading.Thread] = {}

def _is_slot_blinking(idx: int) -> bool:
    t = _blink_threads.get(idx)
    return bool(t and t.is_alive())

def start_green_blink(idx: int, period_s: float = 0.6) -> None:
    """กระพริบไฟเขียวของช่อง idx ระหว่างที่กำลังสั่งงานช่องนั้น"""
    stop_old = _blink_stop_events.get(idx)
    if stop_old:
        stop_old.set()

    stop = threading.Event()
    _blink_stop_events[idx] = stop

    def _loop():
        on = True
        while not stop.is_set():
            if internet_ok():
                if on:
                    set_slot_led_ready(mcp, idx)
                else:
                    set_slot_led_off(mcp, idx)
            time.sleep(period_s / 2.0)
            on = not on

    t = threading.Thread(target=_loop, daemon=True, name=f"blink-green-{INDEX_TO_SLOT[idx]}")
    _blink_threads[idx] = t
    t.start()

def stop_blink(idx: int, set_final: bool = True) -> None:
    ev = _blink_stop_events.pop(idx, None)
    if ev:
        ev.set()
    t = _blink_threads.get(idx)
    if t and t.is_alive():
        try:
            t.join(timeout=0.05)
        except Exception:
            pass
    _blink_threads.pop(idx, None)

    if set_final and internet_ok():
        sid = INDEX_TO_SLOT[idx]
        mm_now = slot_status[idx]["capacity_mm"]
        if is_slot_full(sid, mm_now):
            set_slot_led_error(mcp, idx)
        else:
            set_slot_led_ready(mcp, idx)

# =============================================================================
# I²C SHORT-LOCK HELPERS
# =============================================================================
def i2c_read_sensor_fresh(index: int) -> int:
    """อ่านเซ็นเซอร์แบบสดภายใต้ I2C lock (median สั้น ๆ)"""
    with i2c_lock:
        return read_sensor_fresh(index, samples=3, gap_s=0.02)

def i2c_read_sensor_once_raw(index: int) -> int:
    """อ่านสด 1 ช็อต ไม่อิงบัฟเฟอร์/ฮิสเทอรีซิส"""
    with i2c_lock:
        return read_sensor(index, use_filter=False, reset_before=True)

def i2c_move_servo_180(index: int, angle: int) -> None:
    with i2c_lock:
        move_servo_180(index, angle)

def i2c_is_door_closed(index: int) -> bool:
    with i2c_lock:
        return is_door_reliably_closed(index)

def i2c_set_relay(index: int, value: bool) -> None:
    with i2c_lock:
        relay_pins[index].value = bool(value)

# =============================================================================
# LED-aware read helper
# =============================================================================
def read_slot_and_update_led(slot_id: str):
    idx = SLOT_TO_INDEX[slot_id]

    if not internet_ok():
        try:
            _ = i2c_read_sensor_once_raw(idx)
        except Exception:
            pass
        return None

    try:
        v = i2c_read_sensor_fresh(idx)
        if not isinstance(v, (int, float)) or v <= 0:
            v = read_until_ok_or_reinit(idx, pre_wait_s=1.0, post_wait_s=1.0, step_s=0.12)
    except Exception:
        v = -1

    if not _is_slot_blinking(idx) and v != -1 and internet_ok():
        try:
            if is_slot_full(slot_id, v):
                set_slot_led_error(mcp, idx)
            else:
                set_slot_led_ready(mcp, idx)
        except Exception:
            pass

    return v

def publish_slot_status_quick_single(idx: int) -> None:
    sid = INDEX_TO_SLOT[idx]

    v = i2c_read_sensor_fresh(idx)
    if not isinstance(v, (int, float)) or v <= 0:
        v = -1
    if v == -1:
        v = read_until_ok_or_reinit(idx, pre_wait_s=1.0, post_wait_s=1.0, step_s=0.12)

    try:
        if internet_ok() and not _is_slot_blinking(idx) and v != -1:
            if is_slot_full(sid, v):
                set_slot_led_error(mcp, idx)
            else:
                set_slot_led_ready(mcp, idx)
    except Exception:
        pass

    try:
        is_open = not i2c_is_door_closed(idx)
    except Exception:
        is_open = slot_status[idx].get("is_open", False)

    prev_mm = slot_status[idx]["capacity_mm"]
    mm_now = prev_mm if (not isinstance(v, (int, float)) or v < 0) else int(v)
    is_connected = internet_ok()

    slot_status[idx].update({
        "capacity_mm": mm_now,
        "capacity_percent": mm_to_percent(mm_now),
        "connection_status": is_connected,
        "is_open": is_open,
    })
    publish_status_idx(idx)

def read_until_ok_or_reinit(index: int, pre_wait_s: float = 3.0, post_wait_s: float = 3.0, step_s: float = 0.12) -> int:
    t0 = time.time()
    v = -1
    while time.time() - t0 < pre_wait_s:
        v = i2c_read_sensor_fresh(index)
        if isinstance(v, (int, float)) and v > 0:
            return v
        time.sleep(step_s)

    try:
        _ = i2c_read_sensor_once_raw(index)
    except Exception:
        pass

    for _ in range(3):
        try:
            _ = i2c_read_sensor_once_raw(index)
        except Exception:
            pass
        time.sleep(0.03)

    t1 = time.time()
    while time.time() - t1 < post_wait_s:
        v = i2c_read_sensor_fresh(index)
        if isinstance(v, (int, float)) and v > 0:
            return v
        time.sleep(step_s)

    return -1

# =============================================================================
# MQTT HELPERS (by index)
# =============================================================================
def publish_status_idx(idx: int) -> None:
    global mqtt_client
    if mqtt_client is None:
        log.error("publish_status_idx: mqtt_client is None")
        return
    sid = INDEX_TO_SLOT[idx]
    t = topic_status(sid)
    mid = publish_status(mqtt_client, slot_status[idx], sid)
    log.info(f"[MQTT] Published to {t} -> {slot_status[idx]} (mid={mid})")

def publish_warning_idx(idx: int, message: str) -> None:
    global mqtt_client
    if mqtt_client is None:
        log.error("publish_warning_idx: mqtt_client is None")
        return
    sid = INDEX_TO_SLOT[idx]
    mid = publish_warning(mqtt_client, message, sid)
    log.info(f"[PUB] warning {sid} mid={mid} message={message}")

def send_warning(slot_id: str, message: str, extra: dict | None = None) -> None:
    global mqtt_client
    if mqtt_client is None:
        log.error("send_warning: mqtt_client is None")
        return
    mid = publish_warning(mqtt_client, message, slot_id, extra)
    log.info(f"[PUB] warning {slot_id} mid={mid} message={message} extra={extra}")

# =============================================================================
# READ STABILIZERS
# =============================================================================
def _read_mm_stable(index: int, duration_s: float = 0.8, step_s: float = 0.1, retries: int = 1) -> int:
    vals = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        v = i2c_read_sensor_fresh(index)
        if isinstance(v, (int, float)) and v > 0:
            vals.append(v)
        time.sleep(step_s)
    if not vals and retries > 0:
        time.sleep(0.2)
        return _read_mm_stable(index, duration_s=0.4, step_s=0.1, retries=retries - 1)
    return int(statistics.median(vals)) if vals else -1

def _door_closed_stable(index: int, hold_s: float | None = None, step_s: float = 0.05) -> bool:
    if hold_s is None:
        hold_s = DOOR_DEBOUNCE_CLOSE_S
    t_end = time.time() + max(0.2, hold_s)
    while time.time() < t_end:
        if not i2c_is_door_closed(index):
            return False
        time.sleep(step_s)
    return True

def _door_open_stable(index: int, hold_s: float | None = None, step_s: float = 0.05) -> bool:
    if hold_s is None:
        hold_s = DOOR_DEBOUNCE_OPEN_S
    t_end = time.time() + max(0.2, hold_s)
    while time.time() < t_end:
        if i2c_is_door_closed(index):
            return False
        time.sleep(step_s)
    return True

# =============================================================================
# STORAGE COMPARTMENT STATE MACHINE  (UPDATED ORDER & PUBLISHER)
# =============================================================================
def Storage_compartment(index: int) -> None:
    """
    เปิดช่องให้ใส่ของ โดยใช้ publish_slot_status_quick_single() เป็นตัวเดียวในการส่งข้อมูล
    และ 'สั่งเซอร์โวก่อน' แล้วค่อย ensure_sensors_ready() ตามที่ต้องการ
    """
    try:
        # 1) เปิดมอเตอร์ "ก่อน" ปลุกเซนเซอร์
        i2c_move_servo_180(index, 75)
        log_event(f"🔄 เปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (→ 180°)")
        time.sleep(0.25)  # quiet window หลังขยับ

        # 2) ค่อยปลุก/เตรียมเซนเซอร์
        try:
            ensure_sensors_ready()
        except Exception as e:
            log_dbg(f"ensure_sensors_ready() failed (Storage_compartment): {e}")

        # 3) snapshot สถานะโดยใช้ฟังก์ชันกลาง
        publish_slot_status_quick_single(index)

        # 4) baseline สำหรับตรวจการวางของ
        initial = read_until_ok_or_reinit(index, pre_wait_s=3.0, post_wait_s=2.0)

        state = "wait_insert" if initial > 0 else "close_servo"
        if initial <= 0:
            log_event("❌ baseline ไม่สำเร็จ → ปิดมอเตอร์")

        while state != "done":
            if state == "wait_insert":
                timeout = time.time() + 12
                last_report = 0.0
                while time.time() < timeout:
                    cur = _read_mm_stable(index, duration_s=0.4)
                    publish_slot_status_quick_single(index)

                    if cur != -1 and initial > 0 and (initial - cur) >= max(CHANGE_THRESHOLD, 5):
                        log_event(f"📦 วางของแล้ว (Δ={initial - cur} mm)")
                        state = "monitor_movement"
                        break

                    if time.time() - last_report > 2:
                        log_dbg(f"⏳ รอใส่ของ {INDEX_TO_SLOT[index]}: cur={cur} (base={initial})")
                        last_report = time.time()

                    time.sleep(ACTIVE_CHECK_INTERVAL)
                else:
                    log_event("⏱ หมดเวลาใส่ของ → ปิดมอเตอร์")
                    state = "close_servo"

            elif state == "monitor_movement":
                last_motion_time = time.time()
                last_distance = _read_mm_stable(index, duration_s=0.4)
                log_dbg("🔁 ตรวจจับความเคลื่อนไหว...")

                while True:
                    cur = _read_mm_stable(index, duration_s=0.3)
                    publish_slot_status_quick_single(index)

                    if cur != -1 and last_distance != -1 and abs(cur - last_distance) >= max(CHANGE_THRESHOLD, 5):
                        last_motion_time = time.time()
                        last_distance = cur
                        log_dbg(f"🔍 เคลื่อนไหว: {cur} mm")

                    if time.time() - last_motion_time >= 3:
                        log_event("⏳ ไม่มีการขยับ 3 วิ → ปิดมอเตอร์")
                        break

                    time.sleep(ACTIVE_CHECK_INTERVAL)

                state = "close_servo"

            elif state == "close_servo":
                i2c_move_servo_180(index, 0)
                log_event(f"🔒 ปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (← 0°)")
                time.sleep(0.25)  # quiet window ก่อนอ่านสรุป

                # ใช้ฟังก์ชันกลางส่งสถานะ (อ่าน-คำนวณ-ตั้งไฟ-ส่ง MQTT)
                publish_slot_status_quick_single(index)
                state = "done"

        log_event(f"✅ ช่อง {INDEX_TO_SLOT[index]}: งานเสร็จ")

    except Exception as e:
        log.error(f"[ERR] Storage_compartment({INDEX_TO_SLOT[index]}): {e}")

    finally:
        if AUTO_POWERDOWN:
            try:
                power_down_sensors()
            except Exception as e:
                log_dbg(f"auto powerdown after Storage_compartment failed: {e}")

# =============================================================================
# DOOR UNLOCK (SOLENOID) SEQUENCE  (UPDATED ORDER & PUBLISHER)
# =============================================================================
def handle_door_unlock(index: int) -> None:
    """
    ปลดล็อกประตู โดยใช้ publish_slot_status_quick_single() เป็นตัวเดียวในการส่งข้อมูล
    และ 'สั่งรีเลย์ก่อน' แล้วค่อย ensure_sensors_ready()
    """
    try:
        # 1) ปลดล็อกก่อน (ตาม config pulse/hold)
        pulse = SOLENOID_PULSE_MS
        if pulse > 0:
            i2c_pulse_relay(index, pulse)
        else:
            i2c_set_relay(index, True)
            log_event(f"🔓 เปิดประตูช่อง {INDEX_TO_SLOT[index]} (Relay ON)")
            time.sleep(0.25)

        # 2) ค่อยปลุก/เตรียมเซนเซอร์
        try:
            ensure_sensors_ready()
        except Exception as e:
            log_dbg(f"ensure_sensors_ready() failed (handle_door_unlock): {e}")

        # 3) snapshot สถานะ
        publish_slot_status_quick_single(index)

        # 4) รอให้ "เปิดจริง"
        opened = False
        deadline = time.time() + DOOR_UNLOCK_WINDOW_S

        while time.time() < deadline:
            if _door_open_stable(index, hold_s=DOOR_DEBOUNCE_OPEN_S):
                publish_warning_idx(index, "ประตูถูกเปิดแล้ว")
                log_event("✅ ประตูถูกเปิดแล้ว")

                if SOLENOID_PULSE_MS == 0 and SOLENOID_KEEP_ON_WHILE_OPEN:
                    i2c_set_relay(index, True)

                publish_slot_status_quick_single(index)
                opened = True
                break

            publish_slot_status_quick_single(index)
            time.sleep(SENSOR_CHECK_INTERVAL)

        if not opened:
            i2c_set_relay(index, False)
            publish_warning_idx(index, "ประตูถูกล็อกแล้ว")
            log_event("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            return

        # 5) เฝ้าระหว่างเปิด
        last_warning_time = 0.0
        last_motion_time = time.time()
        last_distance = _read_mm_stable(index, duration_s=0.5)
        log_event("📦 เริ่มตรวจจับการนำเอกสารออก...")

        start_open_time = time.time()

        while True:
            if _door_closed_stable(index, hold_s=DOOR_DEBOUNCE_CLOSE_S):
                log_event("🚪 ผู้ใช้ปิดประตูแล้ว → ไปล็อก")
                break

            cur = _read_mm_stable(index, duration_s=0.3)
            if (cur != -1 and last_distance != -1 and
                    abs(cur - last_distance) >= max(SENSOR_MOTION_THRESHOLD, 1)):
                last_motion_time = time.time()
                last_distance = cur
                log_dbg(f"🔄 เคลื่อนไหวในช่อง: {cur} mm")

            publish_slot_status_quick_single(index)

            if (time.time() - last_motion_time > MOTION_INACTIVE_BEFORE_WARN and
                    not i2c_is_door_closed(index) and
                    time.time() - last_warning_time > TIME_REPEAT_WARNING):
                publish_warning_idx(index, "ลืมปิดประตู !!! กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()

            if time.time() - start_open_time > MOTION_TIMEOUT:
                log_event("⏳ หมดเวลาระหว่างเปิด → ไปขั้นตอนล็อก")
                break

            time.sleep(SENSOR_CHECK_INTERVAL)

        # 6) กันเด้งเล็กน้อย ก่อนสั่งล็อก
        time.sleep(0.3)

        # 7) สั่งล็อก (ตัดไฟ)
        i2c_set_relay(index, False)
        log_event(f"🔐 ล็อกประตูช่อง {INDEX_TO_SLOT[index]} (Relay OFF)")

        if not _door_closed_stable(index, hold_s=DOOR_DEBOUNCE_CLOSE_S):
            publish_warning_idx(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
            log_event("⚠️ ล็อกไม่สำเร็จเพราะประตูยังไม่ปิดคงที่")
            publish_slot_status_quick_single(index)
            return

        publish_warning_idx(index, "ประตูถูกล็อกแล้ว")

        # 8) อ่าน/คำนวณ/ส่ง ด้วยฟังก์ชันกลาง
        time.sleep(0.25)
        publish_slot_status_quick_single(index)

    except Exception as e:
        log.error(f"[ERR] handle_door_unlock({INDEX_TO_SLOT[index]}): {e}")

    finally:
        if AUTO_POWERDOWN:
            try:
                power_down_sensors()
            except Exception as e:
                log_dbg(f"auto powerdown after handle_door_unlock failed: {e}")

# =============================================================================
# TOPIC/PAYLOAD PARSER + MESSAGE DISPATCH
# =============================================================================
def normalize_action(a: str) -> str:
    s = (a or "").strip().lower()
    if s in ("unlock", "open_door", "open-door", "door"):
        return "door"
    if s in ("slot", "open_slot", "open-slot", "compartment", "bin"):
        return "slot"
    return s or "slot"

def parse_command_topic(topic: str):
    parts = topic.split("/")
    if len(parts) < 5 or parts[0] != BASE:
        return None, None

    if parts[2] not in ("slot", "slot_id"):
        return None, None
    slot_id = parts[3]

    action = ""
    key = None
    for k in ("command", "command_open"):
        if k in parts:
            key = k
            break
    if key:
        i = parts.index(key)
        action = "/".join(parts[i + 1:]) if i + 1 < len(parts) else ""

    return slot_id, normalize_action(action)

def parse_payload(raw_bytes: bytes) -> dict:
    raw = (raw_bytes or b"").decode("utf-8", errors="ignore").strip()
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except json.JSONDecodeError as e:
        log.error(f"[payload JSON error] {e} raw={raw!r}")
        return {}

def on_message(client, userdata, msg):
    try:
        slot_id, action = parse_command_topic(msg.topic)
        if not slot_id or not action:
            return
        data = parse_payload(msg.payload)
        role = str(data.get("role", "student")).lower()

        if slot_id not in SLOT_TO_INDEX:
            log.error(f"❌ Unknown slot_id: {slot_id}")
            return
        if not is_valid_role(role):
            log.error(f"❌ Invalid role: {role}")
            return

        log_event(f"[START] {action} {slot_id} by {role}")
        start_slot_task(action, slot_id, role)
    except Exception as e:
        log.error("[on_message ERR] %s", e)

# =============================================================================
# TASK ENQUEUE
# =============================================================================
def start_slot_task(action: str, slot_id: str, role: str) -> None:
    a = normalize_action(action)
    if a == "slot" and not can_open_slot(role):
        return send_warning(slot_id, f"🚫 role '{role}' ไม่มีสิทธิ์ {a}")
    if a == "door" and not can_open_door(role):
        return send_warning(slot_id, f"🚫 role '{role}' ไม่มีสิทธิ์ {a}")
    try:
        slot_queues[slot_id].put_nowait((a, role))
    except Full:
        log_event(f"🗑️ DROP: {slot_id} queue is full")
        send_warning(slot_id, "ช่องนี้กำลังทำงานอยู่ (คิวเต็ม) คำสั่งถูกละทิ้ง")

# =============================================================================
# MQTT CONNECTOR
# =============================================================================
def on_publish(client, userdata, mid, reason_code=None, properties=None, *args):
    rc = getattr(reason_code, "value", reason_code) if reason_code is not None else 0
    log.info(f"[MQTT] PUBACK mid={mid} reason={rc}")

def on_log(c, u, level, buf):
    log.debug(f"[PAHO] {buf}")

def on_connect(c, u, f, rc, props=None):
    for t in get_subscriptions(broad=False):
        c.subscribe(t, qos=1)
        log_event(f"[MQTT] Subscribed: {t}")

def on_disconnect(c, u, flags, rc, props=None):
    log.error(f"[MQTT] Disconnected rc={rc} flags={flags} props={props}")

def build_client():
    host = os.getenv("MQTT_HOST")
    port = int(os.getenv("MQTT_PORT"))
    user = os.getenv("MQTT_USERNAME")
    pw = os.getenv("MQTT_PASSWORD")
    use_tls = bool(int(os.getenv("MQTT_TLS", "0")))
    ws = bool(int(os.getenv("MQTT_WS", "0")))
    client_id = os.getenv("MQTT_CLIENT_ID")
    ca = os.getenv("MQTT_CA") or None

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=client_id,
        transport="websockets" if ws else "tcp",
        protocol=mqtt.MQTTv311,
    )
    if user:
        client.username_pw_set(user, pw)
    if ws:
        try:
            client.ws_set_options(path="/mqtt")
        except Exception:
            pass
    if use_tls:
        if ca:
            client.tls_set(ca_certs=ca, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        else:
            client.tls_set_context(ssl.create_default_context())
        client.tls_insecure_set(False)
        if port == 1883:
            port = 8883

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.on_publish = on_publish
    client.on_log = on_log
    client.reconnect_delay_set(min_delay=1, max_delay=16)

    client.max_inflight_messages_set(10)
    client.max_queued_messages_set(1000)
    client.enable_logger(log)

    delay = RECONNECT_BASE_S
    while True:
        try:
            client.connect(host, port, keepalive=40)
            break
        except Exception as e:
            log.warning("MQTT connect failed: %s (retry in %.1fs)", e, delay)
            time.sleep(delay)
            delay = min(RECONNECT_MAX_S, delay * 2)

    return client

# =============================================================================
# WORKERS
# =============================================================================
def slot_worker(slot_id: str, idx: int) -> None:
    threading.current_thread().name = f"worker-{slot_id}"
    while True:
        action, role = slot_queues[slot_id].get()
        try:
            start_green_blink(idx)

            if action == "slot":
                Storage_compartment(idx)
            elif action == "door":
                handle_door_unlock(idx)
            else:
                send_warning(slot_id, f"ไม่รู้จักคำสั่ง: {action}")
        except Exception:
            log.exception(f"[worker-{slot_id}] error while handling '{action}'")
            send_warning(slot_id, "เกิดข้อผิดพลาด")
        finally:
            stop_blink(idx, set_final=True)
            slot_queues[slot_id].task_done()

def start_workers() -> None:
    for sid in SLOT_IDS:
        idx = SLOT_TO_INDEX[sid]
        t = threading.Thread(target=slot_worker, args=(sid, idx), daemon=True, name=f"worker-{sid}")
        t.start()

# =============================================================================
# STATUS UPDATER
# =============================================================================
def publish_all_slots_status_quick() -> None:
    for idx, _sid in enumerate(SLOT_IDS):
        try:
            publish_slot_status_quick_single(idx)
        except Exception as e:
            log_dbg(f"quick-scan single failed for {INDEX_TO_SLOT[idx]}: {e}")
        time.sleep(0.10)

_status_updater_started = False

def start_status_updater(interval_s: int = 120, initial_delay_s: float = 3.0) -> None:
    global _status_updater_started
    if _status_updater_started:
        return

    for t in threading.enumerate():
        if t.name == "status-updater" and t.is_alive():
            _status_updater_started = True
            return

    def _loop():
        log.info("✅ Status updater started")
        time.sleep(initial_delay_s)
        while True:
            try:
                if 'publish_all_slots_status_once' in globals() and callable(globals()['publish_all_slots_status_once']):
                    globals()['publish_all_slots_status_once']()
                else:
                    publish_all_slots_status_quick()

                if AUTO_POWERDOWN:
                    try:
                        power_down_sensors()
                    except Exception as e:
                        log_dbg(f"power_down_sensors() failed: {e}")

            except Exception:
                log.exception("[status-updater] cycle failed")

            time.sleep(interval_s)

    threading.Thread(target=_loop, daemon=True, name="status-updater").start()
    _status_updater_started = True

# =============================================================================
# MAIN
# =============================================================================
def main() -> None:
    global mqtt_client
    init_mcp()
    init_sensors()

    try:
        log.info("VL53 address map: %s", vl53_address_map())
    except Exception:
        pass

    try:
        log.info("Network initial (hardware_helpers): %s", "ONLINE" if internet_ok() else "OFFLINE")
    except Exception:
        pass

    mqtt_client = build_client()
    mqtt_client.loop_start()

    start_workers()
    start_status_updater(interval_s=120, initial_delay_s=3.0)

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "diagnose":
        init_mcp()
        init_sensors()
        from shared.hardware_helpers import diagnose_sensor
        result = diagnose_sensor(0, samples=50)
        sys.exit(0)
    else:
        main()
