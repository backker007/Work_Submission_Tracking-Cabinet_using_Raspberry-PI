# controller/main_controller.py

from __future__ import annotations
import os, sys, json, time, threading, ssl, logging, statistics
from pathlib import Path
from queue import Queue, Full

# --- Path & .env ต้องมาก่อน imports ในแพ็กเกจภายใน ---
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from dotenv import load_dotenv
load_dotenv(ROOT / ".env")

# --- MQTT client ---
import paho.mqtt.client as mqtt

# --- Project helpers (topics/publish) ---
from shared.topics import (
    CUPBOARD_ID, SLOT_IDS, SLOT_TO_INDEX, INDEX_TO_SLOT, BASE,
    get_subscriptions, publish_status, publish_warning, topic_status,
)

# --- Hardware helpers ---
from shared.hardware_helpers import (
    init_mcp, init_sensors,
    read_sensor, move_servo_180, is_door_reliably_closed,
    mcp_pins, relay_pins, CHANGE_THRESHOLD,
)

# --- Role helpers ---
from shared.role_helpers import can_open_slot, can_open_door, is_valid_role


# =============================================================================
# CONFIG (.env)
# =============================================================================
ZERO_THRESHOLD          = int(os.getenv("ZERO_THRESHOLD", "70"))
ACTIVE_CHECK_INTERVAL   = float(os.getenv("ACTIVE_CHECK_INTERVAL", "0.5"))
LOG_LEVEL               = os.getenv("LOG_LEVEL", "INFO").upper()

# โซลินอยด์/รีดสวิตช์
DOOR_UNLOCK_WINDOW_S    = int(os.getenv("DOOR_UNLOCK_WINDOW_S", "10"))     # เวลารอเปิดจริง
DOOR_DEBOUNCE_OPEN_S    = float(os.getenv("DOOR_DEBOUNCE_OPEN_S", "0.5"))
DOOR_DEBOUNCE_CLOSE_S   = float(os.getenv("DOOR_DEBOUNCE_CLOSE_S", "0.6"))
SOLENOID_PULSE_MS       = int(os.getenv("SOLENOID_PULSE_MS", "0"))         # >0 = จ่ายพัลส์แล้วตัด
SOLENOID_KEEP_ON_WHILE_OPEN = os.getenv("SOLENOID_KEEP_ON_WHILE_OPEN", "0").lower() in ("1","true","yes")

# พารามิเตอร์เสริม (ตามที่ขอให้อยู่ .env)
MOTION_TIMEOUT              = float(os.getenv("MOTION_TIMEOUT", "30"))
MOTION_INACTIVE_BEFORE_WARN = float(os.getenv("MOTION_INACTIVE_BEFORE_WARN", "5"))
TIME_REPEAT_WARNING         = float(os.getenv("TIME_REPEAT_WARNING", "10"))
SENSOR_STABLE_DURATION      = float(os.getenv("SENSOR_STABLE_DURATION", "1.5"))
SENSOR_CHECK_INTERVAL       = float(os.getenv("SENSOR_CHECK_INTERVAL", "0.2"))
# ทนเคส env เป็นค่าว่างโดยไม่เปลี่ยน logic เดิม
_SMT = (os.getenv("SENSOR_MOTION_THRESHOLD", "").strip())
SENSOR_MOTION_THRESHOLD     = int(_SMT) if _SMT.isdigit() else CHANGE_THRESHOLD

logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s [%(threadName)s] %(message)s",
)
log = logging.getLogger("smartlocker")


# =============================================================================
# LOG HELPERS
# =============================================================================
def log_event(msg: str): log.info(msg)
def log_dbg(msg: str): log.debug(msg)


# =============================================================================
# GLOBAL STATE
# =============================================================================
mqtt_client: mqtt.Client | None = None

slot_status = [{"capacity_mm": 0, "connection_status": True, "is_open": False} for _ in SLOT_IDS]

i2c_lock = threading.RLock()
slot_queues: dict[str, Queue] = {sid: Queue(maxsize=10) for sid in SLOT_IDS}


# =============================================================================
# I²C SHORT-LOCK HELPERS
# =============================================================================
def i2c_read_sensor(index: int) -> int:
    with i2c_lock:
        return read_sensor(index)

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
# MQTT HELPERS (by index)
# =============================================================================
def publish_status_idx(idx: int):
    global mqtt_client
    if mqtt_client is None:
        log.error("publish_status_idx: mqtt_client is None")
        return
    sid = INDEX_TO_SLOT[idx]
    t = topic_status(sid)
    mid = publish_status(mqtt_client, slot_status[idx], sid)
    log.info(f"[MQTT] Published to {t} -> {slot_status[idx]} (mid={mid})")

def publish_warning_idx(idx: int, message: str):
    global mqtt_client
    if mqtt_client is None:
        log.error("publish_warning_idx: mqtt_client is None")
        return
    sid = INDEX_TO_SLOT[idx]
    mid = publish_warning(mqtt_client, message, sid)
    log.info(f"[PUB] warning {sid} mid={mid} message={message}")

def send_warning(slot_id: str, message: str, extra: dict | None = None):
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
    """อ่านหลายครั้งคืน median; ถ้าได้ -1 ล้วน retry สั้น ๆ ก่อนคืน -1"""
    vals = []
    t0 = time.time()
    while time.time() - t0 < duration_s:
        v = i2c_read_sensor(index)
        if v != -1:
            vals.append(v)
        time.sleep(step_s)
    if not vals and retries > 0:
        time.sleep(0.2)
        return _read_mm_stable(index, duration_s=0.4, step_s=0.1, retries=retries-1)
    return int(statistics.median(vals)) if vals else -1

def _door_closed_stable(index: int, hold_s: float | None = None, step_s: float = 0.05) -> bool:
    if hold_s is None: hold_s = DOOR_DEBOUNCE_CLOSE_S
    t_end = time.time() + max(0.2, hold_s)
    while time.time() < t_end:
        if not i2c_is_door_closed(index):
            return False
        time.sleep(step_s)
    return True

def _door_open_stable(index: int, hold_s: float | None = None, step_s: float = 0.05) -> bool:
    if hold_s is None: hold_s = DOOR_DEBOUNCE_OPEN_S
    t_end = time.time() + max(0.2, hold_s)
    while time.time() < t_end:
        if i2c_is_door_closed(index):
            return False
        time.sleep(step_s)
    return True


# =============================================================================
# STORAGE COMPARTMENT STATE MACHINE
# =============================================================================
def Storage_compartment(index: int):
    try:
        i2c_move_servo_180(index, 180)
        log_event(f"🔄 เปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (→ 180°)")

        # baseline เสถียร (สูงสุด 5s)
        initial, deadline = -1, time.time() + 5
        while initial <= 0 and time.time() < deadline:
            initial = _read_mm_stable(index, duration_s=0.5)
            if initial <= 0:
                time.sleep(ACTIVE_CHECK_INTERVAL)

        state = "wait_insert" if initial > 0 else "close_servo"
        if initial <= 0:
            log_event("❌ baseline ไม่สำเร็จภายใน 5s → ปิดมอเตอร์")

        while state != "done":
            if state == "wait_insert":
                timeout = time.time() + 12
                last_report = 0.0
                while time.time() < timeout:
                    cur = _read_mm_stable(index, duration_s=0.4)
                    if cur > 0 and (initial - cur) >= max(CHANGE_THRESHOLD, 5):
                        log_event(f"📦 วางของแล้ว (Δ={initial-cur} mm)")
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

                capacity = _read_mm_stable(index, duration_s=0.8)
                is_connected = (capacity != -1)
                is_open = not i2c_is_door_closed(index)

                slot_status[index].update({
                    "capacity_mm": capacity if capacity != -1 else slot_status[index]["capacity_mm"],
                    "connection_status": is_connected,
                    "is_open": is_open,
                })
                publish_status_idx(index)
                state = "done"

        log_event(f"✅ ช่อง {INDEX_TO_SLOT[index]}: งานเสร็จ")
    except Exception as e:
        log.error(f"[ERR] Storage_compartment({INDEX_TO_SLOT[index]}): {e}")


# =============================================================================
# DOOR UNLOCK (SOLENOID) SEQUENCE
# =============================================================================
def handle_door_unlock(index: int):
    """
    โซลินอยด์: ปลดล็อก → รอผู้ใช้ "เปิดจริง" ภายใน DOOR_UNLOCK_WINDOW_S
      - ถ้าเปิดทันเวลา: (เลือกค้างไฟ/ไม่ค้างไฟ) → เฝ้าการเคลื่อนไหว/เตือน → รอ "ปิดจริง"
      - ถ้าไม่เปิดทัน: ตัดไฟและจบ
    ใช้ MC-38 แบบดีบาวน์ผ่าน _door_open_stable/_door_closed_stable
    """
    try:
        # 1) ปลดล็อก
        i2c_set_relay(index, True)
        log_event(f"🔓 เปิดประตูช่อง {INDEX_TO_SLOT[index]} (Relay ON)")

        # โหมดพัลส์: จ่ายไฟ x ms แล้วตัดเลย
        if SOLENOID_PULSE_MS > 0:
            time.sleep(SOLENOID_PULSE_MS / 1000.0)
            i2c_set_relay(index, False)
        else:
            # กันสัญญาณเด้งจากรีเลย์
            time.sleep(0.25)

        # 2) รอให้ "เปิดจริง" (debounce) ภายในหน้าต่างเวลา
        opened = False
        deadline = time.time() + DOOR_UNLOCK_WINDOW_S
        while time.time() < deadline:
            if _door_open_stable(index, hold_s=DOOR_DEBOUNCE_OPEN_S):
                slot_status[index]["is_open"] = True
                publish_status_idx(index)
                publish_warning_idx(index, "ประตูถูกเปิดแล้ว")
                log_event("✅ ประตูถูกเปิดแล้ว")

                # ถ้าไม่ได้ใช้พัลส์ และต้องการค้างไฟไว้ระหว่างเปิด
                if SOLENOID_PULSE_MS == 0 and SOLENOID_KEEP_ON_WHILE_OPEN:
                    i2c_set_relay(index, True)
                opened = True
                break
            time.sleep(SENSOR_CHECK_INTERVAL)

        if not opened:
            # ไม่ได้เปิดภายในเวลา → ตัดไฟและประกาศล็อก
            i2c_set_relay(index, False)
            publish_warning_idx(index, "ประตูถูกล็อกแล้ว")
            log_event("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            return

        # 3) เฝ้าระหว่างเปิด: ตรวจจับการเคลื่อนไหว + เตือนเป็นช่วง ๆ หากค้าง
        last_warning_time = 0.0
        last_motion_time = time.time()
        last_distance = _read_mm_stable(index, duration_s=0.5)
        log_event("📦 เริ่มตรวจจับการนำเอกสารออก...")

        start_open_time = time.time()
        while True:
            # ถ้าปิด "เสถียร" แล้ว → ไปล็อก
            if _door_closed_stable(index, hold_s=DOOR_DEBOUNCE_CLOSE_S):
                log_event("🚪 ผู้ใช้ปิดประตูแล้ว → ไปล็อก")
                break

            # อ่านระยะให้เสถียร (ไม่ยุ่งกับ logic ใน hardware_helpers นอกเหนือจากนี้)
            cur = _read_mm_stable(index, duration_s=0.3)
            if (cur != -1 and last_distance != -1 and
                abs(cur - last_distance) >= max(SENSOR_MOTION_THRESHOLD, 1)):
                last_motion_time = time.time()
                last_distance = cur
                log_dbg(f"🔄 เคลื่อนไหวในช่อง: {cur} mm")

            # เตือนซ้ำหากค้างนาน/ไม่มีการขยับ
            if (time.time() - last_motion_time > MOTION_INACTIVE_BEFORE_WARN and
                not i2c_is_door_closed(index) and
                time.time() - last_warning_time > TIME_REPEAT_WARNING):
                publish_warning_idx(index, "ลืมปิดประตู !!! กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()

            # hard timeout ระหว่างเปิด
            if time.time() - start_open_time > MOTION_TIMEOUT:
                log_event("⏳ หมดเวลาระหว่างเปิด → ไปขั้นตอนล็อก")
                break

            time.sleep(SENSOR_CHECK_INTERVAL)

        # 4) กันเด้งเพิ่มนิดก่อนสั่งล็อก
        time.sleep(0.3)

        # 5) สั่งล็อก (ตัดไฟ)
        i2c_set_relay(index, False)
        log_event(f"🔐 ล็อกประตูช่อง {INDEX_TO_SLOT[index]} (Relay OFF)")

        # Double-check ว่าปิดจริงแบบคงที่
        if not _door_closed_stable(index, hold_s=DOOR_DEBOUNCE_CLOSE_S):
            publish_warning_idx(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
            log_event("⚠️ ล็อกไม่สำเร็จเพราะประตูยังไม่ปิดคงที่")
            return

        # แจ้งล็อกสำเร็จหลัง OFF สำเร็จเท่านั้น
        publish_warning_idx(index, "ประตูถูกล็อกแล้ว")

        # 6) อ่านค่าเสถียรสรุปสถานะ (ระวัง: ไม่แก้ logic VL53L0X)
        final_value = _read_mm_stable(index, duration_s=SENSOR_STABLE_DURATION)
        slot_status[index].update({
            "capacity_mm": final_value if final_value != -1 else slot_status[index]["capacity_mm"],
            "connection_status": (final_value != -1),
            "is_open": not i2c_is_door_closed(index),
        })
        publish_status_idx(index)

    except Exception as e:
        log.error(f"[ERR] handle_door_unlock({INDEX_TO_SLOT[index]}): {e}")


# =============================================================================
# TOPIC/PAYLOAD PARSER + MESSAGE DISPATCH
# =============================================================================
def normalize_action(a: str) -> str:
    s = (a or "").strip().lower()
    if s in ("unlock", "open_door", "open-door", "door"): return "door"
    if s in ("slot", "open_slot", "open-slot", "compartment", "bin"): return "slot"
    return s or "slot"

def parse_command_topic(topic: str):
    """
    รองรับ 2 รูปแบบ:
    - {BASE}/{node}/slot/{slot_id}/command[/...action]          (modern)
    - {BASE}/{node}/slot_id/{slot_id}/command_open[/...action]  (legacy)
    """
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
        action = "/".join(parts[i+1:]) if i+1 < len(parts) else ""

    return slot_id, normalize_action(action)

def parse_payload(raw_bytes: bytes) -> dict:
    raw = (raw_bytes or b"").decode("utf-8", errors="ignore").strip()
    if not raw: return {}
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
            log.error(f"❌ Unknown slot_id: {slot_id}"); return
        if not is_valid_role(role):
            log.error(f"❌ Invalid role: {role}"); return

        log_event(f"[START] {action} {slot_id} by {role}")
        start_slot_task(action, slot_id, role)
    except Exception as e:
        log.error("[on_message ERR] %s", e)


# =============================================================================
# TASK ENQUEUE
# =============================================================================
def start_slot_task(action: str, slot_id: str, role: str):
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
    # ไม่ยิงสถานะทันทีที่นี่ เพื่อกันซ้ำ

def on_disconnect(c, u, flags, rc, props=None):
    log.error(f"[MQTT] Disconnected rc={rc} flags={flags} props={props}")

def build_client():
    host = os.getenv("MQTT_HOST"); port = int(os.getenv("MQTT_PORT"))
    user = os.getenv("MQTT_USERNAME"); pw = os.getenv("MQTT_PASSWORD")
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
        try: client.ws_set_options(path="/mqtt")
        except Exception: pass
    if use_tls:
        if ca: client.tls_set(ca_certs=ca, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        else:  client.tls_set_context(ssl.create_default_context())
        client.tls_insecure_set(False)
        if port == 1883: port = 8883

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.on_publish = on_publish
    client.on_log = on_log
    client.reconnect_delay_set(min_delay=1, max_delay=16)

    # ความเสถียร QoS1
    client.max_inflight_messages_set(10)
    client.max_queued_messages_set(1000)

    client.enable_logger(log)
    client.connect(host, port, keepalive=40)
    return client


# =============================================================================
# WORKERS
# =============================================================================
def slot_worker(slot_id: str, idx: int):
    threading.current_thread().name = f"worker-{slot_id}"
    while True:
        action, role = slot_queues[slot_id].get()
        try:
            if action == "slot": Storage_compartment(idx)
            elif action == "door": handle_door_unlock(idx)
            else: send_warning(slot_id, f"ไม่รู้จักคำสั่ง: {action}")
        except Exception:
            log.exception(f"[worker-{slot_id}] error while handling '{action}'")
            send_warning(slot_id, "เกิดข้อผิดพลาด")
        finally:
            slot_queues[slot_id].task_done()

def start_workers():
    for sid in SLOT_IDS:
        idx = SLOT_TO_INDEX[sid]
        t = threading.Thread(target=slot_worker, args=(sid, idx), daemon=True, name=f"worker-{sid}")
        t.start()


# =============================================================================
# STATUS UPDATER (ตัวเดียว ไม่ซ้ำ)
# =============================================================================
def publish_all_slots_status_once():
    for idx, sid in enumerate(SLOT_IDS):
        capacity = _read_mm_stable(idx, duration_s=0.5)
        is_connected = (capacity != -1)
        is_open = not i2c_is_door_closed(idx)
        slot_status[idx].update({
            "capacity_mm": capacity if capacity != -1 else slot_status[idx]["capacity_mm"],
            "connection_status": is_connected,
            "is_open": is_open,
        })
        publish_status_idx(idx)
        time.sleep(0.35)

_status_updater_started = False
def start_status_updater(interval_s: int = 120, initial_delay_s: float = 3.0):
    global _status_updater_started
    if _status_updater_started: return
    _status_updater_started = True
    def _loop():
        log.info("✅ Status updater started")
        time.sleep(initial_delay_s)
        while True:
            try: publish_all_slots_status_once()
            except Exception: log.exception("[status-updater] cycle failed")
            time.sleep(interval_s)
    threading.Thread(target=_loop, daemon=True, name="status-updater").start()


# =============================================================================
# MAIN
# =============================================================================
def main():
    global mqtt_client
    init_mcp()
    init_sensors()

    mqtt_client = build_client()
    mqtt_client.loop_start()

    start_workers()
    start_status_updater(interval_s=1800, initial_delay_s=3.0)

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
