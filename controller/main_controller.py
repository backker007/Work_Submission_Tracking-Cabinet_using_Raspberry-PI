# controller/main_controller.py
from __future__ import annotations
import os, sys, json, time, threading, ssl
from pathlib import Path
from queue import Queue, Full

# --- Path & .env ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from dotenv import load_dotenv
ROOT = Path(__file__).resolve().parents[1]
load_dotenv(ROOT / ".env")

# --- MQTT client ---
import paho.mqtt.client as mqtt
import logging

# --- Project helpers (จัดการ MQTT topics + publish รวมศูนย์ที่นี่) ---
from shared.topics import (
    CUPBOARD_ID, SLOT_IDS, SLOT_TO_INDEX, INDEX_TO_SLOT,
    get_subscriptions, publish_status, publish_warning,
)

# --- Hardware helpers  ---
from shared.hardware_helpers import (
    init_mcp, init_xshuts, init_sensors,
    read_sensor, move_servo_180, is_door_reliably_closed,
    vl53_sensors, mcp_pins, relay_pins, CHANGE_THRESHOLD, BUS_MODE,  
)

# --- Role helpers  ---
from shared.role_helpers import can_open_slot, can_open_door, is_valid_role

# =============================================================================
# CONFIG
# =============================================================================
ZERO_THRESHOLD = int(os.getenv("ZERO_THRESHOLD")) 
ACTIVE_CHECK_INTERVAL = float(os.getenv("ACTIVE_CHECK_INTERVAL"))
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()

logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s [%(threadName)s] %(message)s",
)
log = logging.getLogger("smartlocker")

# =============================================================================
# LOG HELPERS
# =============================================================================
def log_event(msg: str):
    log.info(msg)

def log_dbg(msg: str):
    log.debug(msg)

# =============================================================================
# GLOBAL STATE
# =============================================================================
selected_sensor_index: int | None = None
user_role: str | None = None
slot_status = [{"capacity_mm": 0, "connection_status": True, "is_open": False} for _ in SLOT_IDS]

i2c_lock = threading.RLock()
slot_queues: dict[str, Queue] = {sid: Queue(maxsize=10) for sid in SLOT_IDS}

# =============================================================================
# I²C SHORT-LOCK HELPERS
# =============================================================================
def i2c_read_sensor(index: int) -> float:
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
    sid = INDEX_TO_SLOT[idx]
    publish_status(sid, slot_status[idx])

def publish_warning_idx(idx: int, message: str):
    sid = INDEX_TO_SLOT[idx]
    publish_warning(sid, message)

# 🔧 VL53: helpers สำหรับอ่านทีละช่องแบบทน error
def _slot_index(slot_id: str) -> int | None:
    try:
        from shared.topics import SLOT_TO_INDEX  # ใช้ของเดิม ไม่แก้ไฟล์ topics
        return SLOT_TO_INDEX.get(slot_id)
    except Exception:
        return None

def read_capacity_and_connection(slot_id: str) -> tuple[int, bool]:
    """
    อ่านค่า mm และสถานะการเชื่อมต่อของช่องนั้น ๆ
      - capacity_mm = -1 เมื่ออ่านไม่ได้/นอกช่วง
      - connection_status = True เฉพาะเมื่อ 'มีอุปกรณ์/อ่านได้'
    หมายเหตุ: โหมด single จะบูต-อ่าน on-demand ที่ 0x29 ไม่มีไดรเวอร์ถาวรใน vl53_sensors
    """
    idx = _slot_index(slot_id)
    if idx is None:
        return -1, False

    # โหมด single: ให้ลองอ่านเลย ไม่ต้องเช็ก vl53_sensors
    if BUS_MODE == "single":
        try:
            mm = i2c_read_sensor(idx)
        except Exception:
            mm = -1
        return (mm, mm != -1)

    # โหมด multi (เดิม): ต้องมี driver ถาวร
    has_device = (0 <= idx < len(vl53_sensors)) and (vl53_sensors[idx] is not None)
    if not has_device:
        return -1, False

    try:
        mm = i2c_read_sensor(idx)
    except Exception:
        mm = -1
    return (mm, mm != -1)


# =============================================================================
# STORAGE COMPARTMENT STATE MACHINE
# =============================================================================
def Storage_compartment(index: int):
    try:
        i2c_move_servo_180(index, 180)
        log_event(f"🔄 เปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (→ 180°)")

        initial = -1.0
        timeout = time.time() + 5
        while initial <= 0 and time.time() < timeout:
            initial = i2c_read_sensor(index)
            time.sleep(ACTIVE_CHECK_INTERVAL)

        state = "wait_insert" if initial > 0 else "close_servo"
        if initial <= 0:
            log_event("❌ อ่านค่าเริ่มต้นไม่สำเร็จ → ปิดมอเตอร์")

        while state != "done":
            if state == "wait_insert":
                timeout = time.time() + 10
                while time.time() < timeout:
                    current = i2c_read_sensor(index)
                    log_dbg(f"⏳ รอการใส่ของ... {INDEX_TO_SLOT[index]}: {current:.1f} mm")
                    if current > 0 and current < initial - CHANGE_THRESHOLD:
                        log_event("📦 ตรวจพบการใส่ของครั้งแรก")
                        state = "monitor_movement"
                        break
                    time.sleep(ACTIVE_CHECK_INTERVAL)
                else:
                    log_event("⏱ หมดเวลาใส่ของ → ปิดมอเตอร์")
                    state = "close_servo"

            elif state == "monitor_movement":
                last_motion_time = time.time()
                last_distance = i2c_read_sensor(index)
                log_dbg("🔁 เริ่มตรวจจับความเคลื่อนไหว...")
                while True:
                    current = i2c_read_sensor(index)
                    log_dbg(f"🚱 {INDEX_TO_SLOT[index]}: {current:.1f} mm")
                    if abs(current - last_distance) >= CHANGE_THRESHOLD:
                        log_event("🔍 พบการขยับ → รีเซ็ตตัวจับเวลา")
                        last_motion_time = time.time()
                        last_distance = current
                    if time.time() - last_motion_time >= 3:
                        log_event("⏳ ไม่มีการขยับนาน 3 วิ → ปิดมอเตอร์")
                        break
                    time.sleep(ACTIVE_CHECK_INTERVAL)
                state = "close_servo"

            elif state == "close_servo":
                log_event(f"🔒 ปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (← 0°)")
                i2c_move_servo_180(index, 0)

                capacity = i2c_read_sensor(index)
                # ✅ ต้องเช็กว่ามีอุปกรณ์จริงและอ่านได้
                sensor_exists = (index < len(vl53_sensors)) and (vl53_sensors[index] is not None)
                is_connected = ((BUS_MODE == "single") and (capacity != -1)) or (sensor_exists and (capacity != -1))
                is_open = not i2c_is_door_closed(index)

                slot_status[index].update({
                    "capacity_mm": capacity,
                    "connection_status": is_connected,
                    "is_open": is_open,
                })
                publish_status_idx(index)
                state = "done"

        log_event(f"✅ ช่อง {INDEX_TO_SLOT[index]}: ปิด servo แล้ว")

    except Exception as e:
        log.error(f"[ERR] Storage_compartment({INDEX_TO_SLOT[index]}): {e}")

# =============================================================================
# DOOR UNLOCK SEQUENCE
# =============================================================================
def handle_door_unlock(index: int):
    try:
        i2c_set_relay(index, True)
        log_event(f"🔓 เปิดประตูช่อง {INDEX_TO_SLOT[index]} (Relay ON)")

        wait_start = time.time()
        while time.time() - wait_start <= 10:
            closed = i2c_is_door_closed(index)
            if not closed:
                slot_status[index]["is_open"] = True
                publish_status_idx(index)
                publish_warning_idx(index, "ประตูถูกเปิดแล้ว")
                log_event("✅ ประตูถูกเปิดแล้ว")
                break
            log_dbg("⏳ ยังไม่มีการเปิดประตู...")
            time.sleep(ACTIVE_CHECK_INTERVAL)
        else:
            i2c_set_relay(index, False)
            publish_warning_idx(index, "ประตูถูกล็อกแล้ว")  
            log_event("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            return

        last_warning_time = 0.0
        last_motion_time = time.time()
        last_distance = i2c_read_sensor(index)

        log_event("📦 เริ่มตรวจจับการนำเอกสารออก...")
        while True:
            if i2c_is_door_closed(index):
                publish_warning_idx(index, "ประตูถูกล็อกแล้ว")             
                log_event("🚪 ผู้ใช้ปิดประตูก่อน timeout → ไปล็อก")
                break

            current = i2c_read_sensor(index)
            log_dbg(f"📉 ความจุปัจจุบัน: {current:.1f} mm")

            if abs(current - last_distance) >= CHANGE_THRESHOLD:
                log_event("🔄 ตรวจพบการเคลื่อนไหว → รีเซ็ตเวลา")
                last_motion_time = time.time()
                last_distance = current

            if time.time() - last_motion_time > 5:
                if not i2c_is_door_closed(index) and time.time() - last_warning_time > 120:
                    log_event("⚠️ ไม่มีการเคลื่อนไหว และประตูยังไม่ปิด → แจ้งเตือนซ้ำ")
                    publish_warning_idx(index, "ลืมปิดประตู !!! กรุณาปิดให้สนิทเพื่อทำการล็อก")
                    last_warning_time = time.time()

            time.sleep(ACTIVE_CHECK_INTERVAL)

        log_event("⏳ ตรวจสอบซ้ำว่าประตูปิดสนิท รอ 1.5 วินาที...")
        while True:
            if i2c_is_door_closed(index):
                break
            if time.time() - last_warning_time > 120:
                publish_warning_idx(index, "ประตูยังไม่ปิดสนิท กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()
            time.sleep(ACTIVE_CHECK_INTERVAL)
        time.sleep(1.5)

        i2c_set_relay(index, False)
        publish_warning_idx(index, "ประตูถูกล็อกแล้ว")
        log_event(f"🔐 ล็อกประตูช่อง {INDEX_TO_SLOT[index]} (Relay OFF)")

        time.sleep(0.5)
        if not i2c_is_door_closed(index):
            publish_warning_idx(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
            return

        log_dbg("📏 รอค่าวัดความจุนิ่งก่อนส่งออก...")
        stable_start = time.time()
        stable_value = i2c_read_sensor(index)
        while True:
            current = i2c_read_sensor(index)
            if abs(current - stable_value) < 1:
                if time.time() - stable_start >= 2:
                    break
            else:
                stable_start = time.time()
                stable_value = current
            time.sleep(ACTIVE_CHECK_INTERVAL)

        new_value = i2c_read_sensor(index)
        if new_value == -1 :
            slot_status[index].update({
                "connection_status": False,  # ✅ แก้ให้สะท้อน mm อ่านไม่สำเร็จ
                "is_open": not i2c_is_door_closed(index),
            })          
        else:
            slot_status[index].update({
                "capacity_mm": new_value,
                "connection_status": True,   # ✅ อ่านสำเร็จถือว่า online
                "is_open": not i2c_is_door_closed(index),
            })
        publish_status_idx(index)

    except Exception as e:
        log.error(f"[ERR] handle_door_unlock({INDEX_TO_SLOT[index]}): {e}")

# =============================================================================
# ALIAS (เข้ากันได้กับโค้ดเก่า)
# =============================================================================
run_state_machine = Storage_compartment

# =============================================================================
# TOPIC/PAYLOAD PARSER + MESSAGE DISPATCH
# =============================================================================
def normalize_action(a: str) -> str:
    s = (a or "").strip().lower()
    if s in ("unlock", "open_door", "open-door"):
        return "door"
    if s in ("slot", "open_slot", "open-slot", "compartment", "bin"):
        return "slot"
    return s

def parse_command_topic(topic: str):
    parts = topic.split("/")
    if len(parts) < 5 or parts[0] != "smartlocker":
        return None, None

    slot_id = parts[3] if len(parts) > 3 else None
    seg4 = parts[4] if len(parts) > 4 else ""
    seg5 = parts[5] if len(parts) > 5 else ""
    seg6 = parts[6] if len(parts) > 6 else ""

    action_raw = None
    if seg4 in ("command", "command_open"):
        action_raw = seg5 or seg6
    elif seg5 in ("command", "command_open"):
        action_raw = seg6
    else:
        action_raw = seg6 or seg5 or seg4

    if not slot_id or not action_raw:
        return None, None

    return slot_id, normalize_action(action_raw)

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
    print("on_message:", msg.topic, msg.payload)
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
# TASK ENQUEUE (แทนการ spin thread)
# =============================================================================
def start_slot_task(action: str, slot_id: str, role: str):
    a = normalize_action(action)
    if a == "slot" and not can_open_slot(role):
        publish_warning(slot_id, f"🚫 role '{role}' ไม่มีสิทธิ์ {a}")
        return
    if a == "door" and not can_open_door(role):
        publish_warning(slot_id, f"🚫 role '{role}' ไม่มีสิทธิ์ {a}")
        return

    try:
        slot_queues[slot_id].put_nowait((a, role))
    except Full:
        log_event(f"🗑️ DROP: {slot_id} queue is full")
        publish_warning(slot_id, "ช่องนี้กำลังทำงานอยู่ (คิวเต็ม) คำสั่งถูกละทิ้ง")

# =============================================================================
# MQTT CONNECTOR (connect/subscribe/loop)
# =============================================================================
def on_connect(c, u, f, rc, props=None):
    for t in get_subscriptions(broad=True):
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
    client.reconnect_delay_set(min_delay=1, max_delay=16)
    client.enable_logger(log)

    client.connect(host, port, keepalive=60)
    return client

# =============================================================================
# WORKER STARTUP
# =============================================================================
def slot_worker(slot_id: str, idx: int):
    threading.current_thread().name = f"worker-{slot_id}"
    while True:
        action, role = slot_queues[slot_id].get()
        try:
            if action == "slot":
                Storage_compartment(idx)
            elif action == "door":
                handle_door_unlock(idx)
            else:
                publish_warning(slot_id, f"ไม่รู้จักคำสั่ง: {action}")
        except Exception as e:
            log.exception(f"[worker-{slot_id}] error while handling '{action}'")
            publish_warning(slot_id, f"เกิดข้อผิดพลาด: {e}")
        finally:
            slot_queues[slot_id].task_done()

def start_workers():
    for sid in SLOT_IDS:
        idx = SLOT_TO_INDEX[sid]
        t = threading.Thread(target=slot_worker, args=(sid, idx), daemon=True, name=f"worker-{sid}")
        t.start()

# =============================================================================
# UPDATE ALL SLOTS STATUS PERIODICALLY
# =============================================================================
def update_all_slots_status():
    # ✅ ใช้การอ่านแบบทน error ต่อช่อง + เว้นจังหวะสั้น ๆ กัน I2C ตีกัน
    for idx, sid in enumerate(SLOT_IDS):
        capacity, is_connected = read_capacity_and_connection(sid)
        is_open = not i2c_is_door_closed(idx)

        slot_status[idx].update({
            "capacity_mm": capacity,
            "connection_status": is_connected,
            "is_open": is_open,
        })
        publish_status_idx(idx)
        time.sleep(0.35)  # กันบัสแน่น

    threading.Timer(120, update_all_slots_status).start()  # 2 นาที

# =============================================================================
# MAIN
# =============================================================================
def main():
    # ✅ อย่าเรียก init_xshuts ซ้ำที่นี่อีก—init_sensors จะจัดการเอง
    init_mcp()
    init_sensors()

    start_workers()
    update_all_slots_status()

    client = build_client()
    client.loop_forever()

if __name__ == "__main__":
    main()
