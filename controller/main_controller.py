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
    init_mcp, init_xshuts, reset_vl53_addresses, init_sensors,
    read_sensor, move_servo_180, is_door_reliably_closed,
    vl53_sensors, mcp_pins, relay_pins, CHANGE_THRESHOLD,
)

# --- Role helpers  ---
from shared.role_helpers import can_open_slot, can_open_door, is_valid_role


# =============================================================================
# CONFIG
# =============================================================================

ZERO_THRESHOLD = int(os.getenv("ZERO_THRESHOLD", "10"))  # >10mm = ว่าง
ACTIVE_CHECK_INTERVAL = float(os.getenv("ACTIVE_CHECK_INTERVAL", "0.10"))
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
    """Edge events (เปิด/ปิด/เตือน/ค่าเปลี่ยนสำคัญ)"""
    log.info(msg)

def log_dbg(msg: str):
    """ข้อความถี่ (เห็นเมื่อ LOG_LEVEL=DEBUG)"""
    log.debug(msg)

# =============================================================================
# GLOBAL STATE
# =============================================================================

selected_sensor_index: int | None = None
user_role: str | None = None

# สถานะของแต่ละช่อง (index ตรงกับ SLOT_IDS)
slot_status = [{"capacity_mm": 0, "connection_status": True, "is_open": False} for _ in SLOT_IDS]

# I2C lock ป้องกัน race ระหว่าง sensor/servo/relay
i2c_lock = threading.RLock()

# คิวงานต่อช่อง + worker ต่อช่อง (จำนวน thread คงที่, ไม่ spin thread ใหม่ทุกคำสั่ง)
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


# =============================================================================
# STORAGE COMPARTMENT STATE MACHINE
# =============================================================================

def Storage_compartment(index: int):
    """เปิดฝาช่องเพื่อ 'หย่อนเอกสาร' แล้วปิดเองเมื่อไม่มีการขยับ 3 วินาที"""
    try:
        i2c_move_servo_180(index, 180)
        log_event(f"🔄 เปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (→ 180°)")

        # อ่านค่าเริ่มต้นภายใน 5 วิ
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
                sensor_exists = index < len(vl53_sensors)
                is_available = (capacity > ZERO_THRESHOLD) and sensor_exists
                is_open = not i2c_is_door_closed(index)

                slot_status[index].update({
                    "capacity_mm": capacity,
                    "connection_status": is_available,
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
    """ปลดล็อกประตู → รอเปิด → ตรวจจับการนำออก → รอปิด → ล็อกกลับ → ส่งสถานะ"""
    try:
        i2c_set_relay(index, True)
        log_event(f"🔓 เปิดประตูช่อง {INDEX_TO_SLOT[index]} (Relay ON)")

        # รอการเปิดประตูสูงสุด 10 วิ
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
            # Timeout → ล็อกกลับ
            i2c_set_relay(index, False)
            publish_warning_idx(index, "ประตูถูกล็อกแล้ว")  
            log_event("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            return

        # เริ่มตรวจจับการนำเอกสารออก
        extract_start = time.time()
        last_warning_time = 0.0
        last_motion_time = time.time()
        last_distance = i2c_read_sensor(index)
        motion_detected = False

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
                motion_detected = True

            # หมดเวลา 30 วิ → ไปต่อขั้นรอปิด
            # if time.time() - extract_start > 30:
            #     if not motion_detected:
            #         log_event("⚠️ เปิดประตูแล้วแต่ไม่พบการขยับเลย → แจ้งเตือน")
            #         publish_warning_idx(index, "เปิดประตูแล้ว ")
            #     log_event("⏳ หมดเวลานำออก → เข้าสู่โหมดรอปิดประตู")
            #     break

            # ไม่มีการขยับ 5 วิ และยังไม่ปิด → เตือนทุก 120 วิ
            if time.time() - last_motion_time > 5:
                if not i2c_is_door_closed(index) and time.time() - last_warning_time > 120:
                    log_event("⚠️ ไม่มีการเคลื่อนไหว และประตูยังไม่ปิด → แจ้งเตือนซ้ำ")
                    publish_warning_idx(index, "ลืมปิดประตู !!! กรุณาปิดให้สนิทเพื่อทำการล็อก")
                    last_warning_time = time.time()

            time.sleep(ACTIVE_CHECK_INTERVAL)

        # รอให้ปิดสนิทจริง ๆ
        log_event("⏳ ตรวจสอบซ้ำว่าประตูปิดสนิท รอ 1.5 วินาที...")
        while True:
            if i2c_is_door_closed(index):
                break
            if time.time() - last_warning_time > 120:
                publish_warning_idx(index, "ประตูยังไม่ปิดสนิท กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()
            time.sleep(ACTIVE_CHECK_INTERVAL)
        time.sleep(1.5)

        # ล็อกกลับ
        i2c_set_relay(index, False)
        publish_warning_idx(index, "ประตูถูกล็อกแล้ว")
        log_event(f"🔐 ล็อกประตูช่อง {INDEX_TO_SLOT[index]} (Relay OFF)")

        # Double-check ปิดสนิท
        time.sleep(0.5)
        if not i2c_is_door_closed(index):
            publish_warning_idx(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
            return

        # รอค่า capacity นิ่งก่อนคำนวนใหม่
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
        slot_status[index]["capacity_mm"] = new_value
        slot_status[index]["is_open"] = not i2c_is_door_closed(index)
        slot_status[index]["connection_status"] = new_value > ZERO_THRESHOLD
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
    """
    รองรับ:
    - smartlocker/{node}/slot/{slotId}/command/{action}
    - smartlocker/{node}/slot_id/{slotId}/command_open/{action}
    - กันกรณีมี '//' บาง segment ว่าง
    คืนค่า: (slot_id, action_norm) หรือ (None, None)
    """
    parts = topic.split("/")
    if len(parts) < 5 or parts[0] != "smartlocker":
        return None, None

    # parts: 0 smartlocker, 1 node, 2 slot|slot_id, 3 slotId, 4 maybe command*, 5 maybe action, 6 maybe action
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
    client.enable_logger(log)  # รวม log ของ Paho เข้ากับ logger เดียวกัน

    client.connect(host, port, keepalive=60)
    return client

# =============================================================================
# WORKER STARTUP
    # Worker ประจำ 'ช่อง' เดียว:
    # - รันวนตลอดชีวิตเธรด
    # - ดึงงานจากคิวของช่องนั้น (serialize งานต่อช่องโดยอัตโนมัติ)
    # - ทำงานตาม action: "slot" หรือ "door"
    # - ไม่สร้างเธรดใหม่ต่อคำสั่ง → จำนวนเธรดคงที่ อ่าน log ง่าย
# =============================================================================

def slot_worker(slot_id: str, idx: int):
    # ตั้งชื่อเธรดให้อ่าน log ง่าย
    threading.current_thread().name = f"worker-{slot_id}"
    while True:
        # ดึงงานจากคิวของช่องนั้น (action, role)
        action, role = slot_queues[slot_id].get()
        try:
            if action == "slot":
                Storage_compartment(idx)
            elif action == "door":
                handle_door_unlock(idx)
            else:
                publish_warning(slot_id, f"ไม่รู้จักคำสั่ง: {action}")
        except Exception as e:
            # กันล้มเธรด worker และมี log/แจ้งเตือน
            log.exception(f"[worker-{slot_id}] error while handling '{action}'")
            publish_warning(slot_id, f"เกิดข้อผิดพลาด: {e}")
        finally:
            # แจ้งคิวว่างให้ตัวถัดไปทำงาน
            slot_queues[slot_id].task_done()

# =============================================================================
# WORKER STARTUP
# =============================================================================

def start_workers():
    for sid in SLOT_IDS:
        idx = SLOT_TO_INDEX[sid]
        t = threading.Thread(target=slot_worker, args=(sid, idx), daemon=True, name=f"worker-{sid}")
        t.start()


# =============================================================================
# MAIN
# =============================================================================

def main():
    # Init hardware
    init_mcp()
    init_xshuts()
    reset_vl53_addresses()
    init_sensors()

    # ค่าเริ่มต้น (ตัวอย่าง/เดโม)
    for i, sid in enumerate(SLOT_IDS):
        publish_status(sid, slot_status[i])
        time.sleep(0.05)

    # 🔧 เริ่ม worker ต่อช่อง (จำนวน thread คงที่, ชื่อคงที่)
    start_workers()

    # MQTT loop
    client = build_client()
    client.loop_forever()
    


if __name__ == "__main__":
    main()
