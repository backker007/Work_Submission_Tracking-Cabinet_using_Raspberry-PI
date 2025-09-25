# controller/main_controller.py
from __future__ import annotations
import os, sys, json, time, threading, queue, ssl
from pathlib import Path

# --- Path & .env ---
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from dotenv import load_dotenv
ROOT = Path(__file__).resolve().parents[1]
load_dotenv(ROOT / ".env")

# --- MQTT client ---
import paho.mqtt.client as mqtt

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

# ===== CONFIG =====
ZERO_THRESHOLD = int(os.getenv("ZERO_THRESHOLD", "70"))  # >70mm = ว่าง

# ===== Global State =====
slot_status = [{"capacity_mm": 0, "available": True, "door_closed": True} for _ in SLOT_IDS]
cmd_q: "queue.Queue[tuple[str,str,str]]" = queue.Queue()   # (action, slot_id, role)
reading_active = False
selected_sensor_index: int | None = None
user_role: str | None = None

# ===== MQTT helpers for idx =====
def publish_status_idx(idx: int):
    sid = INDEX_TO_SLOT[idx]
    publish_status(sid, slot_status[idx])

def publish_warning_idx(idx: int, message: str):
    sid = INDEX_TO_SLOT[idx]
    publish_warning(sid, message)

# ===== STORAGE COMPARTMENT STATE MACHINE (ใส่เอกสาร) =====
def Storage_compartment(index: int):
    """เปิด servo → รอผู้ใช้ 'ใส่ของ' → เฝ้าการขยับจนเงียบ → ปิด servo → อ่านค่า/ประกาศสถานะ"""
    global reading_active
    STATE_WAIT_INSERT = "wait_insert"
    STATE_MONITOR_MOVEMENT = "monitor_movement"
    STATE_CLOSE_SERVO = "close_servo"
    STATE_DONE = "done"

    try:
        move_servo_180(index, 180)
        print(f"🔄 เปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (→ 180°)")

        # อ่านค่าเริ่มต้น (มี timeout)
        initial = -1
        timeout = time.time() + 5
        while initial <= 0 and time.time() < timeout:
            initial = read_sensor(index)
            time.sleep(0.2)

        state = STATE_WAIT_INSERT if initial > 0 else STATE_CLOSE_SERVO
        if initial <= 0:
            print("❌ อ่านค่าเริ่มต้นไม่สำเร็จ → ปิดมอเตอร์")

        while state != STATE_DONE:
            if state == STATE_WAIT_INSERT:
                timeout = time.time() + 10
                while time.time() < timeout:
                    current = read_sensor(index)
                    print(f"⏳ รอการใส่ของ... {INDEX_TO_SLOT[index]}: {current:.1f} mm")
                    if current > 0 and current < initial - CHANGE_THRESHOLD:
                        print("📦 ตรวจพบการใส่ของครั้งแรก")
                        state = STATE_MONITOR_MOVEMENT
                        break
                    time.sleep(0.2)
                else:
                    print("⏱ หมดเวลาใส่ของ → ปิดมอเตอร์")
                    state = STATE_CLOSE_SERVO

            elif state == STATE_MONITOR_MOVEMENT:
                last_motion_time = time.time()
                last_distance = read_sensor(index)
                print("🔁 เริ่มตรวจจับความเคลื่อนไหว...")

                while True:
                    current = read_sensor(index)
                    print(f"🚱 {INDEX_TO_SLOT[index]}: {current:.1f} mm")
                    if abs(current - last_distance) >= CHANGE_THRESHOLD:
                        print("🔍 พบการขยับ → รีเซ็ตตัวจับเวลา")
                        last_motion_time = time.time()
                        last_distance = current
                    if time.time() - last_motion_time >= 3:
                        print("⏳ ไม่มีการขยับนาน 3 วิ → ปิดมอเตอร์")
                        break
                    time.sleep(0.2)

                state = STATE_CLOSE_SERVO

            elif state == STATE_CLOSE_SERVO:
                print(f"🔒 ปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (← 0°)")
                move_servo_180(index, 0)
                capacity = read_sensor(index)
                sensor_exists = index < len(vl53_sensors)
                is_available = capacity > ZERO_THRESHOLD and sensor_exists
                slot_status[index].update(
                    {
                        "capacity_mm": capacity,
                        "available": is_available,
                        "door_closed": True,
                    }
                )
                publish_status_idx(index)
                state = STATE_DONE

        print(f"✅ ช่อง {INDEX_TO_SLOT[index]}: ปิด servo แล้ว")
    except Exception as e:
        print(f"[ERR] Storage_compartment({INDEX_TO_SLOT[index]}): {e}")
    finally:
        reading_active = False

# ===== DOOR UNLOCK SEQUENCE (เอาเอกสารออก) =====
def handle_door_unlock(index: int):
    """
    ปลดล็อก (relay ON) → รอผู้ใช้เปิดประตู → เฝ้าการนำออก + เตือนทุก 10 วิถ้ายังไม่ปิด
    → รอปิดสนิท (debounce) → ล็อก (relay OFF) → อ่านค่า/ประกาศสถานะ
    """
    global reading_active

    # ปรับแต่งเวลา/threshold ได้จากตรงนี้
    TIME_WAIT_OPEN_DOOR = 10
    MOTION_TIMEOUT = 30
    MOTION_INACTIVE_BEFORE_WARN = 5
    TIME_REPEAT_WARNING = 10
    SENSOR_STABLE_DURATION = 1.5
    SENSOR_MOTION_THRESHOLD = CHANGE_THRESHOLD
    SENSOR_CHECK_INTERVAL = 0.2

    try:
        # ปลดล็อก (จ่ายไฟให้โซลินอยด์)
        relay_pins[index].value = True
        print(f"🔓 เปิดประตูช่อง {INDEX_TO_SLOT[index]} (Relay ON)")

        # รอให้ "เปิดประตูจริง" ภายใน 10 วิ
        wait_start = time.time()
        while time.time() - wait_start <= TIME_WAIT_OPEN_DOOR:
            if not is_door_reliably_closed(index):
                publish_warning_idx(index, "ประตูถูกเปิดแล้ว")
                print("✅ ประตูถูกเปิดแล้ว")
                break
            print("⏳ ยังไม่มีการเปิดประตู...")
            time.sleep(SENSOR_CHECK_INTERVAL)
        else:
            relay_pins[index].value = False
            publish_warning_idx(index, "ได้รับคำสั่งปลดล็อก แต่ไม่มีการเปิดประตูภายในเวลาที่กำหนด ระบบได้ทำการล็อกกลับโดยอัตโนมัติ")
            print("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            return

        # เฝ้าดูการนำเอกสารออก + เตือนทุก 10 วิ หากประตูยังไม่ปิด
        extract_start = time.time()
        last_warning_time = 0
        last_motion_time = time.time()
        last_distance = read_sensor(index)
        motion_detected = False

        print("📦 เริ่มตรวจจับการนำเอกสารออก...")
        while True:
            # ถ้าปิดเร็ว ก็ไปล็อกเลย
            if is_door_reliably_closed(index):
                print("🚪 ผู้ใช้ปิดประตูก่อน timeout → ไปล็อก")
                break

            current = read_sensor(index)
            print(f"📉 ความจุปัจจุบัน: {current:.1f} mm")

            if abs(current - last_distance) >= SENSOR_MOTION_THRESHOLD:
                print("🔄 ตรวจพบการเคลื่อนไหว → รีเซ็ตเวลา")
                last_motion_time = time.time()
                last_distance = current
                motion_detected = True

            # ถึงเวลารวมในการนำออก
            if time.time() - extract_start > MOTION_TIMEOUT:
                if not motion_detected:
                    print("⚠️ เปิดประตูแล้วแต่ไม่พบการขยับเลย → แจ้งเตือน")
                    publish_warning_idx(index, "เปิดประตูแล้วแต่ไม่พบการขยับของเลย")
                print("⏳ หมดเวลานำออก → เข้าสู่โหมดรอปิดประตู")
                break

            # ไม่มีการขยับ และยังไม่ปิด → เตือนซ้ำทุก 10 วิ
            if time.time() - last_motion_time > MOTION_INACTIVE_BEFORE_WARN:
                if not is_door_reliably_closed(index) and time.time() - last_warning_time > TIME_REPEAT_WARNING:
                    print("⚠️ ไม่มีการเคลื่อนไหว และประตูยังไม่ปิด → แจ้งเตือนซ้ำ")
                    publish_warning_idx(index, "กรุณาปิดประตูให้สนิทเพื่อทำการล็อก")
                    last_warning_time = time.time()

            time.sleep(SENSOR_CHECK_INTERVAL)

        # รอให้ "ปิดสนิทคงที่" ก่อนสั่งล็อก
        print(f"⏳ ตรวจสอบซ้ำว่าประตูปิดสนิท รอ {SENSOR_STABLE_DURATION:.1f} วินาที...")
        while not is_door_reliably_closed(index):
            # เตือนซ้ำทุก 10 วิ ถ้ายังไม่ปิด
            if time.time() - last_warning_time > TIME_REPEAT_WARNING:
                publish_warning_idx(index, "ประตูยังไม่ปิดสนิท กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()
            time.sleep(SENSOR_CHECK_INTERVAL)
        time.sleep(SENSOR_STABLE_DURATION)

        # สั่ง "ล็อก" (ตัดไฟโซลินอยด์)
        relay_pins[index].value = False
        publish_warning_idx(index, "ประตูถูกล็อกแล้ว")
        print(f"🔐 ล็อกประตูช่อง {INDEX_TO_SLOT[index]} (Relay OFF)")

        # ความปลอดภัย: ตรวจซ้ำว่าปิดจริง
        time.sleep(0.5)
        if not is_door_reliably_closed(index):
            print("⚠️ ตรวจพบว่าประตูยังไม่ปิดสนิทหลังล็อก → แจ้งเตือน")
            publish_warning_idx(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
            return

        # รอค่าวัดนิ่ง แล้วค่อยส่งสถานะ
        print("📏 รอค่าวัดความจุนิ่งก่อนส่งออก...")
        stable_start = time.time()
        stable_value = read_sensor(index)
        while True:
            current = read_sensor(index)
            if abs(current - stable_value) < 1:
                if time.time() - stable_start >= 2:
                    break
            else:
                stable_start = time.time()
                stable_value = current
            time.sleep(0.2)

        new_value = read_sensor(index)
        slot_status[index]["capacity_mm"] = new_value
        slot_status[index]["door_closed"] = True
        slot_status[index]["available"] = new_value > ZERO_THRESHOLD
        publish_status_idx(index)

    except Exception as e:
        print(f"[ERR] handle_door_unlock({INDEX_TO_SLOT[index]}): {e}")
    finally:
        reading_active = False

# === alias เพื่อความเข้ากันได้กับโค้ดเก่า ===
run_state_machine = Storage_compartment

# ===== MQTT LISTENER (เบา: โยนเข้าคิว) =====

def on_message(client, userdata, msg):
    try:
        parts = msg.topic.split("/")  # smartlocker/{node}/slot/SC00+{slot_id}/command/{action}
        if len(parts) < 6 or parts[0] != "smartlocker":
            return
        node, slot_id, action = parts[1], parts[3], parts[5]
        data = json.loads(msg.payload.decode("utf-8") or "{}") if msg.payload else {}
        role = str(data.get("role", "student")).lower()

        if slot_id not in SLOT_TO_INDEX:
            print(f"❌ Unknown slot_id: {slot_id}")
            return

        # เช็คสิทธิ์แบบง่าย
        if not is_valid_role(role):
            print(f"❌ Invalid role: {role}")
            return
        if action == "intake" and not can_open_slot(role):
            print(f"🚫 role '{role}' ไม่มีสิทธิ์ {action}")
            return
        if action == "removal" and not can_open_door(role):
            print(f"🚫 role '{role}' ไม่มีสิทธิ์ {action}")
            return

        cmd_q.put((action, slot_id, role))
        print(f"[ENQUEUE] {action} {slot_id} by {role}")
    except Exception as e:
        print("[on_message ERR]", e)

# ===== MQTT CONNECTOR =====
def on_connect(c, u, f, rc, props=None):
    for t in get_subscriptions(broad=True):  # บริหารง่ายสุด: สมัครกว้าง
        c.subscribe(t, qos=1)
        print("[MQTT] Subscribed:", t)

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
        try: client.ws_set_options(path="/mqtt")
        except: pass
    if use_tls:
        if ca:
            client.tls_set(ca_certs=ca, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        else:
            client.tls_set_context(ssl.create_default_context())
        client.tls_insecure_set(False)
        if port == 1883: port = 8883

    client.on_connect = on_connect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=16)
    client.connect(host, port, keepalive=60)
    return client

# ===== MAIN =====
def main():
    # Init hardware (จากเวอร์ชันแรก)
    init_mcp()
    init_xshuts()
    reset_vl53_addresses()
    init_sensors()

    # Initial status broadcast
    for i, sid in enumerate(SLOT_IDS):
        slot_status[i] = {"capacity_mm": 200 + i, "available": True, "door_closed": True}
        publish_status(sid, slot_status[i])
        time.sleep(0.05)

    # Start worker
    threading.Thread(target=worker, daemon=True).start()

    # Start MQTT loop (จัดการง่าย)
    client = build_client()
    client.loop_forever()

# ===== WORKER THREAD =====
def worker():
    global reading_active, selected_sensor_index, user_role
    while True:
        action, slot_id, role = cmd_q.get()
        try:
            if reading_active:
                publish_warning(slot_id, "ระบบกำลังทำงานอยู่ โปรดลองใหม่อีกครั้ง")
                continue
            if slot_id not in SLOT_TO_INDEX:
                publish_warning(slot_id, f"ไม่รู้จัก slot_id: {slot_id}")
                continue

            idx = SLOT_TO_INDEX[slot_id]
            selected_sensor_index = idx
            user_role = role
            reading_active = True

            try:
                if action == "slot":
                    Storage_compartment(idx)
                elif action == "door":
                    handle_door_unlock(idx)
                else:
                    publish_warning(slot_id, f"ไม่รู้จักคำสั่ง: {action}")
            finally:
                reading_active = False
        except Exception as e:
            publish_warning(slot_id, "เกิดข้อผิดพลาด", extra={"detail": str(e)})
        finally:
            cmd_q.task_done()

if __name__ == "__main__":
    main()
