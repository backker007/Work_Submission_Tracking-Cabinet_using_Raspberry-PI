# controller/main_controller.py
from __future__ import annotations
import os, sys, json, time, threading, ssl
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
# cmd_q: "queue.Queue[tuple[str,str,str]]" = queue.Queue()   # (action, slot_id, role)
# reading_active = False
selected_sensor_index: int | None = None
user_role: str | None = None
slot_status = [{"capacity_mm": 0, "available": True, "is_open": False} for _ in SLOT_IDS]
# --- NEW: per-slot locks สำหรับกันคำสั่งซ้อนใน "ช่องเดียวกัน"
slot_locks: dict[str, threading.Lock] = {sid: threading.Lock() for sid in SLOT_IDS}
# --- NEW: I2C bus lock (VL53L0X / PCA9685 / MCP23017 ใช้บัสเดียวกัน → ต้องกันชน)
i2c_lock = threading.RLock()
# ===== TIMING (ปรับผ่าน ENV ได้) =====
ACTIVE_CHECK_INTERVAL = float(os.getenv("ACTIVE_CHECK_INTERVAL", "0.10"))  # ช่วงรอ event/monitor
IDLE_CHECK_INTERVAL   = float(os.getenv("IDLE_CHECK_INTERVAL",   "0.20"))  # ช่วงทั่วไป

# ===== I²C SHORT-LOCK HELPERS =====
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
        relay_pins[index].value = value


# ===== MQTT helpers for idx =====
def publish_status_idx(idx: int):
    sid = INDEX_TO_SLOT[idx]
    publish_status(sid, slot_status[idx])

def publish_warning_idx(idx: int, message: str):
    sid = INDEX_TO_SLOT[idx]
    publish_warning(sid, message)

# ===== STORAGE COMPARTMENT STATE MACHINE (ใส่เอกสาร) =====

def Storage_compartment(index: int):
    try:
        # เปิดฝากระท่องาน (ล็อกเฉพาะตอนสั่งจริง)
        i2c_move_servo_180(index, 180)
        print(f"🔄 เปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (→ 180°)")

        # อ่านค่าเริ่มต้น (retry ภายใน 5 วินาที)
        initial = -1
        timeout = time.time() + 5
        while initial <= 0 and time.time() < timeout:
            initial = i2c_read_sensor(index)
            time.sleep(ACTIVE_CHECK_INTERVAL)

        state = "wait_insert" if initial > 0 else "close_servo"
        if initial <= 0:
            print("❌ อ่านค่าเริ่มต้นไม่สำเร็จ → ปิดมอเตอร์")

        while state != "done":
            if state == "wait_insert":
                timeout = time.time() + 10
                while time.time() < timeout:
                    current = i2c_read_sensor(index)
                    print(f"⏳ รอการใส่ของ... {INDEX_TO_SLOT[index]}: {current:.1f} mm")
                    if current > 0 and current < initial - CHANGE_THRESHOLD:
                        print("📦 ตรวจพบการใส่ของครั้งแรก")
                        state = "monitor_movement"
                        break
                    time.sleep(ACTIVE_CHECK_INTERVAL)
                else:
                    print("⏱ หมดเวลาใส่ของ → ปิดมอเตอร์")
                    state = "close_servo"

            elif state == "monitor_movement":
                last_motion_time = time.time()
                last_distance = i2c_read_sensor(index)
                print("🔁 เริ่มตรวจจับความเคลื่อนไหว...")
                while True:
                    current = i2c_read_sensor(index)
                    print(f"🚱 {INDEX_TO_SLOT[index]}: {current:.1f} mm")
                    if abs(current - last_distance) >= CHANGE_THRESHOLD:
                        print("🔍 พบการขยับ → รีเซ็ตตัวจับเวลา")
                        last_motion_time = time.time()
                        last_distance = current
                    if time.time() - last_motion_time >= 3:
                        print("⏳ ไม่มีการขยับนาน 3 วิ → ปิดมอเตอร์")
                        break
                    time.sleep(ACTIVE_CHECK_INTERVAL)
                state = "close_servo"

            elif state == "close_servo":
                print(f"🔒 ปิดมอเตอร์ช่อง {INDEX_TO_SLOT[index]} (← 0°)")
                i2c_move_servo_180(index, 0)
                capacity = i2c_read_sensor(index)
                sensor_exists = index < len(vl53_sensors)
                is_available = capacity > ZERO_THRESHOLD and sensor_exists
                is_open = not i2c_is_door_closed(index)
                slot_status[index].update({
                    "capacity_mm": capacity,
                    "available": is_available,
                    "is_open": is_open
                })
                publish_status_idx(index)
                state = "done"

        print(f"✅ ช่อง {INDEX_TO_SLOT[index]}: ปิด servo แล้ว")
    except Exception as e:
        print(f"[ERR] Storage_compartment({INDEX_TO_SLOT[index]}): {e}")


# ===== DOOR UNLOCK SEQUENCE (เอาเอกสารออก) =====
def handle_door_unlock(index: int):
    try:
        i2c_set_relay(index, True)  # Relay ON
        print(f"🔓 เปิดประตูช่อง {INDEX_TO_SLOT[index]} (Relay ON)")

        # รอให้ผู้ใช้เปิดจริงภายใน 10 วิ
        wait_start = time.time()
        while time.time() - wait_start <= 10:
            closed = i2c_is_door_closed(index)
            if not closed:
                slot_status[index]["is_open"] = True  # เปิดแล้ว
                publish_status_idx(index)
                publish_warning_idx(index, "ประตูถูกเปิดแล้ว")
                print("✅ ประตูถูกเปิดแล้ว")
                break
            print("⏳ ยังไม่มีการเปิดประตู...")
            time.sleep(ACTIVE_CHECK_INTERVAL)
        else:
            i2c_set_relay(index, False)  # Relay OFF
            publish_warning_idx(index, "ได้รับคำสั่งปลดล็อก แต่ไม่มีการเปิดประตูภายในเวลาที่กำหนด ระบบได้ทำการล็อกกลับโดยอัตโนมัติ")
            print("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            return

        # เฝ้าการนำเอกสารออก
        extract_start = time.time()
        last_warning_time = 0.0
        last_motion_time = time.time()
        last_distance = i2c_read_sensor(index)
        motion_detected = False

        print("📦 เริ่มตรวจจับการนำเอกสารออก...")
        while True:
            if i2c_is_door_closed(index):
                print("🚪 ผู้ใช้ปิดประตูก่อน timeout → ไปล็อก")
                break

            current = i2c_read_sensor(index)
            print(f"📉 ความจุปัจจุบัน: {current:.1f} mm")

            if abs(current - last_distance) >= CHANGE_THRESHOLD:
                print("🔄 ตรวจพบการเคลื่อนไหว → รีเซ็ตเวลา")
                last_motion_time = time.time()
                last_distance = current
                motion_detected = True

            if time.time() - extract_start > 30:
                if not motion_detected:
                    print("⚠️ เปิดประตูแล้วแต่ไม่พบการขยับเลย → แจ้งเตือน")
                    publish_warning_idx(index, "เปิดประตูแล้วแต่ไม่พบการขยับของเลย")
                print("⏳ หมดเวลานำออก → เข้าสู่โหมดรอปิดประตู")
                break

            if time.time() - last_motion_time > 5:
                if not i2c_is_door_closed(index) and time.time() - last_warning_time > 120:
                    print("⚠️ ไม่มีการเคลื่อนไหว และประตูยังไม่ปิด → แจ้งเตือนซ้ำ")
                    publish_warning_idx(index, "กรุณาปิดประตูให้สนิทเพื่อทำการล็อก")
                    last_warning_time = time.time()

            time.sleep(ACTIVE_CHECK_INTERVAL)

        # รอให้ปิดสนิท แล้วค้าง 1.5s เพื่อความนิ่ง
        print(f"⏳ ตรวจสอบซ้ำว่าประตูปิดสนิท รอ 1.5 วินาที...")
        while True:
            if i2c_is_door_closed(index):
                break
            if time.time() - last_warning_time > 10:
                publish_warning_idx(index, "ประตูยังไม่ปิดสนิท กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()
            time.sleep(ACTIVE_CHECK_INTERVAL)
        time.sleep(1.5)

        # ล็อก (Relay OFF)
        i2c_set_relay(index, False)
        publish_warning_idx(index, "ประตูถูกล็อกแล้ว")
        print(f"🔐 ล็อกประตูช่อง {INDEX_TO_SLOT[index]} (Relay OFF)")

        time.sleep(0.5)
        if not i2c_is_door_closed(index):
            publish_warning_idx(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
            return

        # อ่านค่าความจุแบบนิ่งก่อนส่งออก
        print("📏 รอค่าวัดความจุนิ่งก่อนส่งออก...")
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
        slot_status[index]["available"] = new_value > ZERO_THRESHOLD
        publish_status_idx(index)

    except Exception as e:
        print(f"[ERR] handle_door_unlock({INDEX_TO_SLOT[index]}): {e}")


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
        if not is_valid_role(role):
            print(f"❌ Invalid role: {role}")
            return

        # สตาร์ตงานแบบต่อช่อง (ข้ามคิว)
        print(f"[START] {action} {slot_id} by {role}")
        start_slot_task(action, slot_id, role)

    except Exception as e:
        print("[on_message ERR]", e)

def start_slot_task(action: str, slot_id: str, role: str):
    # ตรวจสิทธิ์ขั้นสุดท้ายก่อนเริ่ม (กัน edge case)
    if action == "slot" and not can_open_slot(role):
        publish_warning(slot_id, f"🚫 role '{role}' ไม่มีสิทธิ์ {action}")
        return
    if action == "door" and not can_open_door(role):
        publish_warning(slot_id, f"🚫 role '{role}' ไม่มีสิทธิ์ {action}")
        return

    idx = SLOT_TO_INDEX[slot_id]
    lock = slot_locks[slot_id]
    if not lock.acquire(blocking=False):
        print(f"🗑️ DROP: {slot_id} is busy")
        publish_warning(slot_id, "ช่องนี้กำลังทำงานอยู่ คำสั่งถูกละทิ้ง")
        return

    def run():
        try:
            if action == "slot":
                Storage_compartment(idx)
            elif action == "door":
                handle_door_unlock(idx)
            else:
                publish_warning(slot_id, f"ไม่รู้จักคำสั่ง: {action}")
        except Exception as e:
            publish_warning(slot_id, "เกิดข้อผิดพลาด", extra={"detail": str(e)})
        finally:
            lock.release()

    threading.Thread(target=run, daemon=True).start()

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
        slot_status[i] = {"capacity_mm": 200 + i, "available": True, "is_open": False}
        publish_status(sid, slot_status[i])
        time.sleep(0.05)

    # Start MQTT loop (จัดการง่าย)
    client = build_client()
    client.loop_forever()


if __name__ == "__main__":
    main()
