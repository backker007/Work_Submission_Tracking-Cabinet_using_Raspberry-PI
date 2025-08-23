# ✅ controller/main_controller.py
# SmartLocker Controller รองรับ Multi-Node + คิวคำสั่ง + ทำงานร่วมกับ helpers

import sys
import os
import time
import json
import threading
import queue
from pathlib import Path

# ---- Project root to sys.path ----
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# ---- Load .env ----
from dotenv import load_dotenv
ROOT = Path(__file__).resolve().parents[1]
load_dotenv(ROOT / ".env")

# (debug) แสดงคอนฟิก MQTT ที่อ่านได้
print(
    "[MQTT cfg]",
    os.getenv("MQTT_HOST"),
    os.getenv("MQTT_PORT"),
    "TLS=" + str(os.getenv("MQTT_TLS")),
    "USER=" + str(os.getenv("MQTT_USERNAME")),
)

# ---- Imports from shared modules ----
from shared.mqtt_helpers import (
    publish_status,
    publish_warning,
    get_command_subscriptions,
)
from shared.hardware_helpers import (
    init_mcp,
    init_xshuts,
    reset_vl53_addresses,
    init_sensors,
    read_sensor,
    move_servo_180,
    is_door_reliably_closed,
    vl53_sensors,
    mcp_pins,
    relay_pins,
    CHANGE_THRESHOLD,  # threshold การเปลี่ยนแปลงระยะ (mm)
)
from shared.role_helpers import can_insert, can_unlock, is_valid_role

# ==== CONFIG ====
NODE_ID = os.getenv("NODE_ID", "C01")
CHECK_INTERVAL = int(os.getenv("CHECK_INTERVAL", "120"))
ZERO_THRESHOLD = int(os.getenv("ZERO_THRESHOLD", "70"))  # >70mm ถือว่ายังว่าง (ปรับตามจริง)

# สถานะช่อง (0..3)
slot_status = [{"capacity_mm": 0, "available": True, "door_closed": True} for _ in range(4)]

# ตัวกันซ้อนงานระดับระบบ (ถ้าต้องการกันต่อช่อง เปลี่ยนเป็น busy[4] ได้)
reading_active = False

# role และ index ล่าสุด (debug/อ้างอิง)
selected_sensor_index = None
user_role = None

# คิวคำสั่งจาก MQTT → worker
cmd_q = queue.Queue()


# ==== MAIN LOOP ====
def main():
    # Init hardware
    init_mcp()
    init_xshuts()
    reset_vl53_addresses()
    init_sensors()

    # Initial status broadcast
    for i in range(len(slot_status)):
        slot_status[i] = {"capacity_mm": 240 + i, "available": True, "door_closed": True}
        publish_status_slot(i)
        time.sleep(0.1)

    # Start MQTT
    client = _build_mqtt_client()
    client.loop_start()

    # Start command worker (สำคัญ: ต้องเป็น thread, ห้ามเรียกฟังก์ชันตรง!)
    threading.Thread(target=command_worker, daemon=True).start()

    last_check = time.time()

    try:
        while True:
            # เช็คไฟ/LED ทุก ๆ CHECK_INTERVAL
            if time.time() - last_check >= CHECK_INTERVAL:
                for i in range(len(vl53_sensors)):
                    value = read_sensor(i)

                    # NOTE: ตรวจ mapping ของบอร์ดคุณให้ถูก
                    green = mcp_pins[i]        # สมมติ LED เขียว
                    red = mcp_pins[8 + i]      # สมมติ LED แดง
                    # หลีกเลี่ยงการไปยุ่ง input ของสวิตช์ประตู

                    if value == -1:           # อ่านไม่ได้/ชิดเกินไป
                        green.value = False
                        red.value = True
                    elif value == 0:          # (ปรับตามโปรเจกต์ของคุณ)
                        green.value = True
                        red.value = False
                    else:
                        green.value = False
                        red.value = True

                last_check = time.time()

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n🛑 หยุดการทำงาน")
    finally:
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass


# ==== MQTT STATUS WRAPPER ====
def publish_status_slot(index: int):
    """ส่งสถานะของช่อง index (0-based) ออก MQTT"""
    publish_status(NODE_ID, index + 1, slot_status[index])


def publish_warning_slot(index: int, message: str = None):
    """ส่งคำเตือนของช่อง index (0-based) ออก MQTT"""
    publish_warning(
        NODE_ID,
        index + 1,
        message or "ประตูยังไม่ปิดสนิทหลังการเปิดใช้งาน",
    )


# ==== STORAGE COMPARTMENT STATE MACHINE (ใส่เอกสาร) ====
def Storage_compartment(index: int):
    """
    เปิด servo → รอผู้ใช้ 'ใส่ของ' → เฝ้าการขยับจนเงียบ → ปิด servo → อ่านค่า/ประกาศสถานะ
    (logic ตามที่คุณยืนยันแล้ว)
    """
    global reading_active

    STATE_WAIT_INSERT = "wait_insert"
    STATE_MONITOR_MOVEMENT = "monitor_movement"
    STATE_CLOSE_SERVO = "close_servo"
    STATE_DONE = "done"

    try:
        move_servo_180(index, 180)
        print(f"🔄 เปิดมอเตอร์ช่อง {index + 1} (→ 180°)")

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
                    print(f"⏳ รอการใส่ของ... S{index + 1}: {current:.1f} mm")
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
                    print(f"🚱 S{index + 1}: {current:.1f} mm")
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
                print(f"🔒 ปิดมอเตอร์ช่อง {index + 1} (← 0°)")
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
                publish_status_slot(index)
                state = STATE_DONE

        print(f"✅ ช่อง {index + 1}: ปิด servo แล้ว")
    except Exception as e:
        print(f"[ERR] Storage_compartment({index}): {e}")
    finally:
        reading_active = False


# ==== DOOR UNLOCK SEQUENCE (เอาเอกสารออก) ====
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
        print(f"🔓 เปิดประตูช่อง {index + 1} (Relay ON)")

        # รอให้ "เปิดประตูจริง" ภายใน 10 วิ
        wait_start = time.time()
        while time.time() - wait_start <= TIME_WAIT_OPEN_DOOR:
            if not is_door_reliably_closed(index):
                print("✅ ประตูถูกเปิดแล้ว")
                break
            print("⏳ ยังไม่มีการเปิดประตู...")
            time.sleep(SENSOR_CHECK_INTERVAL)
        else:
            print("⚠️ ครบเวลาแต่ยังไม่เปิดประตู → ล็อกกลับทันที")
            relay_pins[index].value = False
            publish_warning_slot(
                index,
                "ได้รับคำสั่งปลดล็อก แต่ไม่มีการเปิดประตูภายในเวลาที่กำหนด ระบบได้ทำการล็อกกลับโดยอัตโนมัติ",
            )
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
                    publish_warning_slot(index, "เปิดประตูแล้วแต่ไม่พบการขยับของเลย")
                print("⏳ หมดเวลานำออก → เข้าสู่โหมดรอปิดประตู")
                break

            # ไม่มีการขยับ และยังไม่ปิด → เตือนซ้ำทุก 10 วิ
            if time.time() - last_motion_time > MOTION_INACTIVE_BEFORE_WARN:
                if not is_door_reliably_closed(index) and time.time() - last_warning_time > TIME_REPEAT_WARNING:
                    print("⚠️ ไม่มีการเคลื่อนไหว และประตูยังไม่ปิด → แจ้งเตือนซ้ำ")
                    publish_warning_slot(index, "กรุณาปิดประตูให้สนิทเพื่อทำการล็อก")
                    last_warning_time = time.time()

            time.sleep(SENSOR_CHECK_INTERVAL)

        # รอให้ "ปิดสนิทคงที่" ก่อนสั่งล็อก
        print(f"⏳ ตรวจสอบซ้ำว่าประตูปิดสนิท รอ {SENSOR_STABLE_DURATION:.1f} วินาที...")
        while not is_door_reliably_closed(index):
            # เตือนซ้ำทุก 10 วิ ถ้ายังไม่ปิด
            if time.time() - last_warning_time > TIME_REPEAT_WARNING:
                publish_warning_slot(index, "ประตูยังไม่ปิดสนิท กรุณาปิดให้สนิทเพื่อทำการล็อก")
                last_warning_time = time.time()
            time.sleep(SENSOR_CHECK_INTERVAL)
        time.sleep(SENSOR_STABLE_DURATION)

        # สั่ง "ล็อก" (ตัดไฟโซลินอยด์)
        relay_pins[index].value = False
        print(f"🔐 ล็อกประตูช่อง {index + 1} (Relay OFF)")

        # ความปลอดภัย: ตรวจซ้ำว่าปิดจริง
        time.sleep(0.5)
        if not is_door_reliably_closed(index):
            print("⚠️ ตรวจพบว่าประตูยังไม่ปิดสนิทหลังล็อก → แจ้งเตือน")
            publish_warning_slot(index, "ระบบพยายามล็อกแล้ว แต่ประตูยังไม่ปิดสนิท")
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
            time.sleep(SENSOR_CHECK_INTERVAL)

        new_value = read_sensor(index)
        slot_status[index]["capacity_mm"] = new_value
        slot_status[index]["door_closed"] = True
        slot_status[index]["available"] = new_value > ZERO_THRESHOLD
        publish_status_slot(index)

    except Exception as e:
        print(f"[ERR] handle_door_unlock({index}): {e}")
    finally:
        reading_active = False


# alias ให้โค้ดเก่าที่อาจเรียกชื่อเดิม
run_state_machine = Storage_compartment


# ==== MQTT LISTENER (เบาและเร็ว: โยนคิวอย่างเดียว) ====
def on_message(client, userdata, msg):
    try:
        topic = msg.topic
        payload = msg.payload.decode("utf-8") if msg.payload else "{}"
        data = {}
        try:
            data = json.loads(payload) if payload else {}
        except Exception:
            pass

        # topic format: smartlocker/{NODE}/slot/{N}/command/{kind}
        parts = topic.split("/")
        if len(parts) >= 6 and parts[0] == "smartlocker":
            node = parts[1]
            slot = int(parts[3])  # 1-based
            kind = parts[5]       # "open" | "unlock"
            role = data.get("role", "student")

            if not is_valid_role(role):
                print(f"❌ Invalid role: {role}")
                return

            # ตรวจสิทธิ์ตาม kind
            if kind == "open" and not can_insert(role):
                print(f"🚫 role '{role}' ไม่มีสิทธิ์ {kind}")
                return
            if kind == "unlock" and not can_unlock(role):
                print(f"🚫 role '{role}' ไม่มีสิทธิ์ {kind}")
                return

            # โยนงานเข้า queue (ไม่ทำงานยาวใน callback)
            cmd_q.put((kind, node, slot, role))
            print(f"[ENQUEUE] {kind} node={node} slot={slot} role={role}")
        else:
            print(f"[MQTT] skip topic: {topic}")
    except Exception as e:
        print(f"[on_message ERR] {e}")


# ==== MQTT CONNECTOR (Subscriber) ====
def _build_mqtt_client():
    import ssl
    import paho.mqtt.client as mqtt

    host = os.getenv("MQTT_HOST", "localhost")
    port = int(os.getenv("MQTT_PORT", "1883"))
    username = os.getenv("MQTT_USERNAME")
    password = os.getenv("MQTT_PASSWORD")
    use_tls = bool(int(os.getenv("MQTT_TLS", "0")))
    use_ws = bool(int(os.getenv("MQTT_WS", "0")))
    client_id = os.getenv("MQTT_CLIENT_ID")
    ca_path = os.getenv("MQTT_CA")  # optional

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=client_id,
        transport="websockets" if use_ws else "tcp",
        protocol=mqtt.MQTTv311,
    )

    if username:
        client.username_pw_set(username=username, password=password)

    if use_ws:
        # สำคัญสำหรับ HiveMQ Cloud (WebSocket path)
        try:
            client.ws_set_options(path="/mqtt")
        except Exception:
            pass

    if use_tls:
        if ca_path:
            client.tls_set(ca_certs=ca_path, tls_version=ssl.PROTOCOL_TLS_CLIENT)
        else:
            import ssl as _ssl
            client.tls_set_context(_ssl.create_default_context())
        client.tls_insecure_set(False)
        if port == 1883:
            port = 8883  # default TLS port ifไม่ระบุ

    client.on_message = on_message

    # resubscribe เมื่อ reconnect
    def _on_connect(_c, _u, _f, rc, props=None):
        topics = list(get_command_subscriptions())
        for t in topics:
            _c.subscribe(t)
            print(f"[MQTT] Subscribed: {t}")
    client.on_connect = _on_connect

    # ตั้ง backoff reconnect
    try:
        client.reconnect_delay_set(min_delay=1, max_delay=16)
    except Exception:
        pass

    client.connect(host, port, keepalive=60)
    return client


# ==== WORKER THREAD ====
def command_worker():
    while True:
        try:
            kind, node, slot, role = cmd_q.get()
            global reading_active, selected_sensor_index, user_role

            # กันซ้อนทั้งระบบ; ถ้าต้องการทีละช่อง ใช้ busy[slot-1] แทน
            if reading_active:
                print("⚠️ กำลังมีการทำงานอยู่ รอให้เสร็จก่อน")
                continue

            selected_sensor_index = slot - 1
            user_role = role
            reading_active = True

            try:
                if kind == "open":
                    Storage_compartment(slot - 1)  # index ภายในเริ่ม 0
                elif kind == "unlock":
                    handle_door_unlock(slot - 1)
                else:
                    print(f"❓ ไม่รู้จักคำสั่ง: {kind}")
            finally:
                # NOTE: ทั้งสองฟังก์ชันก็มี finally เคลียร์อยู่แล้ว
                reading_active = False

        except Exception as e:
            print(f"[ERR] worker: {e}")
        finally:
            try:
                cmd_q.task_done()
            except Exception:
                pass




if __name__ == "__main__":
    main()
