# ✅ controller/main_controller.py
# SmartLocker Controller ที่รองรับ Multi-Node และใช้ helper แยกโมดูลครบ

import sys
import os
import threading
import time
import json
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from shared.mqtt_helpers import publish_status, publish_warning, get_command_subscriptions
from shared.hardware_helpers import (
    init_mcp, init_xshuts, reset_vl53_addresses, init_sensors,
    read_sensor, move_servo_180, is_door_reliably_closed,
    vl53_sensors, mcp_pins, relay_pins
)
from shared.role_helpers import can_insert, can_unlock, is_valid_role

# ==== CONFIGURATION ====
NODE_ID = "C01"
CHECK_INTERVAL = 120
slot_status = [{"capacity_mm": 0, "available": True, "door_closed": True} for _ in range(4)]
reading_active = False
selected_sensor_index = None
user_role = None

# ==== MQTT STATUS WRAPPER ====
def publish_status_slot(index):
    publish_status(NODE_ID, index + 1, slot_status[index])

def publish_warning_slot(index, message=None):
    publish_warning(NODE_ID, index + 1, message or "ประตูยังไม่ปิดสนิทหลังการเปิดใช้งาน")

# ==== MQTT LISTENER ====
def on_message(client, userdata, msg):
    global selected_sensor_index, reading_active, user_role

    topic = msg.topic
    payload = json.loads(msg.payload.decode())

    print(f"[MQTT] Received → {topic}: {payload}")

    try:
        slot = int(topic.split("/")[3]) - 1
        role = payload.get("role", "")
    except (IndexError, ValueError):


        print(f"❌ Invalid topic or payload: {topic}")
        
        
        return

    if not is_valid_role(role):
        print(f"❌ Invalid role: {role}")
        return

    if slot < 0 or slot >= len(slot_status):
        print(f"❌ Invalid slot number: {slot + 1}")
        return

    if reading_active:
        print("⚠️ กำลังมีการทำงานอยู่ รอให้เสร็จให้เสร็จก่อน")
        return

    if topic.endswith("/open") and can_insert(role):
        print(f"🔓 รับคำสั่งเปิดช่อง {slot + 1} โดย {role}")
        selected_sensor_index = slot
        user_role = role
        reading_active = True
        threading.Thread(target=run_state_machine, args=(slot,), daemon=True).start()

    elif topic.endswith("/unlock") and can_unlock(role):
        print(f"🛠 รับคำสั่งเปิดประตูช่อง {slot + 1} โดย {role}")
        selected_sensor_index = slot
        user_role = role
        reading_active = True
        threading.Thread(target=handle_door_unlock, args=(slot,), daemon=True).start()

    else:
        print(f"⚠️ คำสั่งไม่รองรับ หรือ role ไม่มีสิทธิ์: {topic}, role: {role}")

# ==== STATE LOGIC ====
def run_state_machine(index):
    global reading_active

    print(f"🔄 เปิดช่อง {index + 1}")
    move_servo_180(index, 180)
    time.sleep(10)  # ตัวอย่าง: ให้เวลาผู้ใช้ใส่ของ
    move_servo_180(index, 0)

    capacity = read_sensor(index)
    is_available = capacity > 70 and index < len(vl53_sensors)
    slot_status[index].update({
        "capacity_mm": capacity,
        "available": is_available,
        "door_closed": True
    })
    publish_status_slot(index)
    reading_active = False


def handle_door_unlock(index):
    global reading_active

    relay_pins[index].value = True
    print(f"🔓 เปิดประตูช่อง {index + 1} (Relay ON)")
    timeout = time.time() + 30

    while time.time() < timeout:
        if not is_door_reliably_closed(index):
            print("✅ ประตูถูกเปิดแล้ว")
            break
        time.sleep(0.1)
    else:
        print("⚠️ ไม่เปิดประตูภายในเวลา → ล็อกกลับ")
        relay_pins[index].value = False
        publish_warning_slot(index, "ไม่มีการเปิดประตูภายในเวลาที่กำหนด")
        reading_active = False
        return

    while not is_door_reliably_closed(index):
        time.sleep(0.2)

    relay_pins[index].value = False
    print(f"🔐 ล็อกประตูช่อง {index + 1} (Relay OFF)")

    capacity = read_sensor(index)
    slot_status[index].update({
        "capacity_mm": capacity,
        "available": capacity > 70,
        "door_closed": True
    })
    publish_status_slot(index)
    reading_active = False

# ==== MAIN LOOP ====
def main():
    init_mcp()
    init_xshuts()
    reset_vl53_addresses()
    init_sensors()
    # ✅ INITIAL STATUS BROADCAST
    for i in range(len(slot_status)):
        slot_status[i] = {
            "capacity_mm": 240,
            "available": True,
            "door_closed": True
        }
        publish_status_slot(i)
        time.sleep(0.1)
    last_check = time.time()

    import paho.mqtt.client as mqtt
    client = mqtt.Client()
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    for topic in get_command_subscriptions():
        client.subscribe(topic)
    client.loop_start()

    try:
        while True:
            if time.time() - last_check >= CHECK_INTERVAL:
                for i in range(len(vl53_sensors)):
                    value = read_sensor(i)
                    green = mcp_pins[i]
                    red = mcp_pins[8 + i]
                    if value == -1:
                        green.value = False
                        red.value = True
                    elif value == 0:
                        green.value = True
                        red.value = False
                    else:
                        green.value = False
                        red.value = True
                last_check = time.time()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n🛑 หยุดการทำงาน")

if __name__ == "__main__":
    main()
