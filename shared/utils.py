# ✅ shared/utils.py
# 📡 ฟังก์ชันพื้นฐานสำหรับ publish MQTT แบบ low-level พร้อม retry

import time
import json
import paho.mqtt.client as mqtt

# ==== MQTT CORE ====
def publish_mqtt(topic, payload, broker="localhost", port=1883, retry=3):
    """
    ส่งข้อมูล MQTT ไปยัง broker ที่ระบุ พร้อม retry หากล้มเหลว

    Args:
        topic (str): หัวข้อ MQTT เช่น "smartlocker/C01/slot/1/status"
        payload (dict): ข้อมูลที่ต้องการส่ง (จะถูกแปลงเป็น JSON)
        broker (str): ที่อยู่ MQTT broker (default: localhost)
        port (int): พอร์ต MQTT (default: 1883)
        retry (int): จำนวนครั้งสูงสุดในการ retry หากส่งไม่สำเร็จ
    """
    for attempt in range(retry):
        try:
            client = mqtt.Client()
            client.connect(broker, port, 60)

            client.loop_start()
            client.publish(topic, json.dumps(payload))
            time.sleep(0.5)
            client.loop_stop()

            client.disconnect()
            print(f"[MQTT] Published to {topic} → {payload}")
            return
        except Exception as e:
            print(f"❌ MQTT Publish Error (attempt {attempt+1}): {e}")
            time.sleep(2)
