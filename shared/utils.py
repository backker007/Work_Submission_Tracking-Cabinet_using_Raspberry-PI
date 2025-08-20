# ✅ shared/utils.py
# 📡 MQTT helper (รองรับ paho-mqtt v2, แปลง payload เป็น JSON อัตโนมัติ)

import time
import json
import paho.mqtt.client as mqtt

def publish_mqtt(topic, payload, broker="localhost", port=1883, retry=3, qos=0, retain=False):
    # แปลง payload ให้ถูกชนิดก่อนส่ง
    if isinstance(payload, (dict, list)):
        payload = json.dumps(payload, ensure_ascii=False)
    elif not isinstance(payload, (str, bytes, bytearray, int, float, type(None))):
        payload = str(payload)

    for attempt in range(1, retry + 1):
        try:
            # ใช้ Callback API รุ่นใหม่ของ paho-mqtt v2
            client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
            client.connect(broker, port, keepalive=60)
            client.loop_start()
            res = client.publish(topic, payload, qos=qos, retain=retain)
            try:
                res.wait_for_publish(timeout=2)
            except Exception:
                pass
            client.loop_stop()
            client.disconnect()
            print(f"[MQTT] Published to {topic} → {payload}")
            return True
        except Exception as e:
            print(f"✗ MQTT Error (attempt {attempt}/{retry}): {e}")
            time.sleep(1)
    return False