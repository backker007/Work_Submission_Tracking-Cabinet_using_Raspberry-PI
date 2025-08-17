# utils.py
# ตั้งค่าตัวแปรสำหรับ MQTT broker
BROKER_URL = "broker.emqx.io"  # หรือใส่ URL broker ที่คุณใช้
BROKER_PORT = 1883  # พอร์ต default ของ MQTT
BROKER_KEEP_ALIVE = 60  # ค่า keep-alive สำหรับ MQTT

def publish_mqtt(topic, payload):
    """
    ฟังก์ชันสำหรับ publish ข้อมูลไปยัง MQTT broker
    """
    import paho.mqtt.client as mqtt
    client = mqtt.Client()
    try:
        # เชื่อมต่อไปยัง MQTT broker ตามค่าที่ตั้งไว้ใน utils.py
        client.connect(BROKER_URL, BROKER_PORT, BROKER_KEEP_ALIVE)
        client.publish(topic, payload)
        print(f"[MQTT] Published to {topic}: {payload}")
    except Exception as e:
        print(f"❌ MQTT Error: {e}")
    finally:
        client.disconnect()
