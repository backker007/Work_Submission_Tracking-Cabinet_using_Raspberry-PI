# shared/utils.py
import os
import ssl
import time
import json
import paho.mqtt.client as mqtt

def publish_mqtt(
    topic,
    payload,
    *,
    broker=None,
    port=None,
    retry=3,
    qos=0,
    retain=False,
    username=None,
    password=None,
    tls=True,
    ca_certs=None,          # path CA .pem (ถ้าไม่ระบุ จะใช้ CA ของระบบ)
    client_id=None,
    websocket=False,        # True = MQTT over WebSocket (wss), มักใช้พอร์ต 443
):
    """
    Cloud‑ready MQTT publisher.

    ถ้าไม่ได้ส่งพารามิเตอร์ จะ fallback ไปอ่านจาก ENV:
      MQTT_HOST, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD,
      MQTT_TLS (1/0), MQTT_WS (1/0), MQTT_CLIENT_ID, MQTT_CA
    """

    # ---------- Load defaults from ENV ----------
    broker    = broker    or os.getenv("MQTT_HOST", "localhost")
    port      = int(port or os.getenv("MQTT_PORT", "1883"))
    username  = username  or os.getenv("MQTT_USERNAME")
    password  = password  or os.getenv("MQTT_PASSWORD")
    env_tls   = os.getenv("MQTT_TLS")
    tls       = bool(int(env_tls)) if env_tls is not None else tls
    env_ws    = os.getenv("MQTT_WS")
    websocket = bool(int(env_ws)) if env_ws is not None else websocket
    client_id = client_id or os.getenv("MQTT_CLIENT_ID")
    ca_certs  = ca_certs  or os.getenv("MQTT_CA")  # optional

    # ---------- Normalize payload ----------
    if isinstance(payload, (dict, list)):
        payload = json.dumps(payload, ensure_ascii=False)
    elif not isinstance(payload, (str, bytes, bytearray, int, float, type(None))):
        payload = str(payload)

    # ---------- Publish with retry ----------
    for attempt in range(1, retry + 1):
        try:
            client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
                transport="websockets" if websocket else "tcp",
                protocol=mqtt.MQTTv311,
            )

            if username:
                client.username_pw_set(username=username, password=password)

            if tls:
                if ca_certs:
                    client.tls_set(
                        ca_certs=ca_certs,
                        certfile=None,
                        keyfile=None,
                        tls_version=ssl.PROTOCOL_TLS_CLIENT,
                    )
                else:
                    client.tls_set_context(ssl.create_default_context())
                client.tls_insecure_set(False)
                if port == 1883:   # เผื่อผู้ใช้ยังไม่ได้ตั้งพอร์ต TLS
                    port = 8883

            client.connect(broker, port, keepalive=60)
            client.loop_start()

            info = client.publish(topic, payload, qos=qos, retain=retain)
            info.wait_for_publish(timeout=3)

            client.loop_stop()
            client.disconnect()
            print(f"[MQTT] Published to {topic} -> {payload}")
            return True

        except Exception as e:
            print(f"[x] MQTT Error (attempt {attempt}/{retry}): {e}")
            time.sleep(1)

    return False
