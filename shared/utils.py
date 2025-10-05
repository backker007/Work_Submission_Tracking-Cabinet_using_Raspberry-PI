# shared/utils.py
from __future__ import annotations
import os
import ssl
import time
import json
from typing import Any, Optional
import paho.mqtt.client as mqtt
from datetime import datetime
from zoneinfo import ZoneInfo  # Python 3.9+

# =============================================================================
# 1) ฟังก์ชัน publish_mqtt : ส่งข้อความไปยัง MQTT Broker (รองรับ TLS/WS/Retry + Backoff)
#    ENV ที่รองรับ (ชื่อไหนมาก็ได้):
#      MQTT_HOST, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD,
#      MQTT_TLS (หรือ MQTT_USE_TLS), MQTT_WS (หรือ MQTT_USE_WEBSOCKET),
#      MQTT_CLIENT_ID, MQTT_CA, MQTT_WS_PATH,
#      MQTT_RECONNECT_BASE_S, MQTT_RECONNECT_MAX_S, MQTT_PUBLISH_TIMEOUT_S
# =============================================================================
def publish_mqtt(
    topic: str,
    payload: Any,
    *,
    broker: Optional[str] = None,
    port: Optional[int] = None,
    retry: int = 3,
    qos: int = 0,
    retain: bool = False,
    username: Optional[str] = None,
    password: Optional[str] = None,
    tls: Optional[bool] = None,
    ca_certs: Optional[str] = None,  # path CA .pem (ถ้าไม่ระบุ จะใช้ CA ของระบบ)
    client_id: Optional[str] = None,
    websocket: Optional[bool] = None,  # True = MQTT over WebSocket (wss)
    ws_path: Optional[str] = None,
) -> Optional[int]:
    """Publish แบบ one-shot พร้อม backoff/retry; คืน mid เมื่อสำเร็จ, None เมื่อผิดพลาด"""

    # ---------- Load defaults from ENV ----------
    broker    = broker or os.getenv("MQTT_HOST", "localhost")
    port_env  = os.getenv("MQTT_PORT", "1883")
    port      = int(port if port is not None else port_env)

    username  = username if username is not None else os.getenv("MQTT_USERNAME")
    password  = password if password is not None else os.getenv("MQTT_PASSWORD")

    # bools: รองรับชื่อเก่า/ใหม่
    env_tls   = os.getenv("MQTT_TLS")
    env_use_tls = os.getenv("MQTT_USE_TLS")
    tls = (tls if tls is not None
           else (bool(int(env_tls)) if env_tls is not None
                 else (bool(int(env_use_tls)) if env_use_tls is not None else False)))

    env_ws    = os.getenv("MQTT_WS")
    env_use_ws= os.getenv("MQTT_USE_WEBSOCKET")
    websocket = (websocket if websocket is not None
                 else (bool(int(env_ws)) if env_ws is not None
                       else (bool(int(env_use_ws)) if env_use_ws is not None else False)))

    client_id = client_id or os.getenv("MQTT_CLIENT_ID")
    ca_certs  = ca_certs or os.getenv("MQTT_CA")
    ws_path   = ws_path or os.getenv("MQTT_WS_PATH", "/mqtt")

    # Backoff / timeouts
    reconnect_base = float(os.getenv("MQTT_RECONNECT_BASE_S", "1.0"))
    reconnect_max  = float(os.getenv("MQTT_RECONNECT_MAX_S",  "32.0"))
    publish_timeout= float(os.getenv("MQTT_PUBLISH_TIMEOUT_S", "3.0"))

    # ---------- Normalize payload ----------
    if isinstance(payload, (dict, list)):
        payload = json.dumps(payload, ensure_ascii=False)
    elif not isinstance(payload, (str, bytes, bytearray, int, float, type(None))):
        payload = str(payload)

    # ---------- Publish with retry + backoff ----------
    delay = reconnect_base
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

            if websocket:
                try:
                    client.ws_set_options(path=ws_path)
                except Exception:
                    pass

            if tls:
                use_default_ctx = True
                if ca_certs:
                    try:
                        if not os.path.isfile(ca_certs):
                            raise FileNotFoundError(ca_certs)
                        client.tls_set(
                            ca_certs=ca_certs,
                            tls_version=ssl.PROTOCOL_TLS_CLIENT,
                        )
                        use_default_ctx = False
                    except Exception as _e:
                        print(f"[!] MQTT_CA invalid: {ca_certs} ({_e}) -> fallback to system CA")

                if use_default_ctx:
                    client.tls_set_context(ssl.create_default_context())
                client.tls_insecure_set(False)
                if port == 1883:
                    port = 8883

            # QoS1 reliability knobs (ปลอดภัยเผื่อ broker หน่วง)
            client.reconnect_delay_set(min_delay=1, max_delay=16)
            client.max_inflight_messages_set(10)
            client.max_queued_messages_set(1000)

            # Connect with backoff loop (ภายใน 1 attempt)
            local_delay = delay
            while True:
                try:
                    client.connect(broker, port, keepalive=60)
                    break
                except Exception as e:
                    print(f"[x] MQTT connect failed: {e} (retry in {local_delay:.1f}s)")
                    time.sleep(local_delay)
                    local_delay = min(reconnect_max, local_delay * 2)

            client.loop_start()
            info = client.publish(topic, payload, qos=qos, retain=retain)
            info.wait_for_publish(timeout=publish_timeout)

            mid = getattr(info, "mid", None)
            client.loop_stop()
            client.disconnect()

            print(f"[MQTT] Published to {topic} (mid={mid}) -> {payload}")
            return mid

        except Exception as e:
            print(f"[x] MQTT Error (attempt {attempt}/{retry}): {e}")
            time.sleep(delay)
            delay = min(reconnect_max, delay * 2)

    return None


# =============================================================================
# 2) ฟังก์ชันเวลา (Asia/Bangkok)
#    - now_bkk_str(): คืนค่าสตริงเวลาแบบท้องถิ่น กทม. ความละเอียดมิลลิวินาที
# =============================================================================
def now_bkk_str() -> str:
    # ตัวอย่าง: 2025-09-27 12:05:18.209
    return datetime.now(ZoneInfo("Asia/Bangkok")).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
