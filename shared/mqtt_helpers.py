# ✅ shared/mqtt_helpers.py
# 🔧 ฟังก์ชันจัดการ MQTT สำหรับระบบ Multi-Node

import time
import json
from shared.utils import publish_mqtt

# ==== TOPIC TEMPLATE (Multi-Node) ====
TOPIC_STATUS = "smartlocker/{node_id}/slot/{slot_id}/status"
TOPIC_WARNING = "smartlocker/{node_id}/slot/{slot_id}/warning"
TOPIC_COMMAND_OPEN = "smartlocker/+/slot/+/command/open"
TOPIC_COMMAND_UNLOCK = "smartlocker/+/slot/+/command/unlock"

# Optional future topics (commented out)
# TOPIC_LOG = "smartlocker/{node_id}/log"
# TOPIC_HEARTBEAT = "smartlocker/{node_id}/heartbeat"

# ==== Topic Generators ====
def make_status_topic(node_id, slot_id):
    return TOPIC_STATUS.format(node_id=node_id, slot_id=slot_id)

def make_warning_topic(node_id, slot_id):
    return TOPIC_WARNING.format(node_id=node_id, slot_id=slot_id)

def get_command_subscriptions():
    return [TOPIC_COMMAND_OPEN, TOPIC_COMMAND_UNLOCK]

# ==== Publish Utilities ====
def publish_status(node_id, slot_id, status_dict):
    topic = make_status_topic(node_id, slot_id)
    payload = {
        **status_dict,
        "slot": slot_id,
        "node": node_id
    }
    publish_mqtt(topic, payload)

def publish_warning(node_id, slot_id, message="ประตูยังไม่ปิดสนิทหลังการเปิดใช้งาน"):
    topic = make_warning_topic(node_id, slot_id)
    payload = {
        "slot": slot_id,
        "node": node_id,
        "message": message,
        "timestamp": time.time()
    }
    publish_mqtt(topic, payload)
