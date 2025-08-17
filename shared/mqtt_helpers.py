# mqtt_helpers.py
import time
from shared.utils import publish_mqtt

# สร้าง Topic Template
TOPIC_STATUS = "smartlocker/{node_id}/slot/{slot_id}/status"
TOPIC_WARNING = "smartlocker/{node_id}/slot/{slot_id}/warning"
TOPIC_COMMAND = "smartlocker/{node_id}/slot/{slot_id}/command/{action}"

# ฟังก์ชันเพื่อสร้างหัวข้อ MQTT
def make_topic(topic_type, node_id, slot_id, action=None):
    if action:
        return topic_type.format(node_id=node_id, slot_id=slot_id, action=action)
    return topic_type.format(node_id=node_id, slot_id=slot_id)

# ฟังก์ชันสำหรับการ Publish ข้อมูล
def publish_status(node_id, slot_id, status_dict):
    topic = make_topic(TOPIC_STATUS, node_id, slot_id)
    publish_mqtt(topic, status_dict)

def publish_warning(node_id, slot_id, message="ประตูยังไม่ปิดสนิทหลังการเปิดใช้งาน"):
    topic = make_topic(TOPIC_WARNING, node_id, slot_id)
    payload = {
        "message": message,
        "timestamp": time.time()
    }
    publish_mqtt(topic, payload)

# ฟังก์ชันสำหรับการสร้าง topic สำหรับคำสั่ง
def make_command_topic(node_id, slot_id, action):
    return make_topic(TOPIC_COMMAND, node_id, slot_id, action)
