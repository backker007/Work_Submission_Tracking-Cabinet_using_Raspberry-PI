import time
from shared.utils import publish_mqtt

# ==== TOPIC TEMPLATE (Multi-Node) ====
TOPIC_STATUS = "smartlocker/{node_id}/slot/{slot_id}/status"
TOPIC_WARNING = "smartlocker/{node_id}/slot/{slot_id}/warning"

# สำหรับ subscribe แบบรวมทุก node/slot
TOPIC_COMMAND_OPEN   = "smartlocker/+/slot/+/command/open"
TOPIC_COMMAND_UNLOCK = "smartlocker/+/slot/+/command/unlock"

# สำหรับสร้าง topic แบบระบุ node/slot/action เฉพาะ
CMD_PATTERN = "smartlocker/{node_id}/slot/{slot_id}/command/{action}"

# ==== Topic Builders ====
def make_status_topic(node_id, slot_id):
    return TOPIC_STATUS.format(node_id=node_id, slot_id=slot_id)

def make_warning_topic(node_id, slot_id):
    return TOPIC_WARNING.format(node_id=node_id, slot_id=slot_id)

def make_command_topic(node_id, slot_id, action):
    return CMD_PATTERN.format(node_id=node_id, slot_id=slot_id, action=action)

def get_command_subscriptions():
    # รายการ topic ที่ controller ต้อง subscribe
    return [TOPIC_COMMAND_OPEN, TOPIC_COMMAND_UNLOCK]

# ==== Publish Helpers ====
def publish_status(node_id, slot_id, status_dict):
    topic = make_status_topic(node_id, slot_id)
    payload = {**status_dict, "slot": slot_id, "node": node_id}
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
