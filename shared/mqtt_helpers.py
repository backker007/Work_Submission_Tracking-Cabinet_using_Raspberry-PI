# import time
# from shared.utils import publish_mqtt

# # ==== TOPIC TEMPLATE (Multi-Node) ====
# TOPIC_STATUS = "smartlocker/{cupboards_id}/slot_id/{slot_id}/status"
# TOPIC_WARNING = "smartlocker/{cupboards_id}/slot_id/{slot_id}/warning"

# # สำหรับ subscribe แบบรวมทุก node/slot_id
# TOPIC_COMMAND_OPEN_REMOVAL  = "smartlocker/+/slot_id/SC00+/open/removal"
# TOPIC_COMMAND_OPEN_INTAKE = "smartlocker/+/slot_id/SC00+/open/intake"

# # สำหรับสร้าง topic แบบระบุ node/slot_id/action เฉพาะ
# CMD_PATTERN = "smartlocker/{cupboards_id}/slot_id/{slot_id}/open/{action}"

# # ==== Topic Builders ====
# def make_status_topic(cupboards_id, slot_id):
#     return TOPIC_STATUS.format(cupboards_id=cupboards_id, slot_id=slot_id)

# def make_warning_topic(cupboards_id, slot_id):
#     return TOPIC_WARNING.format(cupboards_id=cupboards_id, slot_id=slot_id)

# def make_open_topic(cupboards_id, slot_id, action):
#     return CMD_PATTERN.format(cupboards_id=cupboards_id, slot_id=slot_id, action=action)

# def get_open_subscriptions():
#     # รายการ topic ที่ controller ต้อง subscribe
#     return [TOPIC_COMMAND_OPEN_REMOVAL, TOPIC_COMMAND_OPEN_INTAKE]

# # ==== Publish Helpers ====
# def publish_status(cupboards_id, slot_id, status_dict):
#     topic = make_status_topic(cupboards_id, slot_id)
#     payload = {**status_dict, "slot_id": slot_id, "node": cupboards_id}
#     publish_mqtt(topic, payload)

# def publish_warning(cupboards_id, slot_id, message="ประตูยังไม่ปิดสนิทหลังการเปิดใช้งาน"):
#     topic = make_warning_topic(cupboards_id, slot_id)
#     payload = {
#         "slot_id": slot_id,
#         "node": cupboards_id,
#         "message": message,
#         "timestamp": time.time()
#     }
#     publish_mqtt(topic, payload)

# shared/mqtt_helpers.py
# Back-compat shim: เรียกใช้ของจริงจาก shared.topics
from shared.topics import (
    get_subscriptions as get_open_subscriptions,
    publish_status,
    publish_warning,
    publish_unlock,   # optional: เผื่อฝั่งอื่นต้องการเรียกสั่งปลดล็อกโดยตรง
    t_status,
    t_warning,
    t_open,
    CUPBOARDS_ID,
    SLOT_IDS,
    SLOT_TO_INDEX,
    INDEX_TO_SLOT,
)

__all__ = [
    "get_open_subscriptions",
    "publish_status",
    "publish_warning",
    "publish_unlock",
    "t_status",
    "t_warning",
    "t_open",
    "CUPBOARDS_ID",
    "SLOT_IDS",
    "SLOT_TO_INDEX",
    "INDEX_TO_SLOT",
]