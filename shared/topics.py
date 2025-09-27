# shared/topics.py
from __future__ import annotations
import os
from typing import List, Dict
from datetime import datetime
from zoneinfo import ZoneInfo
from .utils import publish_mqtt

# =============================================================================
# 1) ค่าคงที่จาก ENV
#    - CUPBOARD_ID: ไอดีตู้ (เช่น "C01")
#    - SLOT_IDS:    รายการ slot_id คั่นด้วย comma (เช่น "SC001,SC002,SC003")
# =============================================================================
CUPBOARD_ID: str = os.getenv("CUPBOARD_ID")
SLOT_IDS = [s.strip() for s in os.getenv("SLOT_IDS", "").split(",") if s.strip()]

SLOT_TO_INDEX: Dict[str, int] = {sid: i for i, sid in enumerate(SLOT_IDS)}
INDEX_TO_SLOT: Dict[int, str] = {i: sid for sid, i in SLOT_TO_INDEX.items()}

# =============================================================================
# 2) ตัวช่วยประกอบชื่อ MQTT Topic
# =============================================================================
def t_status(slot_id: str) -> str:  return f"smartlocker/{CUPBOARD_ID}/slot_id/{slot_id}/status"
def t_warning(slot_id: str) -> str: return f"smartlocker/{CUPBOARD_ID}/slot_id/{slot_id}/warning"
def t_command(slot_id: str, action: str) -> str: return f"smartlocker/{CUPBOARD_ID}/slot_id/{slot_id}/command_open/{action}"

TOPIC_COMMAND_OPEN_DOOR  = "smartlocker/+/slot_id/+/command_open/door"
TOPIC_COMMAND_OPEN_SLOT = "smartlocker/+/slot_id/+/command_open/slot"

# =============================================================================
# 3) รายการ Subscription
# =============================================================================
def get_subscriptions(broad: bool = True) -> list[str]:
    if broad:
        return [TOPIC_COMMAND_OPEN_DOOR, TOPIC_COMMAND_OPEN_SLOT]
    return [t_command(s, "door") for s in SLOT_IDS] + [t_command(s, "slot") for s in SLOT_IDS]

# =============================================================================
# 4) เวลา (Asia/Bangkok) + ฟังก์ชัน publish
# =============================================================================
def _now_bkk_str() -> str:
    # ตัวอย่าง: 2025-09-27 12:05:18.209
    return datetime.now(ZoneInfo("Asia/Bangkok")).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

def publish_status(slot_id: str, status: dict, *, qos=0, retain=True):
    payload = {
        "cupboard_id": CUPBOARD_ID,
        "slot_id": slot_id,
        **status,
        "time_local": f"กรุงเทพฯ (UTC+7): {_now_bkk_str()}",
    }
    return publish_mqtt(t_status(slot_id), payload, qos=qos, retain=retain)

def publish_warning(slot_id: str, message: str, *, qos=1, retain=False, extra: dict | None = None):
    payload = {
        "cupboard_id": CUPBOARD_ID,
        "slot_id": slot_id,
        "message": message,
        "time_local": f"กรุงเทพฯ (UTC+7): {_now_bkk_str()}",
        **(extra or {}),
    }
    return publish_mqtt(t_warning(slot_id), payload, qos=qos, retain=retain)

def publish_unlock(slot_id: str, role: str = "admin", *, qos=0, retain=True):
    return publish_mqtt(t_command(slot_id, "DOOR"), {"role": role}, qos=qos, retain=retain)
