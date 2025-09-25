# shared/topics.py
from __future__ import annotations
import os, time
from typing import List, Dict
from shared.utils import publish_mqtt

CUPBOARD_ID: str = os.getenv("CUPBOARD_ID")
SLOT_IDS = [s.strip() for s in os.getenv("SLOT_IDS", "").split(",") if s.strip()]

SLOT_TO_INDEX: Dict[str, int] = {sid: i for i, sid in enumerate(SLOT_IDS)}
INDEX_TO_SLOT: Dict[int, str] = {i: sid for sid, i in SLOT_TO_INDEX.items()}

def t_status(slot_id: str) -> str:  return f"smartlocker/{CUPBOARD_ID}/slot_id/{slot_id}/status"
def t_warning(slot_id: str) -> str: return f"smartlocker/{CUPBOARD_ID}/slot_id/{slot_id}/warning"
def t_command(slot_id: str, action: str) -> str: return f"smartlocker/{CUPBOARD_ID}/slot_id/{slot_id}/command_open/{action}"

TOPIC_COMMAND_OPEN_REMOVAL  = "smartlocker/+/slot_id/+/command_open/removal"
TOPIC_COMMAND_OPEN_INTAKE = "smartlocker/+/slot_id/+/command_open/intake"

def get_subscriptions(broad: bool = True) -> list[str]:
    if broad:
        return [TOPIC_COMMAND_OPEN_INTAKE, TOPIC_COMMAND_OPEN_REMOVAL]
    return [t_command(s, "removal") for s in SLOT_IDS] + [t_command(s, "intake") for s in SLOT_IDS]

def publish_status(slot_id: str, status: dict, *, qos=1, retain=False):
    payload = {"cupboard_id": CUPBOARD_ID,"slot_id": slot_id, **status,"ts": time.time()}
    return publish_mqtt(t_status(slot_id), payload, qos=qos, retain=retain)

def publish_warning(slot_id: str, message: str, *, qos=1, retain=False, extra: dict | None = None):
    payload = {"cupboard_id": CUPBOARD_ID,"slot_id": slot_id, "message": message, "ts": time.time(), **(extra or {})}
    return publish_mqtt(t_warning(slot_id), payload, qos=qos, retain=retain)

def publish_unlock(slot_id: str, role: str = "admin", *, qos=1, retain=False):
    return publish_mqtt(t_command(slot_id, "removal"), {"role": role}, qos=qos, retain=retain)
