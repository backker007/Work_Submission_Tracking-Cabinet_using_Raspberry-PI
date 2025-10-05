
# shared/topics.py
from __future__ import annotations
import json, os
from datetime import datetime, timezone, timedelta
try:
    # Python 3.9+ มาตรฐาน
    from zoneinfo import ZoneInfo  # type: ignore
except Exception:
    ZoneInfo = None  # fallback ใช้ระบบ local time

from .config import CFG

# ===== Config =====
BASE         = CFG.MQTT_BASE_TOPIC            # "smartlocker"
CUPBOARD_ID  = CFG.CUPBOARD_ID                # เช่น "C01"
NODE_ID      = CFG.NODE_ID or CUPBOARD_ID     # กันไว้ให้เท่ากัน
SLOT_IDS     = CFG.SLOT_IDS

SLOT_TO_INDEX = {sid: i for i, sid in enumerate(SLOT_IDS)}
INDEX_TO_SLOT = {i: sid for sid, i in SLOT_TO_INDEX.items()}

# ===== เวลา local สำหรับฟิลด์ time_local =====
_TIME_TZ     = os.getenv("TIME_TZ", "Asia/Bangkok")
_TIME_LABEL  = os.getenv("TIME_LABEL", "กรุงเทพฯ")
ADOPT_STALE = os.getenv("VL53_ADOPT_STALE_ADDR", "1").lower() in ("1","true","yes")



def _time_local_str() -> str:
    try:
        if ZoneInfo:
            tz = ZoneInfo(_TIME_TZ)
            now = datetime.now(tz)
            offset = now.utcoffset() or timedelta(0)
        else:
            now = datetime.now().astimezone()
            offset = now.utcoffset() or timedelta(0)
        total_minutes = int(offset.total_seconds() // 60)
        sign = "+" if total_minutes >= 0 else "-"
        hours = abs(total_minutes) // 60
        ms = int(now.microsecond / 1000)
        stamp = now.strftime("%Y-%m-%d %H:%M:%S") + f".{ms:03d}"
        return f"{_TIME_LABEL} (UTC{sign}{hours}): {stamp}"
    except Exception:
        now = datetime.now()
        ms = int(now.microsecond / 1000)
        return f"{_TIME_LABEL}: {now.strftime('%Y-%m-%d %H:%M:%S')}.{ms:03d}"

# =============================================================================
# 2) ตัวช่วยประกอบชื่อ MQTT Topic
# =============================================================================
def topic_status(slot_id: str) -> str:
    # legacy: smartlocker/{CUPBOARD_ID}/slot_id/{slot}/status
    return f"{BASE}/{CUPBOARD_ID}/slot_id/{slot_id}/status"

def topic_warning(slot_id: str) -> str:
    return f"{BASE}/{CUPBOARD_ID}/slot_id/{slot_id}/warning"

def topic_command_wildcard(cupboard: str | None = None) -> List[str]:
    """
    คืน wildcard สำหรับรับคำสั่งเปิด (รองรับทั้ง legacy & modern)
    - legacy: smartlocker/{CUPBOARD}/slot_id/+/command_open/#
    - modern: smartlocker/{CUPBOARD}/slot/+/command/#
    """
    nid = (cupboard or CUPBOARD_ID) or "+"
    legacy = f"{BASE}/{nid}/slot_id/+/command_open/#"
    modern = f"{BASE}/{nid}/slot/+/command/#"
    return [legacy, modern]

def get_subscriptions(broad: bool = False) -> List[str]:
    """
    ใช้ใน on_connect:
      - broad=True  → subscribe ทั่วทั้งระบบของ base (CUPBOARD_ID = '+')
      - broad=False → subscribe เฉพาะตู้ปัจจุบัน
    """
    nid = "+" if broad else CUPBOARD_ID
    return topic_command_wildcard(nid)

# =============================================================================
# 3) Publishers (ใช้ mqtt_client ของ paho โดยตรง)
# =============================================================================
def publish_status(mqtt_client, payload: dict, slot_id: str):
    """
    ส่งสถานะของช่องไปยัง topic_status(slot_id)
    - qos=1, retain=True
    """
    data = dict(payload or {})
    data.setdefault("cupboard_id", CUPBOARD_ID)
    data.setdefault("slot_id", slot_id)
    data.setdefault("time_local", _time_local_str())
    res = mqtt_client.publish(
        topic_status(slot_id),
        json.dumps(data, ensure_ascii=False),
        qos=1,
        retain=True,
    )
    # paho-mqtt จะคืนคล้ายๆ MQTTMessageInfo; เอา mid มา log ได้
    return getattr(res, "mid", None)

def publish_warning(mqtt_client, message: str, slot_id: str, extra: dict | None = None):
    """
    ส่งข้อความเตือนของช่องไปยัง topic_warning(slot_id)
    - qos=1, retain=False
    """
    data = {
        "cupboard_id": CUPBOARD_ID,
        "slot_id": slot_id,
        "message": message,
        "time_local": _time_local_str(),
    }
    if extra:
        data.update(extra)
    res = mqtt_client.publish(
        topic_warning(slot_id),
        json.dumps(data, ensure_ascii=False),
        qos=1,
        retain=False,
    )
    return getattr(res, "mid", None)

# (ออปชัน) utility เผื่อสั่งปลดล็อกจากโค้ดภายนอกอย่างง่าย
def publish_unlock(mqtt_client, slot_id: str, role: str = "admin"):
    """
    ส่งคำสั่ง 'open door' แบบ legacy (ยังคงรองรับ) ไปยังช่องที่ระบุ
    หมายเหตุ: ฝั่ง controller รองรับทั้ง legacy & modern อยู่แล้ว
    """
    topic = f"{BASE}/{CUPBOARD_ID}/slot_id/{slot_id}/command_open/door"
    payload = {"role": role, "time_local": _time_local_str()}
    res = mqtt_client.publish(topic, json.dumps(payload, ensure_ascii=False), qos=1, retain=False)
    return getattr(res, "mid", None)
