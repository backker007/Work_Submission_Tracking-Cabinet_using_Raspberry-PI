# shared/config.py
from __future__ import annotations
import os, json
from dataclasses import dataclass
from typing import List, Optional

# ---------- helpers ----------
def _parse_list(s: Optional[str]) -> List[str]:
    """รับได้ทั้ง JSON list หรือคั่นด้วย comma; ถ้าเป็นสตริงเดี่ยวจะกลายเป็นลิสต์ 1 ตัวอัตโนมัติ"""
    if not s:
        return []
    s = s.strip()
    if s.startswith("["):
        try:
            val = json.loads(s)
            if isinstance(val, list):
                return [str(x).strip() for x in val]
        except Exception:
            pass
    return [x.strip() for x in s.split(",") if x.strip()]

def _to_int_list(ss: List[str]) -> List[int]:
    out: List[int] = []
    for token in ss:
        t = token.lower()
        try:
            out.append(int(t, 16) if t.startswith("0x") else int(t))
        except Exception:
            raise ValueError(f"Cannot parse int from '{token}'")
    return out

def _get_int(name: str, default: int) -> int:
    v = os.getenv(name, "").strip()
    if not v:
        return default
    try:
        return int(v)
    except Exception:
        raise ValueError(f"Env {name} must be an integer, got '{v}'")

def _get_float(name: str, default: float) -> float:
    v = os.getenv(name, "").strip()
    if not v:
        return default
    try:
        return float(v)
    except Exception:
        raise ValueError(f"Env {name} must be a float, got '{v}'")

def _get_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "on")

def _get_str(name: str, default: str) -> str:
    v = os.getenv(name, None)
    return (v if v is not None else default).strip()

# ---------- config ----------
@dataclass(frozen=True)
class Config:
    # Locker identity
    CUPBOARD_ID: str
    NODE_ID: str
    SLOT_IDS: List[str]

    # MQTT
    MQTT_HOST: str
    MQTT_PORT: int
    MQTT_USERNAME: str
    MQTT_PASSWORD: str
    MQTT_TLS: bool
    MQTT_USE_WEBSOCKET: bool
    MQTT_CLIENT_ID: str
    MQTT_CA_PATH: str | None
    MQTT_BASE_TOPIC: str

    # Hardware maps
    XSHUT_PINS: List[int]
    SENSOR_ADDRS: List[int]
    RELAY_PINS: List[int]
    SERVO_CHANNELS: List[int]
    MCP_REED_PINS: List[int]
    MCP_LED_PINS: List[int]

    # VL53 mode / tuning
    VL53_BUS_MODE: str                  # e.g. "multi"
    VL53_BUDGET_US: int
    VL53_CONTINUOUS: bool
    VL53_INIT_ALL_LOW: bool
    VL53_BASE_ADDR: int | None
    VL53_BOOT_DELAY_S: float
    VL53_BOOT_TIMEOUT_S: float
    VL53_ADDR_SET_RETRIES: int
    VL53_ADDR_SET_GAP_S: float
    VL53_POST_SET_DELAY_S: float
    VL53_OPEN_RETRIES: int
    VL53_OPEN_DELAY_S: float
    VL53_OPEN_DELAY_PRE_S: float
    VL53_PROBE_INTERVAL_S: float

    # Range/thresholds
    TARGET_MIN_MM: int
    TARGET_MAX_MM: int
    ZERO_THRESHOLD: int
    ACTIVE_CHECK_INTERVAL: float
    CHECK_INTERVAL: float
    CHANGE_THRESHOLD_MM: int  # fallback สำหรับโค้ดเดิม

    # Logging
    LOG_LEVEL: str

def _derive_sensor_addrs(slot_count: int, explicit_addrs: List[int], base_hex: Optional[int]) -> List[int]:
    if explicit_addrs:
        return explicit_addrs[:slot_count]
    if base_hex is not None:
        return [base_hex + i for i in range(slot_count)]
    # default 0x30.. up to slot_count
    base = 0x30
    return [base + i for i in range(slot_count)]

def _load_cfg() -> Config:
    # --- identity ---
    cupboard_id = _get_str("CUPBOARD_ID", "C01")
    node_id = _get_str("NODE_ID", cupboard_id)  # ถ้าไม่ได้กำหนด NODE_ID ให้เท่ากับ CUPBOARD_ID
    slot_ids = _parse_list(os.getenv("SLOT_IDS")) or ["SC001", "SC002", "SC003", "SC004"]

    # --- mqtt ---
    mqtt_host = _get_str("MQTT_HOST", "localhost")
    mqtt_port = _get_int("MQTT_PORT", 1883)
    mqtt_user = _get_str("MQTT_USERNAME", "")
    mqtt_pass = _get_str("MQTT_PASSWORD", "")
    # รองรับทั้งชื่อเก่า/ใหม่
    mqtt_tls  = _get_bool("MQTT_TLS", _get_bool("MQTT_USE_TLS", False))
    mqtt_ws   = _get_bool("MQTT_WS", _get_bool("MQTT_USE_WEBSOCKET", False))
    mqtt_cid  = _get_str("MQTT_CLIENT_ID", node_id + "-client")
    mqtt_ca   = os.getenv("MQTT_CA", None)
    mqtt_ca   = mqtt_ca.strip() if mqtt_ca else None
    mqtt_base = _get_str("MQTT_BASE_TOPIC", "smartlocker")

    # --- hardware lists (รองรับชื่อทั้งแบบเก่าและใหม่) ---
    # XSHUT pins
    xshut = _to_int_list(_parse_list(os.getenv("VL53_XSHUT_PINS") or os.getenv("XSHUT_PINS") or "")) \
            or [17, 27, 22, 5]

    # SENSOR_ADDRS (ถ้าไม่กำหนด จะ derive จาก VL53_BASE_ADDR)
    explicit_addrs = _to_int_list(_parse_list(os.getenv("SENSOR_ADDRS") or "")) if os.getenv("SENSOR_ADDRS") else []
    base_addr = None
    if os.getenv("VL53_BASE_ADDR"):
        b = os.getenv("VL53_BASE_ADDR").strip().lower()
        base_addr = int(b, 16) if b.startswith("0x") else int(b)

    # slots length เป็นตัวกำหนดจำนวนจริง
    n = len(slot_ids)
    sensor_addrs = _derive_sensor_addrs(n, explicit_addrs, base_addr)

    # relay/servo/mcp (ถ้าไม่ได้ตั้งค่าใน .env จะใส่ค่า dummy ยาวเท่า slot)
    relays = _to_int_list(_parse_list(os.getenv("RELAY_PINS") or "")) or list(range(12, 12 + n))
    servos = _to_int_list(_parse_list(os.getenv("SERVO_CHANNELS") or "")) or list(range(0, n))
    mcp_reed = _to_int_list(_parse_list(os.getenv("MCP_REED_PINS") or "")) or list(range(8, 8 + n))
    mcp_led  = _to_int_list(_parse_list(os.getenv("MCP_LED_PINS")  or "")) or list(range(0, n))  # map เองภายหลัง

    # --- VL53 mode/tuning (รับทั้งชื่อเก่าและใหม่) ---
    vl53_bus_mode        = _get_str("VL53_BUS_MODE", "multi")
    vl53_budget_us       = _get_int("VL53_BUDGET_US", _get_int("VL53_TIMING_BUDGET_US", 20000))
    vl53_continuous      = _get_bool("VL53_CONTINUOUS", True)
    vl53_init_all_low    = _get_bool("VL53_INIT_ALL_LOW", True)
    vl53_boot_delay_s    = _get_float("VL53_BOOT_DELAY_S", 0.35)
    vl53_boot_timeout_s  = _get_float("VL53_BOOT_TIMEOUT_S", 2.0)
    vl53_addr_set_retries= _get_int("VL53_ADDR_SET_RETRIES", 6)
    vl53_addr_set_gap_s  = _get_float("VL53_ADDR_SET_GAP_S", 0.10)
    vl53_post_set_delay_s= _get_float("VL53_POST_SET_DELAY_S", 0.80)
    vl53_open_retries    = _get_int("VL53_OPEN_RETRIES", 12)
    vl53_open_delay_s    = _get_float("VL53_OPEN_DELAY_S", 0.35)
    vl53_open_delay_pre_s= _get_float("VL53_OPEN_DELAY_PRE_S", 0.25)
    vl53_probe_interval_s= _get_float("VL53_PROBE_INTERVAL_S", 0.12)

    # --- ranges/thresholds ---
    target_min_mm        = _get_int("TARGET_MIN_MM", _get_int("VL53_MIN_MM", 20))
    target_max_mm        = _5get_int = _get_int  # alias local (เผื่อใช้ซ้ำ)
    target_max_mm        = _get_int("TARGET_MAX_MM", _get_int("VL53_MAX_MM", 300))
    zero_threshold       = _get_int("ZERO_THRESHOLD", 70)
    active_check_iv      = _get_float("ACTIVE_CHECK_INTERVAL", 0.5)
    check_interval       = _get_float("CHECK_INTERVAL", 120)
    change_threshold_mm  = _get_int("CHANGE_THRESHOLD_MM", 5)

    # --- logging ---
    log_level            = _get_str("LOG_LEVEL", "INFO")

    # ตัดความยาวให้เท่าจำนวน slot เผื่อผู้ใช้กรอกยาวเกิน
    xshut       = xshut[:n]
    sensor_addrs= sensor_addrs[:n]
    relays      = relays[:n]
    servos      = servos[:n]
    mcp_reed    = mcp_reed[:n]
    mcp_led     = mcp_led[:n]

    return Config(
        CUPBOARD_ID=cupboard_id,
        NODE_ID=node_id,
        SLOT_IDS=slot_ids,

        MQTT_HOST=mqtt_host,
        MQTT_PORT=mqtt_port,
        MQTT_USERNAME=mqtt_user,
        MQTT_PASSWORD=mqtt_pass,
        MQTT_TLS=mqtt_tls,
        MQTT_USE_WEBSOCKET=mqtt_ws,
        MQTT_CLIENT_ID=mqtt_cid,
        MQTT_CA_PATH=mqtt_ca,
        MQTT_BASE_TOPIC=mqtt_base,

        XSHUT_PINS=xshut,
        SENSOR_ADDRS=sensor_addrs,
        RELAY_PINS=relays,
        SERVO_CHANNELS=servos,
        MCP_REED_PINS=mcp_reed,
        MCP_LED_PINS=mcp_led,

        VL53_BUS_MODE=vl53_bus_mode,
        VL53_BUDGET_US=vl53_budget_us,
        VL53_CONTINUOUS=vl53_continuous,
        VL53_INIT_ALL_LOW=vl53_init_all_low,
        VL53_BASE_ADDR=base_addr,
        VL53_BOOT_DELAY_S=vl53_boot_delay_s,
        VL53_BOOT_TIMEOUT_S=vl53_boot_timeout_s,
        VL53_ADDR_SET_RETRIES=vl53_addr_set_retries,
        VL53_ADDR_SET_GAP_S=vl53_addr_set_gap_s,
        VL53_POST_SET_DELAY_S=vl53_post_set_delay_s,
        VL53_OPEN_RETRIES=vl53_open_retries,
        VL53_OPEN_DELAY_S=vl53_open_delay_s,
        VL53_OPEN_DELAY_PRE_S=vl53_open_delay_pre_s,
        VL53_PROBE_INTERVAL_S=vl53_probe_interval_s,

        TARGET_MIN_MM=target_min_mm,
        TARGET_MAX_MM=target_max_mm,
        ZERO_THRESHOLD=zero_threshold,
        ACTIVE_CHECK_INTERVAL=active_check_iv,
        CHECK_INTERVAL=check_interval,
        CHANGE_THRESHOLD_MM=change_threshold_mm,

        LOG_LEVEL=log_level,
    )

CFG = _load_cfg()
