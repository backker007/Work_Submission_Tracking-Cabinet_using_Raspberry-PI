# shared/vl53l0x_init.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional
# shared/vl53l0x_init.py (ส่วนบน)
import os           
import time
import logging
...

log = logging.getLogger("vl53x_init")

# ===== Optional native backend (non-Adafruit) =====
try:
    import VL53L0X as vl53_native  # type: ignore[import-not-found]
except Exception:
    vl53_native = None

@dataclass
class XshutDriver:
    def __init__(self, pins: List[int], set_low: Callable[[int], None], set_high: Callable[[int], None]):
        self.pins = pins
        self.set_low = set_low
        self.set_high = set_high
    def all_low(self):
        # เดิม: for p in self.pins: self.set_low(p)
        for idx, _ in enumerate(self.pins):
            self.set_low(idx)
    def one_high(self, index: int):
        # เดิม: self.set_high(self.pins[index])
        self.set_high(index)


class XshutDriver:
    """รับรายชื่อ 'pins' เป็น index 0..N-1 และ set_low/set_high(index)"""
    def __init__(self, pins: List[int], set_low: Callable[[int], None], set_high: Callable[[int], None]):
        self.pins = pins
        self.set_low = set_low
        self.set_high = set_high
    def all_low(self):     # ปิดทุกตัว
        for p in self.pins: self.set_low(p)
    def one_high(self, index: int):  # เปิดเฉพาะตัวที่กำลังตั้ง
        self.set_high(self.pins[index])

# ---------- native helpers ----------
def _native_open_default(bus: int = 1, default_addr: int = 0x29):
    if vl53_native is None:
        raise RuntimeError("native backend not available")
    try:
        return vl53_native.VL53L0X(address=default_addr, bus=bus)
    except Exception:
        return vl53_native.VL53L0X(default_addr, bus)

def _native_change_address(sensor, new_addr: int):
    for m in ("change_address", "set_address", "set_address_i2c"):
        if hasattr(sensor, m):
            getattr(sensor, m)(new_addr); return
    raise RuntimeError("native backend has no API to change address")

def _native_start(sensor, timing_budget_us: int):
    if hasattr(sensor, "set_timing_budget"):
        try: sensor.set_timing_budget(timing_budget_us)
        except Exception: pass
    for m in ("start_ranging", "startContinuous", "start"):
        if hasattr(sensor, m):
            try: getattr(sensor, m)(); break
            except Exception: pass

def _native_read_factory(sensor):
    def _read():
        for m in ("get_distance", "readRangeSingleMillimeters", "read"):
            if hasattr(sensor, m):
                try:
                    v = getattr(sensor, m)()
                    if isinstance(v, (int, float)): return int(v)
                except Exception:
                    return None
        return None
    return _read

# ---------- adafruit helpers ----------
def _adafruit_open(i2c, addr: int):
    import adafruit_vl53l0x
    return adafruit_vl53l0x.VL53L0X(i2c, address=addr)

def _adafruit_read_factory(sensor):
    def _read():
        try:
            v = sensor.range
            if isinstance(v, (int, float)): return int(v)
        except Exception:
            return None
        return None
    return _read

def _probe_adafruit_default(i2c, default_addr, retries, interval):
    last = None
    for _ in range(max(1, retries)):
        try: return _adafruit_open(i2c, default_addr)
        except Exception as e:
            last = e; time.sleep(interval)
    raise last if last else OSError("adafruit probe failed @0x%02X" % default_addr)

# ---------- I2C presence helpers ----------
def _bus_has_addr(i2c, addr: int) -> bool:
    try:
        if hasattr(i2c, "try_lock"):
            if not i2c.try_lock(): return False
            try:
                lst = i2c.scan() or []
                return addr in lst
            finally:
                i2c.unlock()
        return True
    except Exception:
        return False

def _wait_for_addr(i2c, addr: int, timeout_s: float, interval_s: float = 0.05) -> bool:
    t0 = time.time()
    while time.time() - t0 < max(0.05, timeout_s):
        if _bus_has_addr(i2c, addr): return True
        time.sleep(interval_s)
    return _bus_has_addr(i2c, addr)

def _raw_set_address_confirm(i2c, new_addr: int, retries: int, gap_s: float) -> bool:
    """
    เขียนรีจิสเตอร์ 0x8A -> new_addr (7 บิต) พร้อมยืนยันว่าที่อยู่ใหม่โผล่จริงบนบัส
    """
    def _write_once():
        try:
            from smbus2 import SMBus  # เร็วและนิ่งกว่า ถ้ามี
            with SMBus(1) as bus:
                bus.write_byte_data(0x29, 0x8A, new_addr & 0x7F)
            return
        except Exception:
            from adafruit_bus_device.i2c_device import I2CDevice
            with I2CDevice(i2c, 0x29) as dev:
                dev.write(bytes([0x8A, new_addr & 0x7F]))

    for _ in range(max(1, retries)):
        try:
            _write_once()
        except Exception:
            time.sleep(gap_s)
            continue

        time.sleep(gap_s)  # ให้บัสนิ่งก่อนสแกน
        if _wait_for_addr(i2c, new_addr & 0x7F, timeout_s=1.0):
            return True
        time.sleep(gap_s)
    return False

def _adafruit_start(sensor, timing_budget_us: int):
    if hasattr(sensor, "measurement_timing_budget"):
        try: sensor.measurement_timing_budget = timing_budget_us
        except Exception: pass
    for m in ("start_continuous", "startContinuous"):
        if hasattr(sensor, m):
            try: getattr(sensor, m)()
            except Exception: pass
            break

# ---------- main: init N sensors with XSHUT ----------

def init_vl53x_four(
    xshut: XshutDriver,
    i2c,
    new_addrs,
    *,
    bus: int = 1,
    timing_budget_us: int = 20000,
    allow_fallback: bool = True,
    extra_boot_delay_s: float = 0.30,
    probe_retries: int = 3,
    probe_interval_s: float = 0.12,
    boot_timeout_s: float = 1.2,
) -> Dict[int, SensorHandle]:
   

    log.info("==== init_vl53x_four ====")
    handles: Dict[int, SensorHandle] = {}
    DEFAULT = 0x29

    # ปิดทุกตัวแล้วเริ่มเฟส A: เปิดทีละตัวเพื่อ 'ตั้งที่อยู่'
    xshut.all_low()
    time.sleep(0.03)

    assigned: Dict[int, int] = {}  # idx -> addr

    for idx, target_addr in enumerate(new_addrs):
        xshut.one_high(idx)
        time.sleep(0.35 + extra_boot_delay_s)

        # รอให้ 0x29 โผล่ก่อน
        if not _wait_for_addr(i2c, DEFAULT, timeout_s=boot_timeout_s):
            log.error("idx%d: default 0x%02X not found on bus after power-up", idx, DEFAULT)
            try: xshut.set_low(xshut.pins[idx])
            except Exception: pass
            time.sleep(0.05)
            continue

        # ตั้งที่อยู่ด้วย 0x8A (วนรีทรายภายใน)
        try:
            ok = _raw_set_address_confirm(
                i2c, target_addr,
                retries=int(os.getenv("VL53_ADDR_SET_RETRIES","6")),
                gap_s=float(os.getenv("VL53_ADDR_SET_GAP_S","0.10"))
            )
            if ok:
                log.info("idx%d: raw-set-addr OK -> 0x%02X", idx, target_addr)
                assigned[idx] = target_addr
            else:
                log.error("idx%d: set addr to 0x%02X failed", idx, target_addr)
                try: xshut.set_low(xshut.pins[idx])
                except Exception: pass
        except Exception as e:
            log.warning("idx%d: raw-set-addr exception: %r", idx, e)
            try: xshut.set_low(xshut.pins[idx])
            except Exception: pass

        time.sleep(0.02)

    # เฟส B: หลังตั้ง addr ครบแล้ว ค่อยเปิด reader ทีละตัว
    # ยก XSHUT ทุกตัวให้ HIGH เพื่อความเสถียรและปล่อยบัสนิ่งก่อน
    try:
        for k in range(len(xshut.pins)): xshut.set_high(k)
    except Exception:
        pass

    time.sleep(float(os.getenv("VL53_POST_SET_DELAY_S","0.5")))  # ให้เวลานิ่งทั้งบัส

    for idx, target_addr in assigned.items():
        # ยืนยันว่า addr ใหม่ยังมองเห็นอยู่
        _wait_for_addr(i2c, target_addr, timeout_s=max(1.0, boot_timeout_s))
        time.sleep(float(os.getenv("VL53_OPEN_DELAY_PRE_S","0.2")))

        try:
            s1 = _open_reader_with_retries(
                i2c, target_addr, timing_budget_us,
                retries=int(os.getenv("VL53_OPEN_RETRIES","10")),
                delay_s=float(os.getenv("VL53_OPEN_DELAY_S","0.3"))
            )
            handles[idx] = SensorHandle(
                idx=idx, addr=target_addr, backend="adafruit",
                handle=s1, read=_adafruit_read_factory(s1)
            )
            log.info("idx%d: open@0x%02X OK (reader ready)", idx, target_addr)
        except Exception as e:
            log.error("idx%d: open reader at 0x%02X FAILED: %r", idx, target_addr, e)

    log.info("==== init_vl53x_four: %d/%d ready ====", len(handles), len(new_addrs))
    return handles



def read_mm(handles: Dict[int, SensorHandle], idx: int) -> Optional[int]:
    h = handles.get(idx)
    return h.read() if h else None

def debug_summary(handles: Dict[int, SensorHandle]) -> str:
    return " ".join([f"[{i}]0x{h.addr:02X}/{h.backend}" for i, h in sorted(handles.items())]) or "(none)"


def _open_reader_with_retries(i2c, addr: int, timing_budget_us: int,
                              retries: int = 10, delay_s: float = 0.3):
    last = None
    for _ in range(max(1, retries)):
        try:
            s = _adafruit_open(i2c, addr)
            if hasattr(s, "measurement_timing_budget"):
                try: s.measurement_timing_budget = timing_budget_us
                except Exception: pass
            # เริ่มโหมดต่อเนื่องถ้ามี
            for m in ("start_continuous", "startContinuous"):
                if hasattr(s, m):
                    try: getattr(s, m)()
                    except Exception: pass
                    break
            return s
        except Exception as e:
            last = e
            time.sleep(delay_s)
    raise last if last else OSError(f"open reader failed @0x{addr:02X}")
