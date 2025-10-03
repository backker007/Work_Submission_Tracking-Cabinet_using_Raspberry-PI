# shared/vl53l0x_init.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional
import os
import time
import logging

log = logging.getLogger("vl53x_init")

# ===== (Optional) native backend placeholder =====
try:
    import VL53L0X as vl53_native  # type: ignore[import-not-found]
except Exception:
    vl53_native = None  # ไม่ได้ใช้ในเวอร์ชันนี้ แต่เก็บไว้เผื่ออนาคต

# ===== Public handle =====
@dataclass
class SensorHandle:
    idx: int
    addr: int
    backend: str
    handle: object
    read: Callable[[], Optional[int]]

# ===== Simple XSHUT driver interface =====
@dataclass
class XshutDriver:
    """
    Wrapper ให้จัดการ XSHUT ผ่าน index:
    - pins: รายการ "index" (ไม่จำเป็นต้องเป็นหมายเลข GPIO จริง)
    - set_low / set_high: ฟังก์ชันที่รับ index แล้วดึง LOW/HIGH
    """
    pins: List[int]
    set_low: Callable[[int], None]
    set_high: Callable[[int], None]

    def all_low(self):
        for k in range(len(self.pins)):
            self.set_low(k)

    def all_high(self):
        for k in range(len(self.pins)):
            self.set_high(k)

    def one_high(self, index: int):
        self.set_high(index)

# ---------- Adafruit helpers ----------
def _adafruit_open(i2c, addr: int):
    import adafruit_vl53l0x
    return adafruit_vl53l0x.VL53L0X(i2c, address=addr)

def _adafruit_read_factory(sensor):
    def _read():
        try:
            v = sensor.range
            if isinstance(v, (int, float)):
                return int(v)
        except Exception:
            return None
        return None
    return _read

# ---------- I2C presence helpers ----------
def _bus_has_addr(i2c, addr: int) -> bool:
    try:
        if hasattr(i2c, "try_lock"):
            if not i2c.try_lock():
                return False
            try:
                lst = i2c.scan() or []
                return addr in lst
            finally:
                i2c.unlock()
        # ถ้าไม่มี try_lock (เช่นบาง mock) ให้ถือว่าผ่าน
        return True
    except Exception:
        return False

def _wait_for_addr(i2c, addr: int, timeout_s: float, interval_s: float = 0.05) -> bool:
    t0 = time.time()
    while time.time() - t0 < max(0.05, timeout_s):
        if _bus_has_addr(i2c, addr):
            return True
        time.sleep(interval_s)
    return _bus_has_addr(i2c, addr)

def _raw_set_address_confirm(i2c, new_addr: int, retries: int, gap_s: float) -> bool:
    """
    เขียนรีจิสเตอร์ 0x8A -> new_addr (7 บิต) แล้ว confirm ว่าที่อยู่ใหม่โผล่จริงบนบัส
    ใช้ smbus2 เป็นหลัก ถ้าไม่มี/ล้มเหลว fallback เป็น adafruit_bus_device
    """
    def _write_once():
        try:
            from smbus2 import SMBus
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

        time.sleep(gap_s)
        if _wait_for_addr(i2c, new_addr & 0x7F, timeout_s=1.0):
            return True
        time.sleep(gap_s)
    return False

# ---------- Reader open (with retries) ----------
def _open_reader_with_retries(i2c, addr: int, timing_budget_us: int,
                              retries: int = 10, delay_s: float = 0.3):
    last = None
    for _ in range(max(1, retries)):
        try:
            s = _adafruit_open(i2c, addr)
            # ตั้ง timing budget ถ้ามี
            if hasattr(s, "measurement_timing_budget"):
                try:
                    s.measurement_timing_budget = timing_budget_us
                except Exception:
                    pass
            # สั่งโหมดอ่านต่อเนื่องถ้าซัพพอร์ต
            for m in ("start_continuous", "startContinuous"):
                if hasattr(s, m):
                    try:
                        getattr(s, m)()
                    except Exception:
                        pass
                    break
            return s
        except Exception as e:
            last = e
            time.sleep(delay_s)
    raise last if last else OSError(f"open reader failed @0x{addr:02X}")

# ---------- Main: init N sensors with XSHUT ----------
def init_vl53x_four(
    xshut: XshutDriver,
    i2c,
    new_addrs,
    *,
    bus: int = 1,
    timing_budget_us: int = 20000,
    allow_fallback: bool = True,      # (reserved for future native fallback)
    extra_boot_delay_s: float = 0.30,
    probe_retries: int = 3,           # (reserved, ถ้าจะใช้ probe native)
    probe_interval_s: float = 0.12,   # (reserved)
    boot_timeout_s: float = 1.2,
) -> Dict[int, SensorHandle]:

    log.info("==== init_vl53x_four ====")
    handles: Dict[int, SensorHandle] = {}
    DEFAULT = 0x29

    # จำกัดจำนวนตามขา XSHUT และจำนวนที่อยู่เป้าหมาย
    n = min(len(xshut.pins), len(new_addrs))
    if n == 0:
        log.error("No sensors to initialize (XSHUT pins or new_addrs is empty)")
        return handles

    # 1) ดึง XSHUT ทั้งหมดลง LOW ก่อน
    xshut.all_low()
    time.sleep(0.03)

    # 2) เปิดทีละตัว → ยืนยันเจอที่ 0x29 → สั่งเปลี่ยน address → ยืนยันที่อยู่ใหม่
    assigned: Dict[int, int] = {}
    set_retries = int(os.getenv("VL53_ADDR_SET_RETRIES", "6"))
    set_gap_s   = float(os.getenv("VL53_ADDR_SET_GAP_S", "0.10"))

    for idx in range(n):
        target_addr = int(new_addrs[idx]) & 0x7F

        xshut.one_high(idx)
        time.sleep(0.35 + extra_boot_delay_s)

        if not _wait_for_addr(i2c, DEFAULT, timeout_s=boot_timeout_s):
            log.error("idx%d: default 0x%02X not found on bus after power-up", idx, DEFAULT)
            try:
                xshut.set_low(idx)
            except Exception:
                pass
            time.sleep(0.05)
            continue

        try:
            ok = _raw_set_address_confirm(
                i2c, target_addr,
                retries=set_retries,
                gap_s=set_gap_s
            )
            if ok:
                log.info("idx%d: raw-set-addr OK -> 0x%02X", idx, target_addr)
                assigned[idx] = target_addr
            else:
                log.error("idx%d: set addr to 0x%02X failed", idx, target_addr)
                try:
                    xshut.set_low(idx)
                except Exception:
                    pass
        except Exception as e:
            log.warning("idx%d: raw-set-addr exception: %r", idx, e)
            try:
                xshut.set_low(idx)
            except Exception:
                pass

        time.sleep(0.02)

    # 3) เปิด XSHUT ทุกตัวค้างไว้ (สำคัญมาก: ถ้าปิด LOW จะรีเซ็ตกลับ 0x29)
    try:
        xshut.all_high()
    except Exception:
        for k in range(n):
            try:
                xshut.set_high(k)
            except Exception:
                pass

    time.sleep(float(os.getenv("VL53_POST_SET_DELAY_S", "1.0")))

    # 4) สแกนบัสเพื่อ log ให้ดีบักง่าย
    try:
        if hasattr(i2c, 'scan'):
            present_addrs = i2c.scan()
            log.info("==== I2C Bus Scan before opening readers: %s ====", [hex(a) for a in present_addrs])
    except Exception as e:
        log.warning("Could not perform I2C scan: %r", e)

    # 5) เปิด reader ตามที่ตั้งแอดเดรสสำเร็จ (ต่อคิว, ใส่ delay ป้องกันช็อก)
    open_retries   = int(os.getenv("VL53_OPEN_RETRIES", "10"))
    open_delay_s   = float(os.getenv("VL53_OPEN_DELAY_S", "0.3"))
    open_delay_pre = float(os.getenv("VL53_OPEN_DELAY_PRE_S", "0.2"))

    for idx, target_addr in assigned.items():
        time.sleep(0.25)  # spacing ระหว่างตัวต่อไป
        _wait_for_addr(i2c, target_addr, timeout_s=max(1.0, boot_timeout_s))
        time.sleep(open_delay_pre)

        try:
            s1 = _open_reader_with_retries(
                i2c, target_addr, timing_budget_us,
                retries=open_retries, delay_s=open_delay_s
            )
            handles[idx] = SensorHandle(
                idx=idx, addr=target_addr, backend="adafruit",
                handle=s1, read=_adafruit_read_factory(s1)
            )
            log.info("idx%d: open@0x%02X OK (reader ready)", idx, target_addr)
        except Exception as e:
            log.error("idx%d: open reader at 0x%02X FAILED: %r", idx, target_addr, e)

    log.info("==== init_vl53x_four: %d/%d ready ====", len(handles), n)
    return handles

# ---------- Public utils ----------
def read_mm(handles: Dict[int, SensorHandle], idx: int) -> Optional[int]:
    h = handles.get(idx)
    return h.read() if h else None

def debug_summary(handles: Dict[int, SensorHandle]) -> str:
    return " ".join([f"[{i}]0x{h.addr:02X}/{h.backend}" for i, h in sorted(handles.items())]) or "(none)"
