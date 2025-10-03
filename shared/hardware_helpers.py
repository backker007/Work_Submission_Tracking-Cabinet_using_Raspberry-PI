# ✅ shared/hardware_helpers.py
# รวมทุกส่วน I2C / MCP23017 / PCA9685 / Door / Servo / VL53L0X (พร้อม ENV overrides)
# เพิ่มโหมด VL53_BUS_MODE=single : เปิดทีละตัวตอนอ่าน (เลี่ยง pull-up รวม)
from __future__ import annotations
import os, time
from collections import deque

import board, busio, digitalio
from digitalio import Direction
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_mcp230xx.mcp23017 import MCP23017
from adafruit_pca9685 import PCA9685
import adafruit_vl53l0x

def D(n: int): return getattr(board, f"D{n}")

# ---------------- I2C ----------------
shared_i2c = busio.I2C(board.SCL, board.SDA)
def reset_i2c_bus():
    try:
        while shared_i2c.try_lock():
            shared_i2c.unlock()
    except Exception: pass
    time.sleep(0.25)

# ---------------- MCP23017 ----------------
MCP23017_ADDR = int(os.getenv("MCP23017_ADDR", "0x20"), 16)
DOOR_INPUT_PINS = [0,1,2,3]
RELAY_OUTPUT_PINS = [0,1,2,3]
_env_door = os.getenv("MCP23017_DOOR_PINS")
_env_relay = os.getenv("MCP23017_RELAY_PINS")
if _env_door:  DOOR_INPUT_PINS  = [int(s) for s in _env_door.split(",") if s.strip()]
if _env_relay: RELAY_OUTPUT_PINS= [int(s) for s in _env_relay.split(",") if s.strip()]
mcp: MCP23017|None = None
mcp_pins: list[digitalio.DigitalInOut] = []
relay_pins: list[digitalio.DigitalInOut] = []
def init_mcp():
    global mcp, mcp_pins, relay_pins
    mcp = MCP23017(shared_i2c, address=MCP23017_ADDR)
    mcp_pins=[]
    for ch in DOOR_INPUT_PINS:
        p=mcp.get_pin(ch); p.direction=Direction.INPUT; p.pull=digitalio.Pull.UP; mcp_pins.append(p)
    relay_pins=[]
    for ch in RELAY_OUTPUT_PINS:
        p=mcp.get_pin(8+ch); p.switch_to_output(value=False); relay_pins.append(p)
    print("✅ MCP23017 initialized")
def set_relay(index:int, value:bool)->None:
    if 0<=index<len(relay_pins): relay_pins[index].value=bool(value)
def is_door_closed(index:int)->bool:
    if 0<=index<len(mcp_pins): return bool(mcp_pins[index].value)
    return False
def is_door_reliably_closed(index:int, samples:int=12, interval:float=0.03)->bool:
    trues=0
    for _ in range(samples):
        if is_door_closed(index): trues+=1
        time.sleep(interval)
    return trues>=int(0.8*samples)

# ---------------- PCA9685 (Servo) ----------------
PCA9685_ADDR = int(os.getenv("PCA9685_ADDR","0x40"),16)
SERVO_CHANNELS = [0,1,2,3]
_env_srv = os.getenv("SERVOS_CHANNELS")
if _env_srv: SERVO_CHANNELS=[int(s) for s in _env_srv.split(",") if s.strip()]
pca: PCA9685|None = None
def _ensure_pca():
    global pca
    if pca is None:
        pca = PCA9685(shared_i2c, address=PCA9685_ADDR); pca.frequency=50
def _us_to_pwm_on_off(us:float)->tuple[int,int]:
    period_us=20000.0; ticks_per_us=4096.0/period_us
    width=int(us*ticks_per_us); width=max(0,min(4095,width)); return (0,width)
def move_servo_180(index:int, degrees:float)->None:
    _ensure_pca()
    if index<0 or index>=len(SERVO_CHANNELS): return
    ch=SERVO_CHANNELS[index]
    min_us=float(os.getenv("SERVO_MIN_US","500")); max_us=float(os.getenv("SERVO_MAX_US","2500"))
    deg=max(0.0,min(180.0,float(degrees)))
    pulse_us=min_us+(max_us-min_us)*(deg/180.0)
    _on,off=_us_to_pwm_on_off(pulse_us); pca.channels[ch].duty_cycle=off

# ---------------- VL53L0X ----------------
_default_pins=[17,27,22,5]
_pins_env=os.getenv("VL53_XSHUT_PINS")
_pins=[int(s) for s in _pins_env.split(",") if s.strip()] if _pins_env else _default_pins
XSHUT_PINS=[digitalio.DigitalInOut(D(p)) for p in _pins]

ADDRESS_BASE     = int(os.getenv("VL53_ADDR_BASE","0x30"),16)  # 0x30..0x33
BOOT_DELAY_S     = float(os.getenv("VL53_BOOT_DELAY_S","1.2"))
BOOT_TIMEOUT_S   = float(os.getenv("VL53_BOOT_TIMEOUT_S","1.6"))
TIMING_BUDGET_US = int(os.getenv("VL53_BUDGET_US","33000"))
USE_CONTINUOUS   = os.getenv("VL53_CONTINUOUS","1") in ("1","true","True","YES","yes")
KEEP_ADDRESSES   = os.getenv("VL53_KEEP_ON_EXIT","1") in ("1","true","True","YES","yes")
BUS_MODE         = os.getenv("VL53_BUS_MODE","multi").lower()  # 'multi' | 'single'  ← ใหม่!

TARGET_MIN_MM    = 20
TARGET_MAX_MM    = 30
SMOOTH_WINDOW    = 5
OUTLIER_MM       = 15
CHANGE_THRESHOLD = 5  # export ให้ main ใช้ได้

# ---- I2C helpers
def _i2c_scan()->set[int]:
    try:
        shared_i2c.try_lock(); return set(shared_i2c.scan())
    finally:
        try: shared_i2c.unlock()
        except Exception: pass
def _i2c_scan_has(addr:int)->bool: return addr in _i2c_scan()
def _wait_addr(addr:int, timeout:float, poll=0.06, need_consecutive:int=2)->bool:
    seen=0
    t0=time.time()
    while time.time()-t0<timeout:
        if _i2c_scan_has(addr):
            seen+=1
            if seen>=need_consecutive:
                return True
        else:
            seen=0
        time.sleep(poll)
    return False
def _write_u8(addr:int, reg:int, val:int)->None:
    dev=I2CDevice(shared_i2c, addr)
    with dev: dev.write(bytes([reg&0xFF, val&0x7F]))

# ---- XSHUT
def init_xshuts():
    for x in XSHUT_PINS: x.switch_to_output(value=False)
    time.sleep(0.3)
    print(f"✅ XSHUT pins initialized (all LOW) → pins={_pins}")
    if BUS_MODE=="multi":
        print("🔧 INLINE BOOT ACTIVE (multi-device)")
    else:
        print("🧰 SINGLE-ACTIVE mode (software workaround)")

def _reset_all_to_0x29():
    for x in XSHUT_PINS: x.value=False
    time.sleep(0.35)
    for x in XSHUT_PINS: x.value=True
    time.sleep(0.6)
    print("🔄 VL53 reset XSHUT LOW→HIGH (กลับเป็น 0x29)")

# ---- Open driver retries
def _open_sensor_with_retries(addr:int, tries:int=10, sleep_s:float=0.3):
    last=None
    for _ in range(tries):
        if not _wait_addr(addr, timeout=0.25, need_consecutive=2):
            time.sleep(sleep_s); continue
        try:
            s=adafruit_vl53l0x.VL53L0X(shared_i2c, address=addr)
            try: _=s.range
            except Exception: pass
            return s
        except Exception as e:
            last=e; time.sleep(sleep_s)
    raise last if last else OSError(f"VL53L0X 0x{addr:02X} not found")

# =========================
# A) โหมด MULTI (เดิม): INLINE assign→open
# =========================
def _assign_open_one(i:int, x:digitalio.DigitalInOut, target:int):
    """
    LIGHT-STYLE inline assign:
    - XSHUT ตัวนี้ขึ้น HIGH
    - ถ้ามี target อยู่แล้ว -> เปิดเลย
    - ไม่กด XSHUT ลงซ้ำ, เขียน 0x8A ตรง ๆ ที่ 0x29 แล้วรอให้ target โผล่
    """
    # ปลุกตัวนี้
    x.value = True
    time.sleep(BOOT_DELAY_S)

    # ถ้าเคยอยู่ที่ target อยู่แล้ว ให้เปิดเลย
    if _wait_addr(target, timeout=0.8, need_consecutive=2):
        return _open_sensor_with_retries(target, tries=10, sleep_s=0.25)

    # รอ 0x29 โผล่ (บูตเสร็จ)
    if not _wait_addr(0x29, timeout=max(BOOT_TIMEOUT_S, 2.0), need_consecutive=2):
        raise OSError(f"idx{i}: 0x29 not seen")

    # เขียน 0x8A -> target (7-bit address ใช้ 7 LSB)
    _write_u8(0x29, 0x8A, target)
    time.sleep(0.20)

    # รอการย้ายที่อยู่ให้เสถียร (เห็น target)
    t0 = time.time()
    moved = False
    while time.time() - t0 < 2.5:
        if _i2c_scan_has(target):
            moved = True
            break
        time.sleep(0.08)

    if not moved:
        raise OSError(f"idx{i}: move->0x{target:02X} not stable")

    # เปิดไดรเวอร์ที่ที่อยู่ใหม่
    return _open_sensor_with_retries(target, tries=10, sleep_s=0.25)


def _reassign_and_open(i:int, x:digitalio.DigitalInOut, target:int):
    """
    Recovery แบบเบา: power-cycle ตัวเดียว แล้วทำ light-style assign เดิมซ้ำ
    (ยังคง "ไม่" isolate โดยการกด XSHUT ลงระหว่าง assign)
    """
    x.value = False
    time.sleep(0.40)
    x.value = True
    time.sleep(max(BOOT_DELAY_S, 1.0))
    # ถ้าบูตมาแล้ว target มีอยู่ -> เปิดเลย
    if _wait_addr(target, timeout=0.8, need_consecutive=2):
        return _open_sensor_with_retries(target, tries=10, sleep_s=0.25)

    if not _wait_addr(0x29, timeout=max(BOOT_TIMEOUT_S, 2.2), need_consecutive=2):
        raise OSError(f"idx{i}: 0x29 not seen after re-power")

    _write_u8(0x29, 0x8A, target)
    time.sleep(0.20)

    t0 = time.time()
    while time.time() - t0 < 2.5:
        if _i2c_scan_has(target):
            return _open_sensor_with_retries(target, tries=10, sleep_s=0.25)
        time.sleep(0.08)

    raise OSError(f"idx{i}: reassign->0x{target:02X} not stable")

# ---- Runtime state (ใช้ร่วมกัน)
vl53_sensors: list[adafruit_vl53l0x.VL53L0X|None] = []
buffers: list[deque] = []
last_values: list[int|None] = []

# ---- Filters
def _apply_outlier_reject(idx:int, raw:int)->int:
    prev=last_values[idx]
    if prev is not None and abs(raw-prev)>OUTLIER_MM: return prev
    return raw
def _smooth_and_stabilize(idx:int, raw:int)->int:
    buffers[idx].append(raw); s=sorted(buffers[idx]); median=s[len(s)//2]
    prev=last_values[idx]
    if prev is None or abs(median-prev)>=CHANGE_THRESHOLD:
        last_values[idx]=median; return median
    return prev
def _clamp_to_range(mm:int)->int:
    return max(TARGET_MIN_MM, min(TARGET_MAX_MM, mm))

# =========================
# B) โหมด SINGLE-ACTIVE (ใหม่): ปลุกทีละตัวตอนอ่าน
# =========================
def _single_boot_and_read(index: int) -> int:
    """
    SINGLE-ACTIVE:
    - ดึง XSHUT ทุกตัวลง -> ปลุกเฉพาะ index
    - รอ 0x29 เสถียร
    - เปิดไดรเวอร์ 0x29 -> start_continuous สั้น ๆ -> อ่าน median หลายครั้ง
    - ปิด XSHUT ตัวนี้ แล้วคืนค่า (หรือ -1)
    """
    # ปิดทุกตัวก่อน
    for x0 in XSHUT_PINS:
        x0.value = False
    time.sleep(0.18)

    x = XSHUT_PINS[index]

    for attempt in range(3):  # ลองสูงสุด 3 รอบ
        # ปลุกเฉพาะตัวนี้
        x.value = True
        time.sleep(max(BOOT_DELAY_S, 1.2))  # ขยายให้บูตเสถียรขึ้น

        # รอ 0x29 โผล่ติดกัน >=3 ครั้ง
        if not _wait_addr(0x29, timeout=max(BOOT_TIMEOUT_S, 3.0), poll=0.06, need_consecutive=3):
            # power-cycle แล้วลองใหม่
            x.value = False
            time.sleep(0.25)
            continue

        val = -1
        try:
            # เปิดไดรเวอร์ที่ 0x29
            s = _open_sensor_with_retries(0x29, tries=10, sleep_s=0.2)
            try:
                s.measurement_timing_budget = TIMING_BUDGET_US
            except Exception:
                pass

            # วอร์มด้วย continuous burst สั้น ๆ แล้วอ่าน median
            try:
                s.start_continuous()
                time.sleep(0.12)  # ให้เริ่มยิงก่อนเล็กน้อย
                val = _read_vl53_single_shot(s, samples=8, delay_s=0.03)
            finally:
                try:
                    s.stop_continuous()
                except Exception:
                    pass

            # ถ้ายังไม่ได้ค่า valid ลองอ่านซ้ำอีกรอบสั้น ๆ
            if val == -1:
                val = _read_vl53_single_shot(s, samples=6, delay_s=0.04)

        except Exception:
            val = -1

        # ปิดเซ็นเซอร์ตัวนี้เสมอ เพื่อตัด pull-up รวม
        x.value = False
        time.sleep(0.03)

        if val != -1:
            return _clamp_to_range(val)

        # ลองใหม่ (power-cycle) ถ้าอ่านไม่ได้
        time.sleep(0.2)

    return -1

def _read_vl53_single_shot(s, samples: int = 8, delay_s: float = 0.03) -> int:
    """
    อ่านหลายครั้ง ลดโอกาสได้ 0/invalid แล้วคืน median ของค่า 1..2000
    ใช้ได้ทั้งตอน continuous และ single-shot
    """
    vals = []
    for _ in range(max(1, samples)):
        try:
            v = int(s.range)
            if 1 <= v <= 2000:
                vals.append(v)
        except Exception:
            pass
        time.sleep(delay_s)
    if not vals:
        return -1
    vals.sort()
    return vals[len(vals)//2]


# --------- Init sensors ----------
def init_sensors():
    vl53_sensors.clear(); buffers.clear(); last_values.clear()
    init_xshuts()
    if BUS_MODE=="single":
        # ไม่สร้างไดรเวอร์ถาวร ปล่อย None ไว้หมด
        for _ in XSHUT_PINS:
            vl53_sensors.append(None)
            buffers.append(deque(maxlen=SMOOTH_WINDOW))
            last_values.append(None)
        print("🧰 SINGLE-ACTIVE: will boot & read each sensor on-demand at 0x29")
        return

    # ---- โหมด multi (เดิม)
    for i, x in enumerate(XSHUT_PINS):
        target = ADDRESS_BASE + i
        try:
            print(f"\n→ idx{i}: XSHUT HIGH, target 0x{target:02X}")
            s = _assign_open_one(i, x, target)
            vl53_sensors.append(s)
            buffers.append(deque(maxlen=SMOOTH_WINDOW))
            last_values.append(None)
            print(f"✅ Sensor {i} @ I2C 0x{target:02X} ready (inline)")
        except Exception as e1:
            print(f"⚠️ Sensor {i} inline open failed: {e1} → recovery…")
            try:
                s = _reassign_and_open(i, x, target)
                vl53_sensors.append(s)
                buffers.append(deque(maxlen=SMOOTH_WINDOW))
                last_values.append(None)
                print(f"✅ Sensor {i} recovered @ I2C 0x{target:02X} (inline)")
            except Exception as e:
                print(f"❌ Error initializing sensor {i}: {e}")
                vl53_sensors.append(None)
                buffers.append(deque(maxlen=SMOOTH_WINDOW))
                last_values.append(None)
                x.value=False
        time.sleep(0.25)

    if any(s is not None for s in vl53_sensors) and USE_CONTINUOUS:
        for i, s in enumerate(vl53_sensors):
            if s is None: continue
            try:
                try: s.measurement_timing_budget = TIMING_BUDGET_US
                except Exception: pass
                s.start_continuous()
                print(f"▶️  start_continuous: 0x{ADDRESS_BASE + i:02X}")
                time.sleep(0.05)
            except Exception as ce:
                print(f"⚠️ start_continuous failed on 0x{ADDRESS_BASE + i:02X}: {ce}")

# --------- Read safely ----------
def read_sensor(sensor_index: int) -> int:
    try:
        if BUS_MODE == "single":
            return _single_boot_and_read(sensor_index)
        # ---- โหมด multi (คงเดิม)
        if sensor_index >= len(vl53_sensors) or vl53_sensors[sensor_index] is None:
            return -1
        sensor = vl53_sensors[sensor_index]
        raw = int(sensor.range)
        if raw <= 0 or raw > 2000:
            return -1
        filtered = _apply_outlier_reject(sensor_index, raw)
        stable = _smooth_and_stabilize(sensor_index, filtered)
        return _clamp_to_range(stable)
    except Exception as e:
        print(f"⚠️ Error reading sensor {sensor_index}: {e}")
        return -1