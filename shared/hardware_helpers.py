# ✅ shared/hardware_helpers.py
# 🔧 รวมฟังก์ชันและ global ที่เกี่ยวข้องกับการควบคุม hardware: I2C, Servo, Sensor, MCP23017

# =============================================================================
# 1) Imports
# =============================================================================
import time
import os
import board
import busio
import digitalio
from collections import deque
from digitalio import Direction
from adafruit_pca9685 import PCA9685
from adafruit_mcp230xx.mcp23017 import MCP23017
import adafruit_vl53l0x
import json
import statistics


# =============================================================================
# 2) I2C & Global Hardware Interface
# =============================================================================
shared_i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(shared_i2c)
pca.frequency = 50

vl53_sensors = []
buffers = []
last_values = []
mcp = None
mcp_pins = []
relay_pins = []


# =============================================================================
# 3) SERVO CONTROL
# =============================================================================
def angle_to_duty_cycle(angle):
    pulse_us = 500 + (angle / 180.0) * 2000
    return int((pulse_us / 20000.0) * 65535)

def move_servo_180(channel, angle):
    duty_cycle = angle_to_duty_cycle(angle)
    print(f"  → SG90-180° CH{channel} → {angle}° (duty: {duty_cycle})")
    pca.channels[channel].duty_cycle = duty_cycle
    time.sleep(0.7)
    pca.channels[channel].duty_cycle = 0


# =============================================================================
# 4) SENSOR (VL53L0X) อ่านค่า + simple smoothing
# =============================================================================
CHANGE_THRESHOLD = 5
NEAR_SENSOR_THRESHOLD = 80
BUFFER_SIZE = 5

def read_sensor(sensor_index):
    try:
        sensor = vl53_sensors[sensor_index]
        raw = sensor.range
        buffers[sensor_index].append(raw)
        avg = sum(buffers[sensor_index]) / len(buffers[sensor_index])
        if last_values[sensor_index] is None or abs(avg - last_values[sensor_index]) >= CHANGE_THRESHOLD:
            last_values[sensor_index] = avg
        stable = last_values[sensor_index]

        if stable < NEAR_SENSOR_THRESHOLD:
            return -1

        return stable
    except:
        return -1


# =============================================================================
# 5) DOOR SENSOR (MC-38) Debounce/Confirm Close
# =============================================================================
def is_door_reliably_closed(index, samples=20, interval=0.03):
    results = []
    for _ in range(samples):
        val = mcp_pins[8 + index].value
        results.append(val)
        time.sleep(interval)
    false_count = results.count(False)
    return false_count == 0


# =============================================================================
# 6) MCP23017 INIT (รีเลย์ + door switch)
# =============================================================================
def init_mcp():
    global mcp, mcp_pins, relay_pins
    mcp = MCP23017(shared_i2c)
    relay_pin_nums = [12, 13, 14, 15]
    door_switch_pins = [8, 9, 10, 11]
    mcp_pins.clear()
    relay_pins.clear()
    for pin_num in range(16):
        pin = mcp.get_pin(pin_num)
        if pin_num in relay_pin_nums:
            pin.direction = Direction.OUTPUT
            pin.value = False
            relay_pins.append(pin)
        elif pin_num in door_switch_pins:
            pin.direction = Direction.INPUT
            pin.pull_up = False
        else:
            pin.direction = Direction.OUTPUT
        mcp_pins.append(pin)


# =============================================================================
# 7) VL53L0X INIT (XSHUT multi-sensor addressing)
# =============================================================================
def reset_vl53_addresses():
    for i, x in enumerate(XSHUT_PINS):
        x.value = True
        time.sleep(0.1)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(shared_i2c, address=ADDRESS_BASE + i)
            sensor.set_address(0x29)
        except:
            pass
        x.value = False
    time.sleep(0.2)


# ---------- Config เฉพาะงาน 0–200 mm ----------
TIMING_BUDGET_US = 20_000   # 20ms เร็วขึ้น (เทียบกับ default ~33ms)
USE_CONTINUOUS_MODE = True  # ลด latency ของการอ่านค่า
TARGET_MIN_MM = 0
TARGET_MAX_MM = 200

SMOOTH_WINDOW = 15          # ขนาดหน้าต่างค่าล่าสุด (median/mean)
OUTLIER_MM = 15             # ถ้าค่าใหม่โดดจาก median เกินนี้ ให้ทิ้ง/จำกัด
CHANGE_THRESHOLD = 2        # mm: เมื่อส่งออก ถ้าไม่เปลี่ยนเกินค่านี้จะถือว่าเสถียร

XSHUT_PINS = [digitalio.DigitalInOut(pin) for pin in [board.D17, board.D27, board.D22, board.D5]]
ADDRESS_BASE = 0x30  # 0x30..0x33 — จะตั้งทีละตัวตามลำดับ


_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(_THIS_DIR, "vl53_calibration.json")

calib_profile = {"sensors": []}

def _default_calib_entry(addr):
    return {"addr": addr, "a": 1.0, "b": 0.0}

def load_calibration():
    global calib_profile
    try:
        with open(CALIBRATION_FILE, "r") as f:
            calib_profile = json.load(f)
    except Exception:
        calib_profile = {"sensors": []}

def save_calibration():
    try:
        with open(CALIBRATION_FILE, "w") as f:
            json.dump(calib_profile, f, indent=2)
    except Exception as e:
        print(f"⚠️ Failed to save calibration: {e}")

def _get_calib_by_addr(addr):
    for ent in calib_profile.get("sensors", []):
        if ent.get("addr") == addr:
            return ent
    ent = _default_calib_entry(addr)
    calib_profile["sensors"].append(ent)
    return ent

def init_xshuts():
    for x in XSHUT_PINS:
        x.switch_to_output(value=False)  # LOW = OFF
    time.sleep(0.2)

def init_sensors():
    global vl53_sensors, buffers, last_values
    vl53_sensors.clear()
    buffers.clear()
    last_values.clear()

    load_calibration()
    init_xshuts()

    for i, x in enumerate(XSHUT_PINS):
        try:
            x.value = True  # HIGH = ON
            time.sleep(0.08)

            sensor = adafruit_vl53l0x.VL53L0X(shared_i2c)
            sensor.measurement_timing_budget = TIMING_BUDGET_US  # 20ms for 0–200mm use-case

            new_addr = ADDRESS_BASE + i
            sensor.set_address(new_addr)

            if USE_CONTINUOUS_MODE:
                try:
                    sensor.start_continuous()
                except Exception as ce:
                    print(f"⚠️ start_continuous failed on 0x{new_addr:02X}: {ce}")

            vl53_sensors.append(sensor)
            buffers.append(deque(maxlen=SMOOTH_WINDOW))
            last_values.append(None)

            _ = _get_calib_by_addr(new_addr)

            print(f"✅ Sensor {i} @ I2C 0x{new_addr:02X} ready (budget={TIMING_BUDGET_US}us)")
        except Exception as e:
            print(f"Error initializing sensor {i}: {e}")
            x.value = False
            time.sleep(0.05)

    for x in XSHUT_PINS:
        x.value = True

    save_calibration()

    if not vl53_sensors:
        print("⚠️ No sensors found. Trying I2C reset...")
        reset_i2c_bus()
        init_sensors()


# =============================================================================
# 8) I2C Bus Reset helper
# =============================================================================
def reset_i2c_bus():
    os.system("sudo i2cdetect -y 1 > /dev/null 2>&1")
    time.sleep(0.5)


# =============================================================================
# 9) Calibration & Reading Utils
# =============================================================================
def _apply_outlier_reject(sensor_index, mm_value):
    buf = buffers[sensor_index]
    if len(buf) >= 3:
        m = statistics.median(buf)
        if abs(mm_value - m) > OUTLIER_MM:
            return int(m + (OUTLIER_MM if mm_value > m else -OUTLIER_MM))
    return mm_value

def _smooth_and_stabilize(sensor_index, mm_value):
    buffers[sensor_index].append(mm_value)
    stable = int(statistics.median(buffers[sensor_index]))
    if last_values[sensor_index] is None or abs(stable - last_values[sensor_index]) >= CHANGE_THRESHOLD:
        last_values[sensor_index] = stable
    return last_values[sensor_index]

def _apply_calibration(addr, mm_value):
    ent = _get_calib_by_addr(addr)
    a = ent.get("a", 1.0)
    b = ent.get("b", 0.0)
    y = a * float(mm_value) + b
    if y < TARGET_MIN_MM: y = TARGET_MIN_MM
    if y > TARGET_MAX_MM: y = TARGET_MAX_MM
    return int(round(y))

def read_sensor_calibrated(sensor_index):
    try:
        sensor = vl53_sensors[sensor_index]
        raw = int(sensor.range)

        if raw <= 0 or raw > 4000:
            return None

        raw = _apply_outlier_reject(sensor_index, raw)
        stable = _smooth_and_stabilize(sensor_index, raw)
        calibrated = _apply_calibration(sensor._device.device_address, stable)
        return calibrated
    except Exception as e:
        return None

def read_sensor(sensor_index):
    val = read_sensor_calibrated(sensor_index)
    return -1 if val is None else val
