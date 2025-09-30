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
# 4) SENSOR (VL53L0X) Configuration สำหรับช่วง 0-200mm
# =============================================================================
# ช่วงการทำงานของโปรเจ็ค
TARGET_MIN_MM = 0
TARGET_MAX_MM = 200

# การตั้งค่าเซ็นเซอร์
TIMING_BUDGET_US = 20_000   # 20ms เร็วขึ้น (เทียบกับ default ~33ms)
USE_CONTINUOUS_MODE = True  # ลด latency ของการอ่านค่า

# การกรองสัญญาณ
SMOOTH_WINDOW = 5           # ขนาดหน้าต่าง median filter (5 ค่า)
OUTLIER_MM = 15             # ถ้าค่าใหม่โดดจาก median เกินนี้ ให้จำกัด
CHANGE_THRESHOLD = 5        # mm: ถ้าไม่เปลี่ยนเกินค่านี้จะถือว่าเสถียร

# XSHUT pins และ I2C addresses
XSHUT_PINS = [digitalio.DigitalInOut(pin) for pin in [board.D17, board.D27, board.D22, board.D5]]
ADDRESS_BASE = 0x30  # 0x30..0x33 — จะตั้งทีละตัวตามลำดับ


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
    print("✅ MCP23017 initialized")


# =============================================================================
# 7) VL53L0X INIT (XSHUT multi-sensor addressing)
# =============================================================================
def reset_vl53_addresses():
    """Reset all sensors back to default address 0x29"""
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
    print("🔄 VL53L0X addresses reset")


def init_xshuts():
    """Initialize XSHUT pins to LOW (sensors off)"""
    for x in XSHUT_PINS:
        x.switch_to_output(value=False)  # LOW = OFF
    time.sleep(0.2)
    print("✅ XSHUT pins initialized")


def init_sensors():
    """Initialize all VL53L0X sensors"""
    global vl53_sensors, buffers, last_values
    vl53_sensors.clear()
    buffers.clear()
    last_values.clear()

    init_xshuts()

    for i, x in enumerate(XSHUT_PINS):
        try:
            x.value = True  # HIGH = ON
            time.sleep(0.08)

            sensor = adafruit_vl53l0x.VL53L0X(shared_i2c)
            sensor.measurement_timing_budget = TIMING_BUDGET_US  # 20ms for faster reads

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

            print(f"✅ Sensor {i} @ I2C 0x{new_addr:02X} ready (budget={TIMING_BUDGET_US}us, range: {TARGET_MIN_MM}-{TARGET_MAX_MM}mm)")
        except Exception as e:
            print(f"❌ Error initializing sensor {i}: {e}")
            x.value = False
            time.sleep(0.05)

    # Turn all sensors back on
    for x in XSHUT_PINS:
        x.value = True

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
    print("🔄 I2C bus reset")


# =============================================================================
# 9) SENSOR READING WITH FILTERING
# =============================================================================
def _apply_outlier_reject(sensor_index, mm_value):
    """
    Reject outliers based on median of buffer
    ถ้าค่าใหม่ต่างจาก median เกิน OUTLIER_MM จะจำกัดค่าให้อยู่ในขอบเขต
    """
    buf = buffers[sensor_index]
    if len(buf) >= 3:
        m = statistics.median(buf)
        if abs(mm_value - m) > OUTLIER_MM:
            # Limit the value instead of rejecting completely
            return int(m + (OUTLIER_MM if mm_value > m else -OUTLIER_MM))
    return mm_value


def _smooth_and_stabilize(sensor_index, mm_value):
    """
    ใช้ median filter เพื่อกรองสัญญาณรบกวน
    และใช้ CHANGE_THRESHOLD เพื่อป้องกันการส่งค่าที่เปลี่ยนแปลงเล็กน้อย
    """
    buffers[sensor_index].append(mm_value)
    stable = int(statistics.median(buffers[sensor_index]))
    
    # Only update if change is significant
    if last_values[sensor_index] is None or abs(stable - last_values[sensor_index]) >= CHANGE_THRESHOLD:
        last_values[sensor_index] = stable
    
    return last_values[sensor_index]


def _clamp_to_range(mm_value):
    """จำกัดค่าให้อยู่ในช่วง TARGET_MIN_MM ถึง TARGET_MAX_MM"""
    if mm_value < TARGET_MIN_MM:
        return TARGET_MIN_MM
    if mm_value > TARGET_MAX_MM:
        return TARGET_MAX_MM
    return mm_value


def read_sensor(sensor_index):
    """
    อ่านค่าเซ็นเซอร์พร้อม filtering pipeline:
    1. อ่านค่า raw (แม่นอยู่แล้ว)
    2. Reject outliers
    3. Smooth ด้วย median filter
    4. Clamp ให้อยู่ในช่วง 0-200mm
    5. คืนค่า -1 เมื่อเกิด error
    """
    try:
        sensor = vl53_sensors[sensor_index]
        raw = int(sensor.range)

        # กรองค่าที่ผิดปกติชัดเจน (นอกช่วงการทำงานจริงของเซ็นเซอร์ 25-2000mm)
        if raw <= 0 or raw > 2000:
            return -1

        # Apply outlier rejection
        filtered = _apply_outlier_reject(sensor_index, raw)
        
        # Smooth and stabilize
        stable = _smooth_and_stabilize(sensor_index, filtered)
        
        # Clamp to project range (0-200mm)
        result = _clamp_to_range(stable)
        
        return result
        
    except Exception as e:
        print(f"⚠️ Error reading sensor {sensor_index}: {e}")
        return -1