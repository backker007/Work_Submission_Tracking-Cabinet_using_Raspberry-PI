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

# =============================================================================
# 2) I2C & Global Hardware Interface
# =============================================================================
# ==== I2C & Global Hardware Interface ====
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
# ==== SERVO CONTROL ====
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
# ==== SENSOR ====
CHANGE_THRESHOLD = 5
NEAR_SENSOR_THRESHOLD = 10
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
            return -0

        return stable
    except:
        return -1

# =============================================================================
# 5) DOOR SENSOR (MC-38) Debounce/Confirm Close
# =============================================================================
# ==== DOOR SENSOR (MC-38 Debounce) ====
def is_door_reliably_closed(index, samples=20, interval=0.03):
    """
    ✅ ตรวจว่า sensor ให้ค่า True (ประตูปิด) ตลอดทั้ง samples ครั้ง
    ❗ ถ้ามี False เพียงตัวเดียว → ถือว่าไม่ปิด
    """
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
# ==== MCP23017 INIT ====
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
# ==== VL53L0X INIT ====
XSHUT_PINS = [digitalio.DigitalInOut(pin) for pin in [board.D17, board.D27, board.D22, board.D5]]
ADDRESS_BASE = 0x30

def init_xshuts():
    for x in XSHUT_PINS:
        x.direction = digitalio.Direction.OUTPUT
        x.value = False
    time.sleep(0.2)

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

def init_sensors():
    global vl53_sensors, buffers, last_values
    vl53_sensors.clear()
    buffers.clear()
    last_values.clear()
    for i, x in enumerate(XSHUT_PINS):
        x.value = True
        time.sleep(0.3)
        try:
            sensor = adafruit_vl53l0x.VL53L0X(shared_i2c)
            sensor.set_address(ADDRESS_BASE + i)
            sensor.measurement_timing_budget = 33000
            sensor.range = sensor.RANGE_SHORT  # ตั้งเป็น RANGE_SHORT สำหรับการวัดระยะใกล้
            sensor.inter_measurement_period = 50
            vl53_sensors.append(sensor)
            buffers.append(deque(maxlen=BUFFER_SIZE))
            last_values.append(None)
        except:
            x.value = False
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