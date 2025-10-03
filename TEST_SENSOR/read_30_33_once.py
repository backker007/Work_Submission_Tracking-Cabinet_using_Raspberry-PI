# TEST_SENSOR/read_30_33_once.py
import time, board, busio
import adafruit_vl53l0x

i2c = busio.I2C(board.SCL, board.SDA)
addrs = [0x30, 0x31, 0x32, 0x33]
sensors = []
for a in addrs:
    try:
        s = adafruit_vl53l0x.VL53L0X(i2c, address=a)
        sensors.append(s)
        print(f"OK open 0x{a:02X}")
    except Exception as e:
        sensors.append(None)
        print(f"FAIL open 0x{a:02X}: {e}")

for i, s in enumerate(sensors):
    if s is None: continue
    vals = []
    for _ in range(5):
        try:
            v = int(s.range)
            if 1 <= v <= 2000: vals.append(v)
        except: pass
        time.sleep(0.05)
    print(f"0x{addrs[i]:02X} -> {vals if vals else 'no valid'}")
