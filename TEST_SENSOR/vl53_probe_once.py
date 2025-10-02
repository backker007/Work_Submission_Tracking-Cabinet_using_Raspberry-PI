# TEST_SENSOR/vl53_probe_once.py
# สแกน I2C ทีละตัวด้วย XSHUT: เปิดทีละตัวแล้วดูว่าขึ้น 0x29 ไหม
import time, os, board, busio, digitalio

PINS = [17, 27, 22, 5]
p_env = os.getenv("VL53_XSHUT_PINS")
if p_env:
    PINS = [int(s) for s in p_env.split(",") if s.strip()]

def D(n): return getattr(board, f"D{n}")

i2c = busio.I2C(board.SCL, board.SDA)
xs = [digitalio.DigitalInOut(D(p)) for p in PINS]
for x in xs: x.switch_to_output(value=False)
time.sleep(0.3)

def scan():
    try:
        i2c.try_lock()
        return set(i2c.scan())
    finally:
        try: i2c.unlock()
        except: pass

print(f"XSHUT pins: {PINS}")
for i,x in enumerate(xs):
    # ปิดหมด
    for y in xs: y.value = False
    time.sleep(0.15)
    # เปิดเฉพาะตัว i
    x.value = True
    time.sleep(1.0)
    addrs = sorted(list(scan()))
    print(f"idx{i} HIGH → I2C addrs = {[hex(a) for a in addrs]}")
