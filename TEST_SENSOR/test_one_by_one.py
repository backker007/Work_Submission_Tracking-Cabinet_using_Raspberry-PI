# TEST_SENSOR/addr_assign_once.py
import time, board, busio, digitalio, adafruit_vl53l0x
PINS=[board.D17,board.D27,board.D22,board.D5]; BASE=0x30
i2c=busio.I2C(board.SCL,board.SDA)
xs=[digitalio.DigitalInOut(p) for p in PINS]
for x in xs: x.switch_to_output(value=False); time.sleep(0.2)
for idx,x in enumerate(xs):
    x.value=True; time.sleep(0.3)
    try:
        s=adafruit_vl53l0x.VL53L0X(i2c) # 0x29
        s.set_address(BASE+idx)
        print(f"idx{idx} -> 0x{BASE+idx:02X}")
    except Exception as e:
        print(f"idx{idx} FAIL: {e}")
