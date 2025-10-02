#!/usr/bin/env python3
import time, board, busio, digitalio

PINS = [17, 27, 22, 5]  # ให้ตรงกับ .env ของคุณ
def D(n): return getattr(board, f"D{n}")
i2c = busio.I2C(board.SCL, board.SDA)

def scan_hex():
    try:
        i2c.try_lock()
        return [f"0x{a:02X}" for a in sorted(i2c.scan())]
    finally:
        try: i2c.unlock()
        except Exception: pass

xs = [digitalio.DigitalInOut(D(p)) for p in PINS]
for x in xs: x.switch_to_output(value=False)
time.sleep(0.3)
print("ALL LOW  ->", scan_hex())

for idx, x in enumerate(xs):
    print(f"\nidx{idx} HIGH ...")
    x.value = True
    time.sleep(1.3)
    hi = scan_hex()
    print("scan HIGH  :", hi, " | 0x29 present?", ("0x29" in hi))
    x.value = False
    time.sleep(0.4)
    lo = scan_hex()
    print("scan back L:", lo, " | 0x29 present?", ("0x29" in lo))
