# TEST_SENSOR/sensor_selftest.py
# -*- coding: utf-8 -*-
"""
VL53L0X self-test:
- คุม XSHUT ทีละตัว -> จับที่ 0x29 -> set_address() เป็น 0x30/0x31/0x32/0x33
- ถ้าเจอว่าตอบอยู่ที่ address เดิม (เช่น 0x30..33) อยู่แล้ว ก็ใช้ต่อได้เลย
- อ่านค่า range ของทุกตัว (samples ตามที่กำหนด)
- เมื่อจบสคริปต์ (default) จะ "ล้าง address" โดยดึง XSHUT ทุกตัว LOW -> HIGH เพื่อให้กลับ 0x29

การปรับแต่ง:
- เปลี่ยนรายการ GPIO ของ XSHUT ด้วย --pins (เช่น --pins 17,27,22 หรือ 17,27,22,5)
- เปลี่ยนจำนวนตัวอย่างที่อ่านด้วย --samples
- ถ้าไม่ต้องการล้าง address ตอนจบ ให้ใส่ --keep-address
"""

import argparse
import time
import statistics
import board
import busio
import digitalio
import adafruit_vl53l0x

DEFAULT_PINS = [17, 27, 22]          # XSHUT ใช้ GPIO17/27/22 (พินสีเหลืองในรูป)
ADDRESS_BASE = 0x30                   # จะได้ 0x30,0x31,0x32,0x33 ...
TIMING_BUDGET_US = 20_000             # 20ms เร็วขึ้น
BOOT_DELAY_S = 0.30                   # เวลารอหลังปลุก XSHUT
USE_CONTINUOUS_MODE = True

def d(gpio_num: int):
    """map int -> board.Dxx"""
    return getattr(board, f"D{gpio_num}")

def build_xshut(pins_nums):
    xs = [digitalio.DigitalInOut(d(n)) for n in pins_nums]
    for x in xs:
        x.switch_to_output(value=False)  # ปิดทุกตัวก่อน (LOW = shutdown)
    time.sleep(0.2)
    return xs

def probe_29(i2c):
    """ลองคุยกับ 0x29 (default). คืน obj หรือ None"""
    try:
        return adafruit_vl53l0x.VL53L0X(i2c)  # default addr = 0x29
    except Exception:
        return None

def probe_addr(i2c, addr):
    """ลองคุยกับ addr ที่กำหนด"""
    try:
        return adafruit_vl53l0x.VL53L0X(i2c, address=addr)
    except Exception:
        return None

def assign_or_attach(i2c, idx, leave_on=True):
    """
    ปลุก XSHUT ของ index นี้ -> พยายามจับ 0x29 แล้วตั้งเป็น ADDRESS_BASE+idx
    ถ้า 0x29 ไม่ตอบ ลองแนบไปที่ ADDRESS_BASE+idx (ในกรณีเคยตั้งไว้แล้ว)
    คืน (sensor_obj, address_int) หรือ (None, None)
    """
    addr_target = ADDRESS_BASE + idx
    # รอให้บูต
    time.sleep(BOOT_DELAY_S)

    # 1) กรณี default 0x29
    s = probe_29(i2c)
    if s:
        try:
            s.measurement_timing_budget = TIMING_BUDGET_US
        except Exception:
            pass
        try:
            s.set_address(addr_target)
            print(f"✅ idx{idx}: 0x29 -> 0x{addr_target:02X}")
            if USE_CONTINUOUS_MODE:
                try: s.start_continuous()
                except Exception: pass
            return s, addr_target
        except Exception as e:
            print(f"⚠️ idx{idx}: พบ 0x29 แต่ set_address ล้มเหลว: {e}")

    # 2) กรณีเคยตั้ง address แล้ว (เช่น บูตก่อนหน้า)
    s2 = probe_addr(i2c, addr_target)
    if s2:
        try:
            s2.measurement_timing_budget = TIMING_BUDGET_US
        except Exception:
            pass
        if USE_CONTINUOUS_MODE:
            try: s2.start_continuous()
            except Exception: pass
        print(f"ℹ️ idx{idx}: ใช้เซ็นเซอร์ที่มีอยู่แล้ว @0x{addr_target:02X}")
        return s2, addr_target

    # ไม่เจอทั้ง 0x29 และ 0x{base+idx}
    print(f"❌ idx{idx}: ไม่พบอุปกรณ์ที่ 0x29 หรือ 0x{addr_target:02X}")
    return None, None

def restore_all_default(xs):
    """ล้าง address: ดึง XSHUT ทุกตัว LOW -> HIGH เพื่อให้กลับ 0x29"""
    for x in xs: x.value = False
    time.sleep(0.25)
    for x in xs: x.value = True
    time.sleep(0.35)
    # ไม่ set_address ใด ๆ ทิ้งไว้ที่ 0x29 ทั้งหมด
    print("🔄 restore: รีเซ็ต XSHUT LOW->HIGH ทุกตัวแล้ว (กลับเป็น 0x29)")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pins", type=str, default=",".join(map(str, DEFAULT_PINS)),
                    help="รายการ GPIO สำหรับ XSHUT คั่นด้วย comma (เช่น 17,27,22 หรือ 17,27,22,5)")
    ap.add_argument("--samples", type=int, default=10, help="จำนวนตัวอย่างที่อ่านต่อเซ็นเซอร์")
    ap.add_argument("--keep-address", action="store_true",
                    help="ไม่ล้าง address ตอนจบ (คง 0x30..0x33 ไว้)")
    args = ap.parse_args()

    pins_nums = [int(s.strip()) for s in args.pins.split(",") if s.strip()]
    print(f"🔧 XSHUT GPIO pins: {pins_nums}")
    i2c = busio.I2C(board.SCL, board.SDA)

    xs = build_xshut(pins_nums)

    sensors = []
    addrs = []

    try:
        # เปิดทีละตัว + assign หรือ attach
        for idx, x in enumerate(xs):
            # เปิดเฉพาะตัวนี้ (ตัวก่อนหน้าให้ "คงเปิดไว้" เพื่อคง address เดิม)
            x.value = True
            s, addr = assign_or_attach(i2c, idx)
            sensors.append(s)
            addrs.append(addr)

        # สรุปที่พบ
        found = [(i, f"0x{a:02X}") for i, a in enumerate(addrs) if a is not None]
        if not found:
            print("⚠️ ไม่พบเซ็นเซอร์เลย")
        else:
            print("📋 พบเซ็นเซอร์:", ", ".join([f"idx{i}@{a}" for i, a in found]))

        # ทดสอบอ่านค่า
        for i, s in enumerate(sensors):
            if not s:
                print(f"— idx{i}: (no sensor)")
                continue
            vals = []
            for _ in range(args.samples):
                try:
                    vals.append(int(s.range))
                except Exception:
                    vals.append(-1)
                time.sleep(0.05)
            good = [v for v in vals if 0 < v < 2000]
            med = statistics.median(good) if good else None
            print(f"idx{i}@0x{addrs[i]:02X}: samples={vals}  median={med}")
    finally:
        if not args.keep-address:
            restore_all_default(xs)
        else:
            print("⏳ keep-address: คง 0x30..0x33 ไว้ตามที่ตั้งในรอบนี้")

if __name__ == "__main__":
    main()
