# TEST_SENSOR/sensor_selftest_safe.py
# -*- coding: utf-8 -*-
import argparse, time, statistics, sys
import board, busio, digitalio
import adafruit_vl53l0x

ADDRESS_BASE      = 0x30           # จะตั้งเป็น 0x30,0x31,0x32,0x33 ตาม index
TIMING_BUDGET_US  = 20_000         # 20 ms
BOOT_DELAY_S      = 0.35           # หน่วงหลังยก XSHUT ให้บูตจริง
I2C_LOCK_TIMEOUT  = 3.0            # วินาที (กันค้างตอนเปิด I2C)
USE_CONTINUOUS    = True

def gpio_to_boardD(n: int):
    return getattr(board, f"D{n}")

def open_i2c_or_die(timeout=I2C_LOCK_TIMEOUT):
    print("➡️  เปิด I²C...", flush=True)
    i2c = busio.I2C(board.SCL, board.SDA)
    t0 = time.time()
    while True:
        try:
            if i2c.try_lock():
                addrs = i2c.scan()
                i2c.unlock()
                print(f"✅ I²C ready, พบอุปกรณ์: {[hex(a) for a in addrs]}", flush=True)
                return i2c
        except Exception:
            pass
        if time.time() - t0 > timeout:
            print("❌ I²C busy/ไม่ว่าง (อาจมีโปรเซสอื่นใช้งาน หรือบัสค้าง SCL LOW)")
            print("   -> หยุด service/สคริปต์อื่นก่อน แล้วลองใหม่")
            sys.exit(2)
        time.sleep(0.05)

def build_xshut(pins):
    xs = [digitalio.DigitalInOut(gpio_to_boardD(p)) for p in pins]
    for x in xs:
        x.switch_to_output(value=False)  # LOW = ปิดทุกตัว
    print("🔌 ตั้ง XSHUT = LOW ทุกตัวแล้ว", flush=True)
    time.sleep(0.2)
    return xs

def probe_default(i2c):
    try:
        return adafruit_vl53l0x.VL53L0X(i2c)  # ที่อยู่ 0x29
    except Exception:
        return None

def probe_addr(i2c, addr):
    try:
        return adafruit_vl53l0x.VL53L0X(i2c, address=addr)
    except Exception:
        return None

def attach_or_assign(i2c, idx):
    target = ADDRESS_BASE + idx
    print(f"   ⏳ idx{idx}: รอให้บูต {BOOT_DELAY_S:.2f}s...", flush=True)
    time.sleep(BOOT_DELAY_S)

    # 1) ลองที่ default 0x29 ก่อน
    s = probe_default(i2c)
    if s:
        print(f"   🟢 idx{idx}: พบที่ 0x29 → ตั้งเป็น 0x{target:02X}", flush=True)
        try:
            s.measurement_timing_budget = TIMING_BUDGET_US
        except Exception:
            pass
        s.set_address(target)
        if USE_CONTINUOUS:
            try: s.start_continuous()
            except Exception: pass
        return s, target

    # 2) ถ้าไม่ใช่ 0x29 อาจเคยตั้งไว้แล้ว
    s2 = probe_addr(i2c, target)
    if s2:
        print(f"   ℹ️  idx{idx}: ใช้ตัวที่เคยตั้งไว้ @0x{target:02X}", flush=True)
        try:
            s2.measurement_timing_budget = TIMING_BUDGET_US
        except Exception:
            pass
        if USE_CONTINUOUS:
            try: s2.start_continuous()
            except Exception: pass
        return s2, target

    print(f"   ❌ idx{idx}: ไม่พบทั้ง 0x29 และ 0x{target:02X}", flush=True)
    return None, None

def reset_all_to_default(xs):
    print("🔄 รีเซ็ต XSHUT: LOW → HIGH (คืนเป็น 0x29 ทั้งหมด)", flush=True)
    for x in xs: x.value = False
    time.sleep(0.3)
    for x in xs: x.value = True
    time.sleep(0.4)
    print("✅ รีเซ็ตเสร็จ", flush=True)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pins", default="17,27,22",
        help="รายการ GPIO สำหรับ XSHUT (เช่น 17,27,22 หรือ 17,27,22,5)")
    ap.add_argument("--samples", type=int, default=8,
        help="จำนวนตัวอย่างที่จะอ่านต่อเซ็นเซอร์")
    ap.add_argument("--keep-address", action="store_true",
        help="ไม่รีเซ็ต address ตอนจบ (คง 0x30..33)")
    args = ap.parse_args()

    pins = [int(s.strip()) for s in args.pins.split(",") if s.strip()]
    print(f"🔧 XSHUT GPIO pins: {pins}", flush=True)

    i2c = open_i2c_or_die()

    xs = build_xshut(pins)
    sensors = []
    addrs = []

    try:
        print("🚀 ปลุกทีละตัว + ตั้ง address", flush=True)
        for idx, x in enumerate(xs):
            print(f"→ idx{idx}: XSHUT HIGH", flush=True)
            x.value = True
            s, addr = attach_or_assign(i2c, idx)
            sensors.append(s); addrs.append(addr)

        found = [(i, a) for i, a in enumerate(addrs) if a is not None]
        if not found:
            print("⚠️ ไม่พบเซ็นเซอร์เลย (ตรวจสาย VCC/GND/SDA/SCL/XSHUT และหยุด service อื่นก่อน)")
        else:
            print("📋 พบ:", ", ".join([f"idx{i}@0x{a:02X}" for i, a in found]))

        print("📏 ทดสอบอ่านค่า", flush=True)
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
            print(f"   idx{i}@0x{addrs[i]:02X} → samples={vals}  median={med}")

    finally:
        if not args.keep_address:
            reset_all_to_default(xs)
        else:
            print("⛳ keep-address: คง 0x30..0x33 ตามที่ตั้งไว้ในรอบนี้")

if __name__ == "__main__":
    main()
