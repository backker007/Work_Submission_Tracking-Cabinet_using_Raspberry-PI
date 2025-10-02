# TEST_SENSOR/addr_assign_light.py
# ตั้ง address VL53L0X แบบเบาๆ โดยไม่เรียกคาลิเบรตของ adafruit_vl53l0x
# ใช้ XSHUT ปลุกทีละตัว -> รอให้เห็น 0x29 -> เขียนรีจิสเตอร์ 0x8A เป็น 0x30+idx
# รองรับ --keep เพื่อคง address ไว้ / ไม่ keep จะรีเซ็ต XSHUT LOW->HIGH ท้ายสคริปต์
import time, argparse, board, busio, digitalio
from adafruit_bus_device.i2c_device import I2CDevice

ADDRESS_BASE = 0x30     # 0x30..0x33
BOOT_TIMEOUT = 1.2      # รอให้ 0x29 ปรากฏ (วินาที)
DEFAULT_PINS = [17, 27, 22, 5]  # XSHUT GPIO (เพิ่ม 5 ถ้ามีตัวที่สี่)

def D(n): return getattr(board, f"D{n}")

def scan_has(i2c, addr):
    try:
        i2c.try_lock()
        addrs = set(i2c.scan())
    finally:
        try: i2c.unlock()
        except Exception: pass
    return addr in addrs

def write_u8(i2c, addr, reg, val):
    dev = I2CDevice(i2c, addr)
    with dev:
        dev.write(bytes([reg, val & 0x7F]))

def build_xshut(pins):
    xs = [digitalio.DigitalInOut(D(p)) for p in pins]
    for x in xs:
        x.switch_to_output(value=False)  # LOW = ปิดทุกตัว
    time.sleep(0.2)
    return xs

def reset_all(xs):
    # รีเซ็ตกลับ 0x29 ด้วย XSHUT LOW->HIGH
    for x in xs: x.value = False
    time.sleep(0.3)
    for x in xs: x.value = True
    time.sleep(0.5)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pins", default=",".join(map(str, DEFAULT_PINS)),
                    help="เช่น 17,27,22 หรือ 17,27,22,5")
    ap.add_argument("--keep", action="store_true", help="คง address 0x30.. ไว้ตอนจบ")
    args = ap.parse_args()
    pins = [int(s) for s in args.pins.split(",") if s.strip()]

    print(f"🔧 XSHUT pins: {pins}")
    i2c = busio.I2C(board.SCL, board.SDA)

    xs = build_xshut(pins)
    assigned = []

    try:
        for idx, x in enumerate(xs):
            target = ADDRESS_BASE + idx   # 0x30..0x33
            print(f"\n→ idx{idx}: XSHUT HIGH, target 0x{target:02X}")
            x.value = True
            # รอให้ 0x29 ปรากฏ
            t0 = time.time()
            while time.time() - t0 < BOOT_TIMEOUT:
                if scan_has(i2c, 0x29):
                    break
                time.sleep(0.05)
            if not scan_has(i2c, 0x29):
                # ถ้าเคยตั้งไว้แล้ว อาจจะอยู่ที่ target อยู่แล้ว
                if scan_has(i2c, target):
                    print(f"ℹ️  idx{idx}: พบอยู่แล้ว @0x{target:02X}")
                    assigned.append(target)
                    continue
                print("❌ ไม่เห็น 0x29 (ตรวจ XSHUT/VCC/SDA/SCL ของตัวนี้)")
                assigned.append(None)
                continue

            # ตั้งที่อยู่โดยตรง: เขียนรีจิสเตอร์ 0x8A (I2C_SLAVE_DEVICE_ADDRESS)
            try:
                write_u8(i2c, 0x29, 0x8A, target)
                time.sleep(0.02)
                ok = scan_has(i2c, target)
                print(("✅ ตั้งสำเร็จ" if ok else "⚠️ ตั้งแล้วแต่ยังไม่เห็น"),
                      f"0x{target:02X}")
                assigned.append(target if ok else None)
            except Exception as e:
                print(f"❌ set 0x29→0x{target:02X} ล้มเหลว: {e}")
                assigned.append(None)

        print("\n📋 สรุป:", [f"0x{a:02X}" if a else None for a in assigned])

    finally:
        if args.keep:
            print("⛳ keep: คง address ที่ตั้งไว้")
        else:
            print("🔄 reset: คืน XSHUT LOW→HIGH (กลับเป็น 0x29)")
            reset_all(xs)
            print("✅ done")

if __name__ == "__main__":
    main()
