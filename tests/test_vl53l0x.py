# tests/test_vl53l0x.py
import csv, time, statistics as stats
from pathlib import Path
import sys

# ให้ import 'shared' ได้แม้รันไฟล์ตรง ๆ
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# พยายามใช้ helper ของโปรเจกต์ก่อน (จะ set XSHUT + เปลี่ยน address ให้)
sensors = None
try:
    from shared.hardware_helpers import init_xshuts, init_sensors, vl53_sensors
    init_xshuts(); init_sensors()
    sensors = vl53_sensors
except Exception as e:
    print("⚠️ use fallback init:", e)
    import board, busio
    from adafruit_bus_device import i2c_device
    import adafruit_vl53l0x
    i2c = busio.I2C(board.SCL, board.SDA)
    # ลองหลาย address (เผื่อ controller เปลี่ยนมาแล้ว)
    candidates = [0x29, 0x30, 0x31, 0x32, 0x33]
    found = []
    for addr in candidates:
        try:
            i2c_device.I2CDevice(i2c, addr)
            found.append(addr)
        except Exception:
            pass
    if not found:
        raise RuntimeError("ไม่พบ VL53L0X ใน 0x29/0x30/0x31/0x32/0x33 — ตรวจ XSHUT/สาย หรือหยุด controller แล้วลองใหม่")
    sensors = [adafruit_vl53l0x.VL53L0X(i2c, address=a) for a in found]

print(f"Found {len(sensors)} VL53L0X sensor(s).")

# ระยะอ้างอิง + รอบทดสอบ
dist_refs = [50, 100, 150, 200, 250]   # mm
trials_per_dist = 10

outdir = Path("results")
outdir.mkdir(exist_ok=True)

for idx, s in enumerate(sensors):
    out_csv = outdir / f"vl53_sensor{idx}.csv"
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ครั้งที่","ค่าที่อ่านได้ (มม.)","ค่าอ้างอิง (มม.)","คลาดเคลื่อน (มม.)","คลาดเคลื่อน (%)"])
        print(f"\n=== Sensor #{idx} ===")
        for ref in dist_refs:
            input(f"➡️  วางแผ่น/เอกสารที่ระยะ {ref} mm แล้วกด Enter เพื่อเริ่มวัด {trials_per_dist} ครั้ง...")
            reads = []
            for t in range(1, trials_per_dist+1):
                val = s.range  # mm
                reads.append(val)
                diff = val - ref
                pct = (abs(diff) / ref) * 100
                w.writerow([t, val, ref, diff, pct])
                print(f"  run {t}: read={val}  diff={diff:+} mm  ({pct:.2f}%)")
                time.sleep(0.2)

            mean = stats.mean(reads)
            sd   = stats.pstdev(reads) if len(reads) > 1 else 0.0
            abs_err = abs(mean - ref)
            rule = ("PASS" if ((ref <= 200 and abs_err <= 5) or (ref == 250 and abs_err <= 8)) and sd <= 3.0
                    else "FAIL")
            w.writerow(["เฉลี่ย", mean, ref, mean-ref, abs((mean-ref)/ref)*100])
            w.writerow(["SD", sd, "", "", ""])
            w.writerow(["ผลประเมิน", rule, "", "", ""])
            print(f"  ▶ mean={mean:.1f}mm  sd={sd:.2f}mm  -> {rule}")

print("\n✅ บันทึกไฟล์ผลลัพธ์ไว้ที่ ./results/vl53_sensor*.csv")
