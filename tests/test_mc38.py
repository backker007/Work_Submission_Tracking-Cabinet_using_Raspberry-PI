# tests/test_mc38.py
# ทดสอบสวิตช์แม่เหล็ก MC-38 (Door Sensor) ช่องที่ 1 เท่านั้น + Debounce (time-stability)
# ผลลัพธ์: results/mc38_slot1_results.csv

import csv
import time
from pathlib import Path
import sys

# ให้ import โปรเจกต์ได้แม้รันไฟล์ตรง ๆ
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# ================== CONFIG ==================
MCP23017_ADDR   = 0x20          # I2C ของ MCP23017
DOOR_PINS_GPA   = [0, 1, 2, 3]  # GPA0..GPA3 = เซ็นเซอร์ประตู (active-low)
SLOT_ID         = 1             # ✅ ทดสอบ "ช่องที่ 1" เท่านั้น
SLOT_INDEX      = 0             # index ใน GPA (ช่อง1 -> GPA0 -> index 0)
RUNS_PER_STATE  = 10            # จำนวนรอบต่อหนึ่งกรณี (ปรับได้)
ACTIVE_LOW      = True          # MC-38 + Pull-up: ปิด = LOW

# Debounce (time-stability)
STABLE_MS       = 80            # ต้องคงที่อย่างน้อยกี่มิลลิวินาที
SAMPLE_EVERY_MS = 5             # อ่านซ้ำทุกกี่มิลลิวินาที
TIMEOUT_MS      = 2000          # กันหลุด loop สูงสุด (ms)
# ============================================

import board, busio
from adafruit_mcp230xx.mcp23017 import MCP23017
from digitalio import Direction, Pull

# ---------- Setup I2C + MCP23017 ----------
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP23017(i2c, address=MCP23017_ADDR)

door_pins = [mcp.get_pin(p) for p in DOOR_PINS_GPA]
for p in door_pins:
    p.direction = Direction.INPUT
    p.pull      = Pull.UP  # HIGH เมื่อ "เปิด", LOW เมื่อ "ปิด"

# ---------- Helpers ----------
def read_door_raw(idx: int, *, active_low: bool = ACTIVE_LOW) -> bool:
    """คืนค่า True=ปิด, False=เปิด (ตีความ active-low ให้เรียบร้อย)"""
    val = door_pins[idx].value  # True=HIGH, False=LOW
    return (not val) if active_low else val

def is_door_reliably_closed(
    idx: int,
    stable_ms: int = STABLE_MS,
    sample_every_ms: int = SAMPLE_EVERY_MS,
    active_low: bool = ACTIVE_LOW,
    timeout_ms: int = TIMEOUT_MS
) -> tuple[bool, int]:
    """Debounce แบบ time-stability -> (closed, latency_ms)"""
    t0 = time.perf_counter()
    last = read_door_raw(idx, active_low=active_low)
    stable_start = None

    while True:
        cur = read_door_raw(idx, active_low=active_low)
        now = time.perf_counter()

        if cur == last:
            if stable_start is None:
                stable_start = now
            elif (now - stable_start) * 1000 >= stable_ms:
                return cur, int((now - t0) * 1000)
        else:
            last = cur
            stable_start = None

        if (now - t0) * 1000 > timeout_ms:
            return cur, int((now - t0) * 1000)

        time.sleep(sample_every_ms / 1000.0)

# ---------- Test Runner (เฉพาะช่อง 1) ----------
def main():
    outdir = Path("results"); outdir.mkdir(exist_ok=True)
    out_csv = outdir / "mc38_slot1_results.csv"

    states = [
        ("ปิดสนิท",   True),   # ให้ประกบกันแน่น
        ("เปิด",      False),  # แยกแม่เหล็กออก
        ("ปิดไม่สนิท", False)  # เผื่อทดสอบกรณีใกล้/สั่น (ส่วนใหญ่จะเหมือน "เปิด")
    ]

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ครั้งที่","ช่อง","กรณีทดสอบ","ค่าอ่าน(0=ปิด,1=เปิด)","ตีความ","เวลาเสถียร(ms)"])

        print("เริ่มทดสอบ MC-38 เฉพาะช่องที่ 1")
        print(f"- stable_ms={STABLE_MS}, sample_every_ms={SAMPLE_EVERY_MS}, runs/กรณี={RUNS_PER_STATE}\n")

        for name, _expect in states:
            print(f"\n== ช่อง {SLOT_ID}: {name} ==")
            for t in range(1, RUNS_PER_STATE + 1):
                input(f"จัดสภาพ '{name}' ให้เรียบร้อย แล้วกด Enter เพื่อเริ่ม… ")
                closed, ms = is_door_reliably_closed(SLOT_INDEX)
                value01 = 0 if closed else 1
                interp  = "Closed" if closed else "Open"
                w.writerow([t, SLOT_ID, name, value01, interp, ms])
                print(f"  run {t}: {interp} ({value01}) ใน {ms} ms")

    print(f"\n✅ บันทึกผลที่: {out_csv}")

if __name__ == "__main__":
    main()


# ตัวอย่าง: ทดสอบช่อง 1, วัด 20 รอบ/กรณี, ต้องนิ่ง 100ms
#python -m tests.test_mc38 --slot 1 --runs 20 --stable-ms 100
