#!/usr/bin/env python3
# test_mc38_multi.py
import time, argparse, os, board, busio
from digitalio import Direction, Pull
from adafruit_mcp230xx.mcp23017 import MCP23017

def label(b): return "CLOSED" if b else "OPEN"

def read_closed_raw(pin, invert=False):
    # Pull.UP: LOW (False) = วงจรปิด = CLOSED
    closed = (pin.value is False)
    return (not closed) if invert else closed

def main():
    ap = argparse.ArgumentParser("ทดสอบ MC-38 หลายช่อง (8,9,10,11) พร้อมพิมพ์สถานะต่อเนื่องได้")
    ap.add_argument("--pins", type=str, default="8,9,10,11", help="รายการขา MCP23017 คั่นด้วยจุลภาค (เช่น 8,9,10,11)")
    ap.add_argument("--invert", action="store_true",
                    default=os.getenv("DOOR_SENSOR_INVERT","0").lower() in ("1","true","yes"),
                    help="กลับขั้วความหมาย (ถ้าอ่านกลับด้าน)")
    ap.add_argument("--open-hold", type=float, default=float(os.getenv("DOOR_DEBOUNCE_OPEN_S", 0.5)),
                    help="เวลาที่ต้องอ่านว่า 'เปิด' ติดกันก่อนยืนยัน (วินาที)")
    ap.add_argument("--close-hold", type=float, default=float(os.getenv("DOOR_DEBOUNCE_CLOSE_S", 0.6)),
                    help="เวลาที่ต้องอ่านว่า 'ปิด' ติดกันก่อนยืนยัน (วินาที)")
    ap.add_argument("--interval", type=float, default=0.02, help="รอบอ่าน (วินาที)")
    ap.add_argument("--log-every", type=float, default=0.0,
                    help="พิมพ์สรุปสถานะทุก ๆ N วินาที (0 = เงียบจนกว่าจะมีการเปลี่ยน)")
    ap.add_argument("--raw", action="store_true", help="พิมพ์ค่า raw (HIGH/LOW) ตอน log-every ด้วย")
    args = ap.parse_args()

    pins_num = [int(x.strip()) for x in args.pins.split(",") if x.strip()]
    print(f"Watching pins: {pins_num} invert: {args.invert}")

    i2c = busio.I2C(board.SCL, board.SDA)
    mcp = MCP23017(i2c)
    pins = []
    for n in pins_num:
        p = mcp.get_pin(n)
        p.direction = Direction.INPUT
        p.pull = Pull.UP
        pins.append(p)

    # สถานะเสถียรและตัวจับเวลา per pin
    stable = [read_closed_raw(p, args.invert) for p in pins]
    cand   = stable[:]
    since  = [time.time()] * len(pins)

    print("Initial:", [label(s) for s in stable])
    last_dump = time.time()

    try:
        while True:
            now = time.time()
            for i, p in enumerate(pins):
                raw_closed = read_closed_raw(p, args.invert)

                if raw_closed != cand[i]:
                    cand[i] = raw_closed
                    since[i] = now
                else:
                    hold = args.close_hold if cand[i] else args.open_hold
                    if (now - since[i] >= hold) and (cand[i] != stable[i]):
                        stable[i] = cand[i]
                        print("%s  pin %d -> %s (raw=%s, held=%.2fs)" %
                              (time.strftime("%H:%M:%S"), pins_num[i], label(stable[i]),
                               "LOW" if p.value is False else "HIGH", now - since[i]))

            # พิมพ์สรุปเป็นระยะแม้ไม่มีการเปลี่ยน
            if args.log_every > 0 and (now - last_dump) >= args.log_every:
                if args.raw:
                    raws = [("LOW" if p.value is False else "HIGH") for p in pins]
                    print("%s  summary: %s  raw=%s" %
                          (time.strftime("%H:%M:%S"), [label(s) for s in stable], raws))
                else:
                    print("%s  summary: %s" %
                          (time.strftime("%H:%M:%S"), [label(s) for s in stable]))
                last_dump = now

            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
