#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED Debug Script - ทดสอบ LED ทีละดวง
ใช้เพื่อตรวจสอบว่า LED ต่อถูกหรือไม่
"""

import time
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017
from digitalio import Direction

# เชื่อมต่อ MCP23017
i2c = busio.I2C(board.SCL, board.SDA)
mcp = MCP23017(i2c)

# กำหนดพิน LED (ตาม .env ของคุณ)
LED_READY_PINS = [0, 2, 4, 6]  # เขียว
LED_ERROR_PINS = [1, 3, 5, 7]  # แดง
ACTIVE_HIGH = True  # True=ส่ง HIGH เพื่อติด, False=ส่ง LOW เพื่อติด

def setup_led(pin_num):
    """ตั้งค่าพินเป็น OUTPUT"""
    pin = mcp.get_pin(pin_num)
    pin.direction = Direction.OUTPUT
    return pin

def led_on(pin):
    """เปิดไฟ"""
    pin.value = True if ACTIVE_HIGH else False

def led_off(pin):
    """ปิดไฟ"""
    pin.value = False if ACTIVE_HIGH else True

def test_all_leds():
    """ทดสอบ LED ทั้งหมดทีละดวง"""
    print("=" * 60)
    print("LED Debug Test - กำลังทดสอบ LED ทีละดวง")
    print("=" * 60)
    
    # Setup ทุกพิน
    ready_pins = [setup_led(p) for p in LED_READY_PINS]
    error_pins = [setup_led(p) for p in LED_ERROR_PINS]
    
    # ปิดทุกดวงก่อน
    print("\n1️⃣ ปิดไฟทั้งหมด...")
    for pin in ready_pins + error_pins:
        led_off(pin)
    time.sleep(1)
    
    # ทดสอบไฟเขียว (Ready)
    print("\n2️⃣ ทดสอบไฟเขียว (Ready) ทีละช่อง...")
    for i, pin in enumerate(ready_pins, start=1):
        print(f"   → กำลังเปิดไฟเขียวช่อง {i} (pin {LED_READY_PINS[i-1]})...")
        led_on(pin)
        time.sleep(1.5)
        led_off(pin)
        time.sleep(0.5)
    
    # ทดสอบไฟแดง (Error)
    print("\n3️⃣ ทดสอบไฟแดง (Error) ทีละช่อง...")
    for i, pin in enumerate(error_pins, start=1):
        print(f"   → กำลังเปิดไฟแดงช่อง {i} (pin {LED_ERROR_PINS[i-1]})...")
        led_on(pin)
        time.sleep(1.5)
        led_off(pin)
        time.sleep(0.5)
    
    # เปิดทั้งหมดพร้อมกัน
    print("\n4️⃣ เปิดไฟเขียวทั้งหมด 2 วินาที...")
    for pin in ready_pins:
        led_on(pin)
    time.sleep(2)
    for pin in ready_pins:
        led_off(pin)
    
    print("\n5️⃣ เปิดไฟแดงทั้งหมด 2 วินาที...")
    for pin in error_pins:
        led_on(pin)
    time.sleep(2)
    for pin in error_pins:
        led_off(pin)
    
    # กระพริบสลับ
    print("\n6️⃣ กระพริบสลับเขียว-แดง 5 รอบ...")
    for _ in range(5):
        # เขียว
        for pin in ready_pins:
            led_on(pin)
        time.sleep(0.3)
        for pin in ready_pins:
            led_off(pin)
        
        # แดง
        for pin in error_pins:
            led_on(pin)
        time.sleep(0.3)
        for pin in error_pins:
            led_off(pin)
    
    print("\n✅ ทดสอบเสร็จสิ้น!")
    print("\n📊 ผลการทดสอบ:")
    print("=" * 60)
    print("ถ้าไฟติดตามลำดับ → LED ทำงานปกติ")
    print("ถ้าไฟไม่ติด/ติดผิดตำแหน่ง → ตรวจสอบ:")
    print("  1. สาย LED ต่อถูกพินหรือไม่")
    print("  2. SLOT_LED_ACTIVE_HIGH ใน .env ถูกต้องหรือไม่")
    print("  3. LED มี resistor หรือไม่ (แนะนำ 220-330Ω)")
    print("  4. แรงดันไฟเลี้ยง LED (ควรเป็น 3.3V)")
    print("=" * 60)

def test_specific_pin(pin_num, duration=2):
    """ทดสอบพินเฉพาะ"""
    print(f"\n🔍 ทดสอบพิน {pin_num} เป็นเวลา {duration} วินาที...")
    pin = setup_led(pin_num)
    led_on(pin)
    time.sleep(duration)
    led_off(pin)
    print(f"✅ ปิดพิน {pin_num} แล้ว")

def interactive_test():
    """โหมดทดสอบแบบ interactive"""
    print("\n" + "=" * 60)
    print("LED Interactive Test")
    print("=" * 60)
    
    ready_pins = [setup_led(p) for p in LED_READY_PINS]
    error_pins = [setup_led(p) for p in LED_ERROR_PINS]
    all_pins = ready_pins + error_pins
    all_pin_nums = LED_READY_PINS + LED_ERROR_PINS
    
    # ปิดทั้งหมดก่อน
    for pin in all_pins:
        led_off(pin)
    
    print("\nคำสั่ง:")
    print("  r1-r4 : เปิดไฟเขียวช่อง 1-4")
    print("  e1-e4 : เปิดไฟแดงช่อง 1-4")
    print("  off   : ปิดทั้งหมด")
    print("  on    : เปิดทั้งหมด")
    print("  quit  : ออกจากโปรแกรม")
    print()
    
    while True:
        cmd = input("คำสั่ง > ").strip().lower()
        
        if cmd == "quit":
            for pin in all_pins:
                led_off(pin)
            print("ปิดไฟทั้งหมดแล้ว. ออกจากโปรแกรม.")
            break
        
        elif cmd == "off":
            for pin in all_pins:
                led_off(pin)
            print("✅ ปิดไฟทั้งหมด")
        
        elif cmd == "on":
            for pin in all_pins:
                led_on(pin)
            print("✅ เปิดไฟทั้งหมด")
        
        elif cmd.startswith("r") and len(cmd) == 2 and cmd[1].isdigit():
            idx = int(cmd[1]) - 1
            if 0 <= idx < len(ready_pins):
                led_on(ready_pins[idx])
                print(f"✅ เปิดไฟเขียวช่อง {idx + 1} (pin {LED_READY_PINS[idx]})")
            else:
                print("❌ ช่องไม่ถูกต้อง")
        
        elif cmd.startswith("e") and len(cmd) == 2 and cmd[1].isdigit():
            idx = int(cmd[1]) - 1
            if 0 <= idx < len(error_pins):
                led_on(error_pins[idx])
                print(f"✅ เปิดไฟแดงช่อง {idx + 1} (pin {LED_ERROR_PINS[idx]})")
            else:
                print("❌ ช่องไม่ถูกต้อง")
        
        else:
            print("❌ คำสั่งไม่ถูกต้อง")

if __name__ == "__main__":
    print("""
╔════════════════════════════════════════════════════════════╗
║           Smart Locker LED Debug Tool                      ║
║                                                            ║
║  ใช้เพื่อทดสอบ LED บน MCP23017                            ║
║  ตรวจสอบว่า LED ต่อถูกต้องและทำงานปกติ                  ║
╚════════════════════════════════════════════════════════════╝
    """)
    
    print("\nโหมดการทดสอบ:")
    print("1. ทดสอบอัตโนมัติ (แนะนำ)")
    print("2. ทดสอบแบบ Interactive")
    print("3. ทดสอบพินเฉพาะ")
    
    try:
        choice = input("\nเลือก (1/2/3): ").strip()
        
        if choice == "1":
            test_all_leds()
        elif choice == "2":
            interactive_test()
        elif choice == "3":
            pin = int(input("ระบุหมายเลขพิน (0-15): "))
            if 0 <= pin <= 15:
                test_specific_pin(pin)
            else:
                print("❌ พินไม่ถูกต้อง (ต้องเป็น 0-15)")
        else:
            print("❌ ตัวเลือกไม่ถูกต้อง")
    
    except KeyboardInterrupt:
        print("\n\n⚠️ ยกเลิกโดยผู้ใช้")
    except Exception as e:
        print(f"\n❌ เกิดข้อผิดพลาด: {e}")
        import traceback
        traceback.print_exc()