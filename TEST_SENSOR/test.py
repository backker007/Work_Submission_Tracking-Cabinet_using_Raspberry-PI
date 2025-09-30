import time
import board
import busio
import adafruit_vl53l0x
from collections import deque

# สร้าง I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# สร้างอ็อบเจ็กต์เซนเซอร์
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

# ตั้งค่าระยะที่ต้องการ
MIN_DISTANCE = 0
MAX_DISTANCE = 200
BUFFER_SIZE = 5  # เก็บข้อมูล 5 ค่า

# สร้าง buffer สำหรับเก็บค่า
distance_buffer = deque(maxlen=BUFFER_SIZE)

def calculate_average(buffer):
    return sum(buffer) / len(buffer)

if __name__ == "__main__":
    try:
        while True:
            raw_distance = vl53.range
            # เก็บค่าระยะใน buffer
            distance_buffer.append(raw_distance)
            
            # คำนวณค่าเฉลี่ยจาก buffer
            if len(distance_buffer) == BUFFER_SIZE:
                average_distance = calculate_average(distance_buffer)
                
                # ตรวจสอบค่าระยะให้อยู่ในช่วง 0-200 มม.
                if average_distance < MIN_DISTANCE:
                    adjusted_distance = MIN_DISTANCE
                elif average_distance > MAX_DISTANCE:
                    adjusted_distance = MAX_DISTANCE
                else:
                    adjusted_distance = average_distance
                
                print(f"Adjusted Average Distance: {adjusted_distance:.1f} mm")
            else:
                print("Waiting for enough data points...")
            
            time.sleep(0.1)  # ให้เวลาเซ็นเซอร์พักก่อนการอ่านครั้งถัดไป

    except KeyboardInterrupt:
        print("Exit program.")
        # อย่าใช้ deinit() ก่อนการใช้ I2C เสร็จสิ้น
        i2c.deinit()  # รีเซ็ต I2C ก่อนออกจากโปรแกรม
