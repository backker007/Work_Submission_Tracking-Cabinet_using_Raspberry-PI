# 📦 SmartLocker Controller

SmartLocker เป็นระบบควบคุมตู้ล็อกเกอร์อัจฉริยะที่ทำงานร่วมกับ Raspberry Pi และอุปกรณ์ IoT ต่าง ๆ เช่น VL53L0X, SG90 Servo, Relay, และ MCP23017 โดยรองรับการสื่อสารผ่าน MQTT และสามารถขยายไปสู่การใช้งานแบบ multi-node ได้

---

## 🚀 ฟีเจอร์หลัก
- 📡 รองรับ MQTT เต็มรูปแบบ (open/unlock/status/warning)
- 🧠 มี state machine สำหรับควบคุมการเปิด-ปิดและตรวจจับเอกสาร
- 🧰 รองรับ sensor VL53L0X และ mc-38 (แม่เหล็กประตู)
- 🔐 ระบบสิทธิ์การเข้าถึง: student / professor / admin
- 🧪 ทดสอบผ่าน Node-RED หรือ mosquitto CLI

---

## 🔧 ระบบที่รองรับ
- Raspberry Pi 3/4/5
- VL53L0X (Time-of-Flight Sensor)
- PCA9685 (PWM Servo Controller)
- MCP23017 (GPIO Expander)
- Relay Module
- SG90 Servo (180 deg)
- mc-38 magnetic door switch

---

## 📂 โครงสร้างโปรเจกต์
```
SmartLocker/
├── controller/
│   ├── main_controller.py         # 🔁 ควบคุม logic หลัก (state machine, sensor loop)
│   └── gpio_setup.py              # (option) setup MCP แยกได้
│
├── shared/
│   ├── utils.py                   # publish_mqtt()
│   ├── mqtt_helpers.py            # ฟังก์ชันช่วยจัด topic และ publish status/warning
│   ├── hardware_helpers.py        # ฟังก์ชัน sensor, debounce, servo, MCP setup
│   └── role_helpers.py            # ตรวจสอบ role (admin, professor, student)
│
├── .gitignore                     # กันไฟล์ .venv, __pycache__ ไม่ให้ push
├── CONTRIBUTING.md                # คู่มือการ contribute
├── node-red.md                    # วิธีใช้ Node-RED จำลองระบบและ MQTT
├── readme.md                      # (ไฟล์นี้)
├── requirements.txt               # Python dependency ทั้งหมด
├── setup_instructions.md          # วิธีติดตั้งทั้งหมดบน Raspberry Pi
└── smartlocker_ui_flow.json       # Node-RED flow พร้อม UI dashboard
```

---

## ▶️ วิธีใช้งานเบื้องต้น (ดูรายละเอียดใน setup_instructions.md)
```bash
# ติดตั้งและรัน
python controller/main_controller.py
```
ระบบจะเริ่มทำงาน รอรับคำสั่งผ่าน MQTT และประมวลผลเซ็นเซอร์

---

## 🧪 ทดสอบด้วย MQTT
สามารถใช้ Node-RED หรือ mosquitto ทดสอบได้:
```bash
mosquitto_pub -t "smartlocker/C01/slot/1/command/open" -m '{"role": "student"}'
```

---

## 📚 เอกสารอื่น ๆ
- `setup_instructions.md` – ติดตั้งระบบบน Raspberry Pi
- `CONTRIBUTING.md` – แนวทางสำหรับผู้พัฒนา
- `node-red.md` – ทดสอบ MQTT ด้วย Node-RED

---

> โครงการนี้ถูกออกแบบเพื่อใช้ในสถานศึกษา วิจัย หรืองานทดลองที่เน้นการเรียนรู้ระบบ IoT
# Work-Submission-Tracking-Cabinet-using-Raspberry-PI.
