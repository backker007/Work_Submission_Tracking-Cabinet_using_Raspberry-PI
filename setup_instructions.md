# ⚙️ SmartLocker Setup Instructions (Raspberry Pi)

เอกสารนี้อธิบายวิธีการติดตั้งและเตรียมระบบ SmartLocker บน Raspberry Pi สำหรับใช้งานจริง รวมถึง hardware และซอฟต์แวร์ที่จำเป็น

---

## 📂 โครงสร้างโปรเจกต์ (Project Structure)
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
├── .gitignore                     # ไฟล์ไม่ควร track (เช่น .venv, __pycache__)
├── CONTRIBUTING.md                # คู่มือการ contribute แบบเต็ม
├── node-red.md                    # คู่มือใช้งาน Node-RED flow
├── readme.md                      # ภาพรวมของโปรเจกต์ (ระบบทั้งหมด)
├── requirements.txt               # รายชื่อไลบรารี Python
├── setup_instructions.md          # ขั้นตอนติดตั้ง hardware/software
└── smartlocker_ui_flow.json       # Node-RED flow พร้อม UI dashboard
```

---

## 🧱 ขั้นตอนพื้นฐาน

### 1. อัปเดตระบบและติดตั้งแพ็กเกจที่จำเป็น
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-pip python3-venv git mosquitto mosquitto-clients nodejs npm i2c-tools
```

### 2. เปิด I2C Interface
```bash
sudo raspi-config
```
- ไปที่ `Interfacing Options`
- เปิดใช้งาน `I2C`

จากนั้นรีบูต:
```bash
sudo reboot
```

---

## 🐍 ติดตั้ง Python Environment

### 3. โคลนโปรเจกต์
```bash
git clone https://github.com/your-username/SmartLocker.git
cd SmartLocker
```

### 4. สร้าง Virtual Environment
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

---

## 🔁 ทดสอบระบบด้วย MQTT (Node-RED หรือ CLI)
### ติดตั้ง Node-RED (ถ้ายังไม่มี)
```bash
sudo npm install -g --unsafe-perm node-red
```
แล้วรัน:
```bash
node-red
```
เปิด browser ที่: [http://localhost:1880](http://localhost:1880)

### หรือใช้คำสั่ง CLI ทดสอบ
```bash
mosquitto_pub -t "smartlocker/C01/slot/1/command/open" -m '{"role": "student"}'
```

---

## ▶️ เริ่มใช้งานจริง
```bash
python controller/main_controller.py
```

คุณสามารถดู log การทำงาน หรือ status จาก sensor ได้ที่ console โดยตรง

---

## 🧠 หมายเหตุ
- หาก sensor ไม่ทำงาน ลองตรวจสอบการเชื่อมสาย I2C และใช้คำสั่ง `i2cdetect -y 1`
- หากใช้หลาย node → แก้ config topic ให้ใช้ `smartlocker/C02/...` ตามต้องการ
- ตรวจสอบว่า servo / relay ต่อถูกช่อง และ power เพียงพอ

---

> พัฒนาเพื่อระบบล็อกเกอร์อัจฉริยะในสถานศึกษา  
> สำหรับผู้ดูแลระบบ, นักพัฒนา, และนักวิจัย
