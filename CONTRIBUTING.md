# 🤝 Contributing to SmartLocker

ขอบคุณที่สนใจร่วมพัฒนาโครงการ **SmartLocker** 🎉
เอกสารนี้จะช่วยให้คุณสามารถตั้งค่าระบบ ทดสอบ และมีส่วนร่วมกับโปรเจกต์ได้อย่างถูกต้องและมีมาตรฐาน

---

## 🔧 เตรียมระบบบน Raspberry Pi
### ✅ 1. อัปเดตระบบและติดตั้งแพ็กเกจที่จำเป็น
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3 python3-venv python3-pip git mosquitto mosquitto-clients nodejs npm
```

### ✅ 2. ติดตั้ง Node-RED (สำหรับทดสอบ MQTT)
```bash
sudo npm install -g --unsafe-perm node-red
node-red
```
จากนั้นเปิด: `http://localhost:1880`

ตั้งค่าให้ Node-RED ทำงานอัตโนมัติ:
```bash
sudo systemctl enable nodered.service
```

---

### ✅ 3. Clone โปรเจกต์
```bash
git clone https://github.com/your-username/SmartLocker.git
cd SmartLocker
```

### ✅ 4. สร้าง Python Virtual Environment และติดตั้ง dependencies
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

---

## 📦 Python Dependencies
รายชื่อไลบรารีที่ใช้เก็บไว้ใน `requirements.txt`:

```txt
adafruit-circuitpython-vl53l0x     # VL53L0X sensor วัดระยะ
adafruit-circuitpython-pca9685     # PWM ควบคุม servo
adafruit-circuitpython-mcp230xx    # I/O Expander
paho-mqtt                          # MQTT client สำหรับ publish/subscribe
adafruit-blinka                    # Layer สำหรับ GPIO บน Pi
RPi.GPIO                           # ควบคุม GPIO โดยตรง
```

ติดตั้งทั้งหมด:
```bash
pip install -r requirements.txt
```

---

## 🧭 เริ่มต้น Contribution
1. Fork repo นี้ไปยัง GitHub ของคุณ
2. Clone ลงเครื่อง:
```bash
git clone https://github.com/your-username/SmartLocker.git
```
3. สร้าง branch ใหม่:
```bash
git checkout -b feature/your-feature-name
```

---

## 🧼 แนวทางการเขียนโค้ด
- เขียนตาม **PEP8**
- ตั้งชื่อตัวแปร/ฟังก์ชันให้สื่อความหมาย
- ใช้ `black` format:
```bash
black .
```

---

## 📂 โครงสร้างโปรเจกต์ (ล่าสุด)
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
├── flow.js                        # Node-RED flow สำหรับทดสอบ MQTT
├── node-red.md                    # คู่มือใช้งาน Node-RED flow
├── readme.md                      # ภาพรวมของโปรเจกต์ (ระบบทั้งหมด)
├── requirements.txt               # รายชื่อไลบรารี Python
└── setup_instructions.md          # (option) รวมขั้นตอนติดตั้ง hardware/software ทั้งหมด
```

---

## ✅ รูปแบบ Commit ที่แนะนำ
ใช้แบบ Conventional:
```bash
feat: เพิ่มฟังก์ชันตรวจสอบสิทธิ์
fix: แก้ logic debounce ประตู
chore: ย้าย init_mcp ไปที่ shared module
```

อย่า commit:
- .venv/
- __pycache__/
- ไฟล์ชั่วคราว `.log`, `.tmp`, `credentials`

---

## 🧪 ทดสอบ MQTT
### วิธีส่งคำสั่งเปิด/ปลดล็อกผ่าน CLI
```bash
mosquitto_pub -t "smartlocker/C01/slot/1/command/open" -m '{"role": "student"}'
```

### หรือใช้ Node-RED:
1. เปิด Node-RED ที่ `http://localhost:1880`
2. Import ไฟล์ `flow.js`
3. ทดสอบเปิดช่อง / ปลดล็อก / ดู status ได้เลย

ดูคู่มือเพิ่มเติม: `node-red.md`

---

## 🙋 หากมีคำถามหรือข้อเสนอแนะ
เปิด issue หรือ PR บน GitHub หรือ inbox มาที่ maintainer ได้เลยครับ

**ขอบคุณที่ช่วยพัฒนา SmartLocker! 💡**
