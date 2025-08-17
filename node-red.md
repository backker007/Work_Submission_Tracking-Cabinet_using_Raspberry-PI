# 🧪 Node-RED Testing Guide

เอกสารนี้ใช้สำหรับทดสอบระบบ **SmartLocker** โดยไม่ต้องเชื่อมต่อกับ frontend จริง โดยใช้เครื่องมือ Node-RED ผ่าน MQTT
เหมาะสำหรับการพัฒนาเริ่มต้น หรือการ debug อุปกรณ์บน Raspberry Pi

---

## 🚀 คุณสามารถทำอะไรได้บ้างด้วย Node-RED:
- ✅ แสดงผล status / warning ของ slot ผ่าน MQTT
- ✅ ส่งคำสั่งทดสอบเปิดช่อง / ปลดล็อก ผ่าน MQTT inject
- ✅ จำลองการทำงานของ web app ได้โดยไม่ต้องมี UI จริง
- ✅ ใช้ UI Dashboard เพื่อดูสถานะและควบคุมได้ผ่านหน้าเว็บ

---

## 🔧 การติดตั้ง Node-RED
ตรวจสอบให้แน่ใจว่ามี Node.js และ npm แล้ว จากนั้นติดตั้ง Node-RED:
```bash
sudo npm install -g --unsafe-perm node-red
```
เริ่มใช้งาน:
```bash
node-red
```
แล้วเปิดเบราว์เซอร์: `http://localhost:1880`

---

## 📥 การ Import Flow ทดสอบ
1. เปิด Node-RED ในเบราว์เซอร์
2. เมนูมุมขวาบน → **Import** → **Clipboard**
3. วางเนื้อหาจาก `smartlocker_ui_flow.json`
4. กด **Deploy**

---

## 🧪 รายละเอียด Flow
### MQTT In:
- `smartlocker/+/slot/+/status` → แสดงค่าความจุและสถานะบน debug หรือ UI
- `smartlocker/+/slot/+/warning` → แสดงแจ้งเตือนใน debug หรือ popup

### MQTT Out (จำลอง Command):
- 🔘 ปุ่ม Inject: ส่งคำสั่งเปิดช่อง 1 (role: `student`)
- 🔘 ปุ่ม Inject: ส่งคำสั่งปลดล็อกช่อง 1 (role: `admin`)

### MQTT Broker:
- ใช้ `localhost:1883` เป็นค่าเริ่มต้น (Mosquitto local)
- หากใช้ broker ที่อื่น สามารถเปลี่ยนได้ใน MQTT Config Node

---

## 🧑‍💻 เพิ่ม UI Dashboard (Optional)
หากต้องการดูสถานะผ่าน UI:

### ติดตั้ง Node-RED Dashboard
```bash
cd ~/.node-red
npm install node-red-dashboard
```
จากนั้น restart Node-RED:
```bash
node-red
```

### ใช้งาน Dashboard
- ลาก nodes ในหมวด `dashboard` มาวางเช่น:
  - `ui_button` → เปิดช่อง / ปลดล็อก
  - `ui_text` / `ui_gauge` → แสดงค่าความจุ
  - `ui_toast` → แจ้งเตือน warning
- เพิ่มหน้าเว็บ UI ที่: `http://localhost:1880/ui`

---

## 📌 หมายเหตุเพิ่มเติม
- สามารถใช้ CLI สำหรับส่ง MQTT แบบ custom ได้เช่น:
```bash
mosquitto_pub -t "smartlocker/C01/slot/1/command/open" -m '{"role": "student"}'
```

---

## 📂 Topics ที่ใช้บ่อย
```
smartlocker/C01/slot/1/command/open
smartlocker/C01/slot/1/command/unlock
smartlocker/C01/slot/1/status
smartlocker/C01/slot/1/warning
```

---

## 📚 เอกสารเพิ่มเติม
- การติดตั้งระบบ: `setup_instructions.md`
- คู่มือ contribute: `CONTRIBUTING.md`
- คู่มือ Node-RED: `node-red.md`

---

Node-RED คือเครื่องมือทดสอบระบบที่ทรงพลังและรวดเร็ว ⚡  
เหมาะกับทีมพัฒนาและผู้ดูแลระบบที่ต้องการตรวจสอบฟังก์ชันแบบไม่ต้องรอ UI จริง
