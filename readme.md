# Work Submission Tracking Cabinet using Raspberry Pi

## 📌 Overview
โปรเจกต์นี้เป็นระบบ **Smart Locker** สำหรับการติดตามการส่งงาน (Work Submission Tracking Cabinet) โดยใช้ **Raspberry Pi** ควบคุมการทำงานของเซ็นเซอร์, เซอร์โว และรีเลย์ พร้อมทั้งสื่อสารข้อมูลผ่าน **MQTT** และ **Node-RED Dashboard** เพื่อให้สามารถตรวจสอบสถานะและควบคุมระยะไกลได้

## ⚙️ Features
- ตรวจจับสถานะช่องเก็บงาน (เปิด/ปิด, ความจุ, ประตูสนิทหรือไม่)
- รองรับผู้ใช้งานหลายระดับ (Student / Professor / Admin)  
- สื่อสารด้วย **MQTT protocol** (publish/subscribe)  
- มี **Node-RED flow** สำหรับแสดงผลและควบคุมผ่าน UI  
- รองรับ Multi-node ผ่าน `NODE_ID`  

## 📂 Project Structure
```
controller/
 ┗ main_controller.py          # สคริปต์หลักสำหรับควบคุม Smart Locker

shared/                        # โมดูลช่วยเหลือ (Helper modules)
 ┣ hardware_helpers.py         # ฟังก์ชันควบคุมเซ็นเซอร์, เซอร์โว, รีเลย์
 ┣ mqtt_helpers.py             # ฟังก์ชัน publish/subscribe MQTT
 ┣ role_helpers.py             # ตรวจสอบสิทธิ์ผู้ใช้งาน (Student/Professor/Admin)
 ┗ utils.py                    # Utility functions ทั่วไป

Locker_System_MQTT_Topics_Specification.md   # สเปก topic MQTT
node-red.md                    # คำอธิบายการใช้งาน Node-RED
smartlocker_ui_flow.json       # Flow สำหรับ Node-RED Dashboard
setup_instructions.md          # วิธีติดตั้งและเซ็ตอัพระบบ
requirements.txt               # Python dependencies
CONTRIBUTING.md                # แนวทางการ contribute โค้ด
.gitignore
readme.md
```

## 🛠️ Installation
1. Clone repository:
   ```bash
   git clone https://github.com/backker007/Work_Submission_Tracking-Cabinet_using_Raspberry-PI.git
   cd Work_Submission_Tracking-Cabinet_using_Raspberry-PI
   ```
2. ติดตั้ง dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. แก้ไขการตั้งค่า MQTT broker และ `NODE_ID` ในไฟล์ `shared/utils.py` หรือ config ที่เกี่ยวข้อง
4. รัน controller:
   ```bash
   python controller/main_controller.py
   ```

## 📡 Node-RED Setup
1. เปิด Node-RED บน Raspberry Pi หรือ Server
2. Import flow จาก `smartlocker_ui_flow.json`
3. ตรวจสอบการเชื่อมต่อ MQTT broker
4. เปิด Dashboard เพื่อควบคุมและดูสถานะ locker

## 👥 User Roles
- **Student** → เปิดช่องใส่เอกสารได้ แต่ไม่สามารถเปิดประตูหลัก  
- **Professor & Admin** → เปิดได้ทั้งช่องใส่เอกสารและประตูเก็บเอกสาร  

## 📑 Documentation
- [MQTT Topics Specification](./Locker_System_MQTT_Topics_Specification.md)  
- [Setup Instructions](./setup_instructions.md)  
- [Node-RED Guide](./node-red.md)  


> โครงการนี้ถูกออกแบบเพื่อใช้ในสถานศึกษา วิจัย หรืองานทดลองที่เน้นการเรียนรู้ระบบ IoT
# Work-Submission-Tracking-Cabinet-using-Raspberry-PI.