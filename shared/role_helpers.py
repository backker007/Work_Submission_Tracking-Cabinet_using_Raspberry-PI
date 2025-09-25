# ✅ shared/role_helpers.py
# 🔒 ฟังก์ชันช่วยตรวจสอบสิทธิ์การเข้าถึงตาม role ของผู้ใช้งาน

# ==== Basic Role Checkers ====
def is_admin(role):
    return role == "admin"

def is_teachers(role):
    return role == "teacher"

def is_student(role):
    return role == "student"

def is_valid_role(role: str) -> bool:
    return role in ("admin", "teacher", "student")

def can_intake(role: str) -> bool:
    return role in ("admin", "teacher", "student")

def can_removal(role: str) -> bool:
    # ถ้าต้องการให้เฉพาะ admin → เปลี่ยนเป็น return role == "admin"
    return role in ("admin", "teacher")
