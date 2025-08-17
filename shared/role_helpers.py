# ✅ shared/role_helpers.py
# 🔒 ฟังก์ชันช่วยตรวจสอบสิทธิ์การเข้าถึงตาม role ของผู้ใช้งาน

# ==== Basic Role Checkers ====
def is_admin(role):
    return role == "admin"

def is_professor(role):
    return role == "professor"

def is_student(role):
    return role == "student"

# ==== Permission Logic ====
def can_unlock(role):
    """สามารถปลดล็อกประตู (relay) ได้"""
    return role in ["admin", "professor"]

def can_insert(role):
    """สามารถเปิดช่องใส่เอกสารได้"""
    return role in ["admin", "professor", "student"]

def is_valid_role(role):
    return role in ["admin", "professor", "student"]
