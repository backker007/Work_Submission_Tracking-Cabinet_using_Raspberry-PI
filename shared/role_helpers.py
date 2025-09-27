# shared/role_helpers.py

# =============================================================================
# 1) Basic Role Checkers
# =============================================================================
def is_admin(role):
    return role == "admin"

def is_teachers(role):
    return role == "teacher"

def is_student(role):
    return role == "student"

# =============================================================================
# 2) Role Validation
# =============================================================================
def is_valid_role(role: str) -> bool:
    return role in ("admin", "teacher", "student")

# =============================================================================
# 3) Permission Checks
# =============================================================================
def can_open_slot(role: str) -> bool:
    return role in ("admin", "teacher", "student")

def can_open_door(role: str) -> bool:
    # ถ้าต้องการให้เฉพาะ admin → เปลี่ยนเป็น return role == "admin"
    return role in ("admin", "teacher")
