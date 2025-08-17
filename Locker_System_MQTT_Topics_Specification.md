# 📦 Locker System MQTT Topics

This document describes the MQTT topic structure used in the **Locker System Project**.  
It defines how nodes (lockers) communicate status, warnings, and receive commands.

---

## 🔑 Topic Structure
```
{node_id}/slot/{slot_id}/{action}
```

- `{node_id}` → Locker Node ID (e.g., `C01`, `C02`)
- `{slot_id}` → Slot number in the locker (e.g., `1`, `2`, `3`)
- `{action}` → Type of communication (`status`, `warning`, `command/...`)

---

## 🌳 Topic Tree Diagram

```
{node_id}
 └── slot
      ├── {slot_id}
      │    ├── status      # Report current state
      │    ├── warning     # Notify abnormal events
      │    └── command
      │         ├── open   # Open slot for document submission
      │         ├── close  # Close slot (optional)
      │         └── relay  # Unlock main door (professor/admin only)
      │
      └── {slot_id+1} ...
```

---

## 📌 Topics

### 1. Status Topics
```
{node_id}/slot/{slot_id}/status
```

**Example Payload:**
```json
{
  "capacity_mm": 238.5,
  "available": true,
  "door_closed": true,
  "slot": 1,
  "node": "C01"
}
```

### 2. Warning Topics
```
{node_id}/slot/{slot_id}/warning
```

**Example Payload:**
```json
{
  "slot": 1,
  "node": "C01",
  "warning": "door_not_closed"
}
```

### 3. Command Topics
```
{node_id}/slot/{slot_id}/command/open
{node_id}/slot/{slot_id}/command/close
{node_id}/slot/{slot_id}/command/relay
```

---

## ✅ Example Usage
- **Publish status every 5s**  
  `C01/slot/1/status`
- **Door not closed warning**  
  `C01/slot/1/warning`
- **Send open command**  
  `C01/slot/1/command/open`

---

## 📖 Summary
The Locker System uses **3 main groups of topics**:
1. `status` → Report current status  
2. `warning` → Notify abnormal events  
3. `command` → Receive commands  
