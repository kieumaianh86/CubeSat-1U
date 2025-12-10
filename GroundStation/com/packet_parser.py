import struct
import time
import json
# Ensure the repository's `GroundStation` folder is on sys.path so
# `from util.crc16 import crc16` works even when this module is run
# directly from a different working directory.
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from util.crc16 import crc16

# ---------------------------------------------
# Loại gói theo frame mới
# ---------------------------------------------
PKG_TYPE = {
    0x01: "TELEMETRY",
    0x02: "COMMAND",
    0x03: "ACK",
    0x04: "DATA",
    0x05: "SCIENCE",
    0x06: "END"
}

HEADER = 0xAA   # chỉ còn 1 byte header


# -------------------------------------------------
# 1) Hàm phân tích Frame theo chuẩn mới
# -------------------------------------------------
def parse_packet(frame: bytes):
    """
    Frame mới theo tài liệu:
        [0xAA][TYPE][PAYLOAD...][CRC16]
    Trả về dict: type, payload, crc_ok
    """

    if len(frame) < 4:
        print("Frame quá ngắn.")
        return None

    if frame[0] != HEADER:
        print("Sai header, bỏ frame.")
        return None

    try:
        pkg_type = frame[1]
        payload = frame[2:-2]
        crc_recv = int.from_bytes(frame[-2:], "little")
        crc_calc = crc16(frame[:-2])
        crc_ok = (crc_recv == crc_calc)

        return {
            "type": pkg_type,
            "type_name": PKG_TYPE.get(pkg_type, "UNKNOWN"),
            "payload": payload,
            "crc_ok": crc_ok
        }

    except Exception:
        print("Lỗi parse frame.")
        return None


# -------------------------------------------------
# 2) Decoder Telemetry dạng text như code gốc
# -------------------------------------------------
def decode_telemetry(payload: bytes):
    try:
        text = payload.decode("utf-8", errors="ignore")
        fields = text.strip().split(',')
        telemetry = {}

        for f in fields:
            if ':' in f:
                k, v = f.split(':', 1)
                telemetry[k.strip()] = v.strip()

        telemetry["timestamp"] = time.strftime("%H:%M:%S")
        return telemetry

    except Exception:
        return {"error": "decode_failed"}


# -------------------------------------------------
# 3) Decoder binary DATA
# -------------------------------------------------
def decode_data(payload: bytes):
    preview = payload[:8]
    return {
        "length": len(payload),
        "preview_hex": " ".join(f"{b:02X}" for b in preview)
    }


# -------------------------------------------------
# 4) Decoder ACK
# -------------------------------------------------
def decode_ack(payload: bytes):
    try:
        return {"ack_msg": payload.decode(errors="ignore")}
    except Exception:
        return {"ack_msg": "<invalid>"}


# -------------------------------------------------
# 5) Bộ router xử lý gói (giữ nguyên logic cũ)
# -------------------------------------------------
def handle_packet(frame: bytes):
    """Router phân loại theo type."""
    pkt = parse_packet(frame)
    if pkt is None:
        return None

    if not pkt["crc_ok"]:
        print("CRC sai – bỏ gói.")
        return None

    t = pkt["type"]
    payload = pkt["payload"]

    # ------------------------------
    # TELEMETRY (0x01)
    # ------------------------------
    if t == 0x01:
        decoded = decode_telemetry(payload)
        print(f"TELEMETRY: {decoded}")
        return {"type": "telemetry", "content": decoded}

    # ------------------------------
    # COMMAND ECHO (0x02)
    # ------------------------------
    elif t == 0x02:
        msg = payload.decode(errors='ignore')
        print(f"COMMAND ECHO: {msg}")
        return {"type": "command", "content": msg}

    # ------------------------------
    # ACK (0x03)
    # ------------------------------
    elif t == 0x03:
        decoded = decode_ack(payload)
        print(f"ACK: {decoded['ack_msg']}")
        return {"type": "ack", "content": decoded}

    # ------------------------------
    # DATA (0x04)
    # ------------------------------
    elif t == 0x04:
        decoded = decode_data(payload)
        print(f"DATA: {decoded['length']} bytes ({decoded['preview_hex']} ...)")
        return {"type": "data", "content": decoded}

    # ------------------------------
    # SCIENCE PACKET (0x05)
    # Bạn có thể thêm parser riêng nếu muốn
    # ------------------------------
    elif t == 0x05:
        print(f"SCIENCE PACKET ({len(payload)} bytes)")
        return {"type": "science", "raw": payload}

    # ------------------------------
    # END PACKET (0x06)
    # ------------------------------
    elif t == 0x06:
        print("END OF DATA")
        return {"type": "end"}

    else:
        print(f"Gói loại không xác định: {t}")
        return None
