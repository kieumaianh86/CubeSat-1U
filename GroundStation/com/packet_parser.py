import struct
from utils.crc16 import crc16
import json
import time

# -------------------------------------------------
# Hằng số định nghĩa loại gói dữ liệu
# -------------------------------------------------
PKG_TYPE = {
    0x01: "TELEMETRY",
    0x02: "COMMAND",
    0x03: "ACK",
    0x04: "DATA",
}

HEADER = b'\xAA\x55'

# -------------------------------------------------
# Hàm phân tích gói dữ liệu
# -------------------------------------------------
def parse_packet(frame: bytes):
    """
    Phân tích gói dữ liệu LoRa nhận được.
    Trả về dict gồm các trường: type, src, dst, len, data, crc_ok.
    """
    if len(frame) < 8:
        return None  # gói quá ngắn

    if not frame.startswith(HEADER):
        print("⚠️ Sai header, bỏ qua.")
        return None

    try:
        pkg_type = frame[2]
        src = frame[3]
        dst = frame[4]
        length = frame[5]
        payload = frame[6:6+length]
        crc_recv = int.from_bytes(frame[6+length:8+length], 'little')
        crc_calc = crc16(frame[:-2])
        crc_ok = (crc_recv == crc_calc)
    except Exception:
        print(" Gói lỗi hoặc độ dài không hợp lệ.")
        return None

    return {
        "type": pkg_type,
        "type_name": PKG_TYPE.get(pkg_type, "UNKNOWN"),
        "src": src,
        "dst": dst,
        "len": length,
        "data": payload,
        "crc_ok": crc_ok
    }

# -------------------------------------------------
# Giải mã dữ liệu telemetry dạng text (payload)
# -------------------------------------------------
def decode_telemetry(payload: bytes):
    """
    Giải mã dữ liệu Telemetry từ vệ tinh.
    Giả sử định dạng chuỗi: b'TEMP:25.3,VOLT:7.4,CURR:0.8,MODE:SCIENCE'
    Trả về dict chứa các giá trị.
    """
    try:
        text = payload.decode('utf-8', errors='ignore')
        fields = text.strip().split(',')
        telemetry = {}
        for f in fields:
            if ':' in f:
                k, v = f.split(':', 1)
                telemetry[k.strip()] = v.strip()
        telemetry['timestamp'] = time.strftime("%H:%M:%S")
        return telemetry
    except Exception:
        return {"error": "decode_failed"}
    
# -------------------------------------------------
# Giải mã gói dữ liệu ảnh hoặc nhị phân (DATA)
# -------------------------------------------------
def decode_data(payload: bytes):
    """
    Dữ liệu nhị phân (ví dụ ảnh, log) – chỉ trả về độ dài và phần đầu.
    """
    preview = payload[:8]
    return {
        "length": len(payload),
        "preview_hex": " ".join([f"{b:02X}" for b in preview]),
    }

# -------------------------------------------------
# Xử lý gói ACK
# -------------------------------------------------
def decode_ack(payload: bytes):
    try:
        msg = payload.decode(errors='ignore')
        return {"ack_msg": msg}
    except Exception:
        return {"ack_msg": "<invalid>"}

# -------------------------------------------------
# Bộ xử lý tổng hợp (router)
# -------------------------------------------------
def handle_packet(frame: bytes):
    """Phân tích gói và xử lý theo loại."""
    pkt = parse_packet(frame)
    if pkt is None:
        return None

    if not pkt["crc_ok"]:
        print("⚠️ CRC sai – bỏ qua gói.")
        return None

    t = pkt["type"]
    data = pkt["data"]

    if t == 0x01:  # Telemetry
        decoded = decode_telemetry(data)
        print(f"📡 TELEMETRY: {decoded}")
        return {"type": "telemetry", "content": decoded}

    elif t == 0x03:  # ACK
        decoded = decode_ack(data)
        print(f"✅ ACK: {decoded['ack_msg']}")
        return {"type": "ack", "content": decoded}
    
    elif t == 0x04:  # Data
        decoded = decode_data(data)
        print(f"📦 DATA: {decoded['length']} bytes ({decoded['preview_hex']} ...)")
        return {"type": "data", "content": decoded}

    elif t == 0x02:  # Command echo
        msg = data.decode(errors='ignore')
        print(f"🛰️ Command echo từ vệ tinh: {msg}")
        return {"type": "command", "content": msg}

    else:
        print(f"⚠️ Gói loại không xác định: {t}")
        return None