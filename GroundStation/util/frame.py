"""
frame_utils.py
----------------
Các hàm phụ trợ cho việc xử lý khung dữ liệu LoRa.
Dùng chung cho cả trạm mặt đất và mô phỏng CubeSat.
"""

from utils.crc16 import crc16

HEADER = 0xAA


def extract_frame(buffer: bytearray):
    """
    Kiểm tra và trích xuất frame đầy đủ từ buffer.
    Trả về frame (bytes) nếu đủ dữ liệu, ngược lại trả None.
    Vì không có trường LENGTH, ta phải xác định theo:
    - CRC luôn là 2 byte cuối
    - LoRa E32 gửi theo từng packet đầy đủ: HEADER + TYPE+SRC+DST+LEN+DATA+CRC
    """
    if len(buffer) < 4:
        return None

    # Header đúng?
    if buffer[0] != HEADER:
        # Sai header → bỏ toàn bộ đến khi gặp 0xAA
        while buffer and buffer[0] != HEADER:
            buffer.pop(0)
        return None

    if len(buffer) >= 4:
        # Giới hạn frame theo LoRa E32
        if len(buffer) > 240:
            frame = buffer[:240]
            del buffer[:240]
            return bytes(frame)

        # Nếu buffer nằm trong [4..240], coi như frame đầy
        return bytes(buffer)

    return None


def parse_frame(frame: bytes):
    """
    Giải mã frame và kiểm tra CRC.
    Trả về dict: {"type", "src", "dst", "len", "data"} nếu CRC hợp lệ.
    Trả về None nếu CRC sai.
    """
    if len(frame) < 4:
        return None

    header = frame[0]
    if header != HEADER:
        return None

    frame_no_crc = frame[:-2]
    crc_recv = int.from_bytes(frame[-2:], 'little')
    crc_calc = crc16(frame_no_crc)

    if crc_recv != crc_calc:
        return None

    packet_type = frame[1]
    payload = frame[2:-2]

    return {
        "header": header,
        "type": packet_type,
        "payload": payload,
        "crc": crc_recv,
    }

