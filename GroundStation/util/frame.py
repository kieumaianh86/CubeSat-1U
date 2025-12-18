"""
frame_utils.py
----------------
Các hàm phụ trợ cho việc xử lý khung dữ liệu LoRa.
Dùng chung cho cả trạm mặt đất và mô phỏng CubeSat.
"""

from utils.crc16 import crc16
from typing import Optional

HEADER = 0xAA
MAX_FRAME= 240  # Giới hạn kích thước frame tối đa
MIN_FRAME = 4    # Kích thước frame tối thiểu (HEADER + TYPE + CRC16)


def extract_frame(buffer: bytearray) -> bytes | None:
    """
    Kiểm tra và trích xuất frame đầy đủ từ buffer.
    Trả về frame (bytes) nếu đủ dữ liệu, ngược lại trả None.
    Vì không có trường LENGTH, ta phải xác định theo:
    - CRC luôn là 2 byte cuối
    - LoRa E32 gửi theo từng packet đầy đủ: HEADER + TYPE+SRC+DST+LEN+DATA+CRC
    """
    # Dọn dữ liệu sai đầu buffer
    while buffer and buffer[0] != HEADER:
        buffer.pop(0)

    # Không đủ để tạo frame
    if len(buffer) < MIN_FRAME:
        return None

    # Frame LoRa không có length, dùng toàn bộ buffer làm frame
    # hoặc cắt theo max để tránh overflow.
    if len(buffer) > MAX_FRAME:
        frame = bytes(buffer[:MAX_FRAME])
        del buffer[:MAX_FRAME]
        return frame

    # Buffer nhỏ, assume frame đầy đủ
    frame = bytes(buffer)
    buffer.clear()
    return frame


def parse_frame(frame: bytes) -> dict | None:
    """
    Trả về dict:
    {
        "header": 0xAA,
        "type": int,
        "payload": bytes,
        "crc": int
    }
    """

    if len(frame) < MIN_FRAME:
        return None

    if frame[0] != HEADER:
        return None

    try:
        pkt_type = frame[1]
        payload = frame[2:-2]

        crc_recv = int.from_bytes(frame[-2:], "little")
        crc_calc = crc16(frame[:-2])

        if crc_recv != crc_calc:
            return None

        return {
            "header": HEADER,
            "type": pkt_type,
            "payload": payload,
            "crc": crc_recv
        }

    except Exception:
        return None
