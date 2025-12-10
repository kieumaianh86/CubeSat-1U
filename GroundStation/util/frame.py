"""
frame_utils.py
----------------
Các hàm phụ trợ cho việc xử lý khung dữ liệu LoRa.
Dùng chung cho cả trạm mặt đất và mô phỏng CubeSat.
"""

from utils.crc16 import crc16

HEADER = b'\xAA\x55'


def extract_frame(buffer: bytearray):
    """
    Kiểm tra và trích xuất frame đầy đủ từ buffer.
    Trả về frame (bytes) nếu đủ dữ liệu, ngược lại trả None.
    """
    if len(buffer) < 8 or buffer[:2] != HEADER:
        return None

    length = buffer[5]
    expected = 2 + 1 + 1 + 1 + 1 + length + 2  # HEADER + TYPE+SRC+DST+LEN+DATA+CRC

    if len(buffer) >= expected:
        return bytes(buffer[:expected])

    return None


def parse_frame(frame: bytes):
    """
    Giải mã frame và kiểm tra CRC.
    Trả về dict: {"type", "src", "dst", "len", "data"} nếu CRC hợp lệ.
    Trả về None nếu CRC sai.
    """
    try:
        length = frame[5]
        payload = frame[6:6+length]
        crc_recv = int.from_bytes(frame[-2:], 'little')
        crc_calc = crc16(frame[:-2])

        if crc_recv != crc_calc:
            return None

        return {
            "type": frame[2],
            "src": frame[3],
            "dst": frame[4],
            "len": length,
            "data": payload
        }

    except Exception as e:
        print(f" Lỗi khi phân tích frame: {e}")
        return None