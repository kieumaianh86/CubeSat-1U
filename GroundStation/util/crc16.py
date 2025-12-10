"""
crc16.py
--------------------
Hàm tính toán mã CRC16 cho gói dữ liệu truyền thông giữa CubeSat và Ground Station.
CRC16 được dùng để kiểm tra tính toàn vẹn dữ liệu trong mọi frame LoRa.
"""

# CRC16 mặc định: Polynomial 0xA001 (chuẩn Modbus)
# Có thể thay đổi nếu hệ thống vệ tinh sử dụng variant khác.

CRC16_POLY = 0xA001
CRC16_INIT = 0xFFFF


def crc16(data: bytes, poly: int = CRC16_POLY, init_val: int = CRC16_INIT) -> int:
    """
    Tính CRC16 cho dữ liệu đầu vào (bytes)
    Trả về giá trị CRC 16-bit (int)

    Args:
        data (bytes): Chuỗi byte cần tính CRC
        poly (int): Đa thức CRC (mặc định 0xA001)
        init_val (int): Giá trị khởi tạo (mặc định 0xFFFF)
    Returns:
        int: CRC16 16-bit
    """
    
    crc = init_val
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
            crc &= 0xFFFF  # giữ 16-bit
    return crc


def verify_crc(frame: bytes) -> bool:
    """
    Kiểm tra tính đúng đắn của CRC trong frame hoàn chỉnh
    Frame gồm phần dữ liệu và 2 byte CRC ở cuối (little-endian)

    Args:
        frame (bytes): Frame đầy đủ (AA55...CRC16)
    Returns:
        bool: True nếu CRC đúng, False nếu sai
    """
    
    if len(frame) < 4:
        return False

    crc_recv = int.from_bytes(frame[-2:], "little")
    crc_calc = crc16(frame[:-2])
    return crc_recv == crc_calc