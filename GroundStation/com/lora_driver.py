import time
import struct
from pathlib import Path
import json

# serial (pyserial) is optional for running the console without hardware.
try:
    import serial
except Exception:
    serial = None

from util.crc16 import crc16
from util.frame import extract_frame, parse_frame
from util.logger import Logger

log = Logger()
class LoRa:
    def __init__(self, port, baudrate=9600, timeout=0.2):
        """Khởi tạo driver LoRa dựa trên file cấu hình settings.json"""
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0
            )
            print(f"[LoRa] Đã mở cổng {port}")
        except Exception as e:
            raise RuntimeError(f"[LoRa] Không mở được cổng {port}: {e}")
        # Định danh trạm mặt đất và vệ tinh
        self.ground_id = 0x01  # ID trạm mặt đất
        self.sat_id = 0x10     # ID vệ tinh

    # ----------------------------
    # 1️ Đóng gói dữ liệu
    # ----------------------------
    def build_packet(self, packet_type: int, payload: bytes):
        """Tạo gói dữ liệu đầy đủ theo chuẩn LoRa:
        [0xAA][TYPE][SRC][DST][LEN][DATA...][CRC16]
        """
        header = bytes([0xAA])
        no_crc = header + bytes([packet_type]) + payload
        crc = crc16(no_crc).to_bytes(2, "little")
        return no_crc + crc
    
    # ----------------------------
    # 2️ Gửi dữ liệu
    # ----------------------------
    def send(self, packet_type: int, payload: bytes = b""):
        """
        Gửi frame theo format:
        [0xAA][TYPE][PAYLOAD...][CRC16]
        """

        packet = self.build_packet(packet_type, payload)

        if not self.ser:
            log.warn("Không có serial — bỏ qua gửi.")
            return

        try:
            self.ser.write(packet)
            self.ser.flush()
            log.cmd(f"Gửi TYPE=0x{packet_type:02X}, {len(payload)} bytes", direction="TX")
            print(f"Gửi TYPE=0x{packet_type:02X}, {len(payload)} bytes")
        except Exception as e:
            log.error(f"Lỗi gửi LoRa: {e}")
            print(f"Lỗi gửi: {e}")

    # ----------------------------
    # 3️ Nhận dữ liệu
    # ----------------------------
    def receive(self):
        """Nhận và giải mã gói theo format mới.
        Vì không có LENGTH → dùng extract_frame() để phân tách frame.
        Trả về dict nếu nhận được gói hợp lệ, ngược lại trả None."""
        if not self.ser:
            return None

        start = time.time()
        buffer = bytearray()

        while time.time() - start < self.timeout:
            try:
                if not self.ser.in_waiting:
                    continue
                buffer.extend(self.ser.read(self.ser.in_waiting))
            except Exception:
                return None

            frame = extract_frame(buffer)
            if frame:
                parsed = parse_frame(frame)
                buffer.clear()

                if parsed:
                    log.cmd(
                        f"Nhận TYPE=0x{parsed['type']:02X}, payload={len(parsed['payload'])} bytes",
                        direction="RX"
                    )
                    print(f"Nhận TYPE=0x{parsed['type']:02X}")
                    return parsed
                else:
                    log.warn("CRC sai — bỏ gói", direction="RX")
                    print("⚠ CRC sai — bỏ gói")

        return None

    # ----------------------------
    # 4️ Gửi lệnh điều khiển
    # ----------------------------
    def send_command(self, text: str):
        """Gửi lệnh điều khiển (ví dụ: SCIENCE, SAFE, COMM, CONFIG...)"""
        payload = text.upper().encode()
        self.send(packet_type=0x02, payload=payload)
        print(f"Đã gửi lệnh: {text}")

        ack = self.receive()
        if ack and ack["type"] == 0x03:
            print("Vệ tinh ACK lệnh.")
        else:
            print("Không nhận được ACK.")

    # ----------------------------
    # 5️Ghi log dữ liệu Telemetry
    # ----------------------------
    def save_telemetry(self, payload: bytes):
        """Ghi dữ liệu Telemetry dạng text/csv"""
        log_file = self.log_dir / time.strftime("%Y%m%d_telemetry.csv")
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(f"{time.strftime('%H:%M:%S')}, {payload.decode(errors='ignore')}\n")

    # ----------------------------
    # 6️ Hàm kiểm tra kết nối
    # ----------------------------
    def check_link(self):
        """Gửi gói ping và chờ phản hồi"""
        # If no serial, assume no link but do not raise
        if not self.ser:
            print("Không có serial — không thể ping.")
            return False

        self.send(0x02, b"PING")
        reply = self.receive()

        if reply and reply["payload"] == b"ACK_PING":
            print("Kết nối OK.")
            return True

        print("Không phản hồi PING.")
        return False
    
    # ----------------------------
    # 7️ Đóng kết nối
    # ----------------------------
    def close(self):
        """Đóng kết nối UART"""
        if self.ser and getattr(self.ser, "is_open", False):
            try:
                self.ser.close()
                print("Đã đóng kết nối LoRa.")
            except Exception:
                pass

