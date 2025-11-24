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
    def __init__(self, settings_path="config/settings.json"):
        """Khởi tạo driver LoRa dựa trên file cấu hình settings.json"""
        with open(settings_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        comm = cfg["communication"]
        self.port = comm["com_port"]
        self.baudrate = comm["baudrate"]
        self.sat_id = comm["satellite_id"]
        self.ground_id = cfg["station"]["ground_id"]
        self.timeout = comm.get("timeout_ms", 1500) / 1000
        
        # If pyserial is not installed or serial is None, continue without hardware
        if serial is None:
            print("Lưu ý: pyserial không được cài hoặc không thể import; chế độ LoRa sẽ không hoạt động.")
            self.ser = None
        else:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                print(f" Đã mở kết nối LoRa tại {self.port} ({self.baudrate} bps)")
            except Exception:
                # Do not raise; allow app to continue in a 'no-hardware' mode
                print(f" Không thể mở cổng {self.port}; chuyển sang chế độ không có phần cứng.")
                self.ser = None

        self.log_dir = Path(cfg["telemetry"]["log_folder"])
        self.log_dir.mkdir(parents=True, exist_ok=True)

    # ----------------------------
    # 1️ Đóng gói dữ liệu
    # ----------------------------
    def build_packet(self, pkg_type, data: bytes):
        """Tạo gói dữ liệu đầy đủ theo chuẩn AA55...CRC16"""
        header = b'\xAA\x55'
        src = self.ground_id
        dst = self.sat_id
        length = len(data)
        frame = header + bytes([pkg_type, src, dst, length]) + data
        crc = crc16(frame).to_bytes(2, 'little')
        return frame + crc
    
    # ----------------------------
    # 2️ Gửi dữ liệu
    # ----------------------------
    def send(self, pkg_type, data: bytes):
        """Gửi gói dữ liệu qua LoRa"""
        packet = self.build_packet(pkg_type, data)
        if not self.ser:
            print("[LoRa] Không có kết nối serial. Bỏ qua gửi gói.")
            log.warn("Không có kết nối serial; bỏ qua gửi gói.")
            return
        try:
            self.ser.write(packet)
            log.cmd(f"Gửi gói type={pkg_type:#02x}, len={len(data)} byte", direction="TX")
            print(f" Gửi gói type={pkg_type:#02x}, len={len(data)} byte")
            self.ser.flush()
        except Exception as e:
            log.error(f"Lỗi khi gửi gói: {e}")
            print(f" Lỗi khi gửi gói: {e}")

    # ----------------------------
    # 3️ Nhận dữ liệu
    # ----------------------------
    def receive(self):
        """Nhận và giải mã gói dữ liệu"""
        start = time.time()
        buffer = bytearray()

        if not self.ser:
            # No serial device; return None immediately
            return None

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
                if parsed:
                    log.cmd(f"Nhận gói type={parsed['type']:#02x}, len={parsed['len']} byte", direction="RX")
                    return parsed
                else:
                    log.warn("CRC sai – bỏ gói", direction="RX")
                    print("⚠️ CRC sai – bỏ gói.")
                    buffer.clear()

        return None

    # ----------------------------
    # 4️ Gửi lệnh điều khiển
    # ----------------------------
    def send_command(self, command: str):
        """Gửi lệnh điều khiển (ví dụ: SCIENCE, SAFE, COMM, CONFIG...)"""
        cmd = command.upper().encode()
        self.send(pkg_type=0x02, data=cmd)
        print(f" Đã gửi lệnh: {command}")
        ack = self.receive()
        if ack and ack["type"] == 0x03:
            print(" Đã nhận ACK từ vệ tinh.")
        else:
            print(" Không nhận được ACK.")

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
            print(" Không phản hồi từ vệ tinh (không có kết nối serial).")
            return False

        self.send(pkg_type=0x02, data=b'PING')
        reply = self.receive()
        if reply and reply["data"] == b'ACK_PING':
            print(" Kết nối vệ tinh OK.")
            return True
        print(" Không phản hồi từ vệ tinh.")
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
