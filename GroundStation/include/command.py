"""
command.py
------------------
Quản lý các lệnh điều khiển gửi từ trạm mặt đất (Ground Station)
tới vệ tinh CubeSat qua module LoRa.
"""

import time
from com.lora_driver import LoRa
from util.logger import Logger
log = Logger()

# ----------------------------
#  Danh sách lệnh hợp lệ
# ----------------------------
VALID_COMMANDS = {
    "SCIENCE":  b"SCIENCE",   # Bắt đầu thu thập dữ liệu khoa học
    "COMM":     b"COMM",      # Chế độ truyền thông
    "SAFE":     b"SAFE",      # Chế độ an toàn (tắt module phụ)
    "SLEEP":    b"SLEEP",     # Ngủ tiết kiệm năng lượng
    "CONFIG":   b"CONFIG",    # Cập nhật cấu hình từ GCS
    "RESET":    b"RESET",     # Reset hệ thống CubeSat
    "PING":     b"PING",      # Kiểm tra kết nối
}


class CommandManager:
    def __init__(self, lora: LoRa):
        """Khởi tạo bộ điều khiển lệnh, nhận đối tượng LoRa"""
        self.lora = lora

    # ----------------------------
    # 1️ Gửi lệnh điều khiển
    # ----------------------------
    def send_command(self, cmd_name: str):
        """
        Gửi lệnh điều khiển tới vệ tinh.
        Tự động kiểm tra hợp lệ, gửi, chờ phản hồi ACK.
        """
        cmd_name = cmd_name.upper().strip()

        if cmd_name not in VALID_COMMANDS:
            print(f" Lệnh '{cmd_name}' không hợp lệ!")
            print(f" Lệnh hợp lệ: {', '.join(VALID_COMMANDS.keys())}")
            return

        data = VALID_COMMANDS[cmd_name]
        self.lora.send(pkg_type=0x02, data=data)
        print(f" Đã gửi lệnh: {cmd_name}")
        log.cmd(f"Gửi lệnh: {cmd_name}", "TX")

# Chờ ACK
        reply = self.lora.receive()
        if reply and reply["type"] == 0x03:  # ACK
            try:
                msg = reply["data"].decode(errors="ignore")
                print(f" Nhận ACK: {msg}")
            except Exception:
                log.cmd("Nhận ACK từ vệ tinh: {msg}", "RX")
                print(" Nhận ACK từ vệ tinh.")
        else:
            log.warn("Không nhận được phản hồi (ACK)")
            print(" Không nhận được phản hồi (ACK).")

    # ----------------------------
    # 2️ Gửi gói cấu hình (tùy chọn)
    # ----------------------------
    def send_config(self, config_dict: dict):
        """
        Gửi lệnh CONFIG kèm dữ liệu cấu hình (JSON hoặc key=value)
        """
        import json
        try:
            payload = json.dumps(config_dict).encode()
        except Exception:
            print(" Cấu hình không hợp lệ, không thể gửi.")
            return
        
        self.lora.send(pkg_type=0x02, data=b"CONFIG" + payload)
        print(" Đã gửi gói CONFIG.")
        reply = self.lora.receive()
        if reply and reply["type"] == 0x03:
            print(" Cấu hình đã được vệ tinh xác nhận.")
        else:
            print(" Không nhận phản hồi từ CubeSat.")

    # ----------------------------
    # 3️ Ping kiểm tra kết nối
    # ----------------------------
    def ping(self):
        """Gửi PING và chờ phản hồi"""
        self.lora.send(pkg_type=0x02, data=b"PING")
        print(" Đang ping vệ tinh...")
        reply = self.lora.receive()
        if reply and reply["data"] == b"ACK_PING":
            print(" Vệ tinh phản hồi PING OK.")
        else:
            print(" Không có phản hồi PING.")
