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
    "SCIENCE":  b"SCIENCE",
    "COMM":     b"COMM",
    "SAFE":     b"SAFE",
    "SLEEP":    b"SLEEP",
    "CONFIG":   b"CONFIG",
    "RESET":    b"RESET",
    "PING":     b"PING",
}


class CommandManager:
    def __init__(self, lora: LoRa):
        """Khởi tạo bộ điều khiển lệnh, nhận đối tượng LoRa"""
        self.lora = lora

    # ----------------------------
    # 1) Gửi lệnh điều khiển
    # ----------------------------
    def send_command(self, cmd_name: str):
        """
        Gửi lệnh điều khiển theo frame mới:
        [0xAA][0x02][PAYLOAD=b"CMD"][CRC]
        """
        cmd_name = cmd_name.upper().strip()

        if cmd_name not in VALID_COMMANDS:
            print(f"Lệnh '{cmd_name}' không hợp lệ!")
            print(f"Lệnh hợp lệ: {', '.join(VALID_COMMANDS.keys())}")
            return

        payload = VALID_COMMANDS[cmd_name]

        # Gửi bằng TYPE = 0x02
        self.lora.send(packet_type=0x02, payload=payload)
        print(f"Đã gửi lệnh: {cmd_name}")
        log.cmd(f"Gửi lệnh: {cmd_name}", "TX")

        # -----------------------------
        # Chờ ACK
        # -----------------------------
        reply = self.lora.receive()
        if reply and reply["type"] == 0x03:
            try:
                msg = reply["payload"].decode(errors="ignore")
                print(f"ACK: {msg}")
            except Exception:
                print("Nhận ACK từ vệ tinh.")
        else:
            log.warn("Không nhận được ACK.")
            print("Không nhận được phản hồi (ACK).")

    # ----------------------------
    # 2) Gửi cấu hình CONFIG
    # ----------------------------
    def send_config(self, config_dict: dict):
        """
        Gửi gói CONFIG:
        [0xAA][0x02][b"CONFIG" + JSON][CRC]
        """
        import json

        try:
            cfg_json = json.dumps(config_dict).encode()
        except Exception:
            print("Cấu hình không hợp lệ, không thể mã hóa JSON.")
            return

        payload = b"CONFIG" + cfg_json
        self.lora.send(packet_type=0x02, payload=payload)

        print("Đã gửi gói CONFIG.")

        reply = self.lora.receive()
        if reply and reply["type"] == 0x03:
            print("Vệ tinh xác nhận CONFIG.")
        else:
            print("Không nhận phản hồi CONFIG.")

    # ----------------------------
    # 3) Ping vệ tinh
    # ----------------------------
    def ping(self):
        """
        Ping theo chuẩn mới:
        gửi payload b"PING"
        chờ payload b"ACK_PING"
        """
        self.lora.send(packet_type=0x02, payload=b"PING")
        print("Đang ping vệ tinh...")

        reply = self.lora.receive()

        if reply and reply["type"] == 0x03 and reply["payload"] == b"ACK_PING":
            print("🟢 Ping OK — Vệ tinh phản hồi.")
            return True


        print("Không phản hồi PING.")
        return False