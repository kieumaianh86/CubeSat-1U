"""
init.py
--------------------
Khởi tạo hệ thống trạm mặt đất (Ground Station Initialization)
- Đọc cấu hình
- Thiết lập kết nối LoRa
- Kiểm tra liên kết vệ tinh
- Chuẩn bị module lệnh, cấu hình, download
"""

import os
import time
from pathlib import Path

from include.config import ConfigManager
from include.command import CommandManager
from include.download import DownloadManager
from com.lora_driver import LoRa
from util.logger import Logger
log = Logger()


class GroundStationSystem:
    def __init__(self, config_path="config/settings.json"):
        """Khởi tạo toàn bộ hệ thống trạm mặt đất"""
        log.info("\n === KHỞI TẠO HỆ THỐNG GROUND STATION ===")

        # 1️ Đọc file cấu hình
        self.cfg = ConfigManager(config_path)

        # 2️ Khởi tạo module giao tiếp LoRa
        try:
            self.lora = LoRa(config_path)
        except Exception as e:
            print(f" Không thể khởi tạo LoRa: {e}")
            self.lora = None
            return

        # 3️ Khởi tạo các module phụ trợ
        self.cmd = CommandManager(self.lora)
        self.download = DownloadManager(self.lora)

        # 4️ Kiểm tra kết nối vệ tinh
        print("\n Kiểm tra liên kết với vệ tinh...")
        if self.lora.check_link():
            log.info(" Liên kết vệ tinh hoạt động bình thường.")
        else:
            log.warn(" Không có phản hồi từ vệ tinh. Hãy kiểm tra kết nối LoRa.")
        log.info(" Hệ thống Ground Station khởi tạo hoàn tất.")

        # 5️ Khởi tạo thư mục lưu dữ liệu
        data_dir = Path(self.cfg.get("telemetry", "log_folder", default="./data/logs"))
        data_dir.mkdir(parents=True, exist_ok=True)
        print(f" Thư mục dữ liệu: {data_dir.resolve()}")

        print(" Hệ thống Ground Station đã sẵn sàng.\n")

    # ----------------------------
    # 1️ Gửi lệnh điều khiển
    # ----------------------------
    def send_command(self, cmd: str):
        """Gửi lệnh điều khiển vệ tinh"""
        if not self.lora:
            print(" Chưa có kết nối LoRa.")
            return
        self.cmd.send_command(cmd)

    # ----------------------------
    # 2️ Đồng bộ cấu hình xuống vệ tinh
    # ----------------------------
    def sync_config(self):
        """Gửi cấu hình hiện tại xuống vệ tinh"""
        self.cfg.sync_to_satellite()

    # ----------------------------
    # 3️ Tải dữ liệu
    # ----------------------------
    def download_data(self, session_id=1, expected_packets=10, save_as="data.bin"):
        """Bắt đầu quá trình tải dữ liệu"""
        return self.download.auto_download(session_id, expected_packets, save_as)

    # ----------------------------
    # 4️ Đóng hệ thống
    # ----------------------------
    def shutdown(self):
        """Đóng tất cả kết nối và giải phóng tài nguyên"""
        if self.lora:
            self.lora.close()
        log.info(" Hệ thống Ground Station đã tắt.")

InitManager = GroundStationSystem