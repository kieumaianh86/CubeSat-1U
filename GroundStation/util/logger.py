"""
logger.py
--------------------
Bộ ghi log trung tâm cho Ground Station.
Ghi log ra file (theo ngày) và console với nhiều mức độ (INFO, WARN, ERROR, CMD, DATA).
"""

import os
import sys
import time
from pathlib import Path
from datetime import datetime


class Logger:
    def __init__(self, log_dir="./data/logs"):
        """Khởi tạo logger, tự động tạo thư mục log nếu chưa có"""
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_dir / f"groundstation_{time.strftime('%Y%m%d')}.log"
        self._write_header()

    # ----------------------------
    # 1️ Ghi dòng log chung
    # ----------------------------
    def log(self, level: str, message: str, to_console=True):
        """Ghi log ra file và console (nếu bật)"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] [{level.upper():^5}] {message}"

        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception as e:
            print(f" Không thể ghi log file: {e}")

        if to_console:
            color = self._get_color(level)
            print(color + line + "\033[0m")

    # ----------------------------
    # 2️ Log mức thông tin
    # ----------------------------
    def info(self, message: str):
        self.log("INFO", message)
        
    # ----------------------------
    # 3️ Log cảnh báo
    # ----------------------------
    def warn(self, message: str):
        self.log("WARN", message)

    # ----------------------------
    # 4️ Log lỗi
    # ----------------------------
    def error(self, message: str):
        self.log("ERROR", message)

    # ----------------------------
    # 5️ Log lệnh điều khiển
    # ----------------------------
    def cmd(self, message: str, direction="TX"):
        """Ghi log cho lệnh (direction = TX hoặc RX)"""
        prefix = "--> Gói" if direction == "TX" else "<-- Nhận"
        self.log("CMD", f"{prefix} {message}")

    # ----------------------------
    # 6️ Log dữ liệu telemetry
    # ----------------------------
    def data(self, message: str):
        self.log("DATA", f" {message}")

    # ----------------------------
    # 7️ Tạo header mỗi ngày
    # ----------------------------
    def _write_header(self):
        """Tạo dòng tiêu đề mới trong file log"""
        try:
            with open(self.log_file, "a", encoding="utf-8") as f:
                f.write("\n" + "=" * 60 + "\n")
                f.write(f" Log session started: {datetime.now()}\n")
                f.write("=" * 60 + "\n")
        except (OSError, IOError):
            pass

    # ----------------------------
    # 8️ Màu cho console
    # ----------------------------
    def _get_color(self, level: str) -> str:
        """Đặt màu theo mức độ log"""
        colors = {
            "INFO": "\033[92m",   # xanh lá
            "WARN": "\033[93m",   # vàng
            "ERROR": "\033[91m",  # đỏ
            "CMD": "\033[96m",    # xanh dương nhạt
            "DATA": "\033[95m",   # tím
        }
        return colors.get(level.upper(), "\033[0m")