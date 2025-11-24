"""
config.py
--------------------
Quản lý cấu hình của phần mềm trạm mặt đất (Ground Station Configuration).
Đọc / ghi / cập nhật file settings.json và hỗ trợ đồng bộ cấu hình với CubeSat.
"""

import json
from pathlib import Path
from datetime import datetime
from com.lora_driver import LoRa
from include.command import CommandManager
from util.logger import Logger
log = Logger()


class ConfigManager:
    def __init__(self, config_path="config/settings.json"):
        """Khởi tạo bộ quản lý cấu hình"""
        self.path = Path(config_path)
        self.config = {}
        self.load()

    # ----------------------------
    # 1️ Đọc cấu hình từ file JSON
    # ----------------------------
    def load(self):
        """Đọc file cấu hình settings.json, tạo mặc định nếu chưa có"""
        if not self.path.exists():
            print(" Không tìm thấy file cấu hình, tạo mới mặc định...")
            self.config = self.default_config()
            self.save()
            return

        try:
            with open(self.path, "r", encoding="utf-8") as f:
                self.config = json.load(f)
            log.info(f" Đã tải cấu hình từ {self.path}")
        except json.JSONDecodeError:
            print(" Lỗi định dạng JSON — khôi phục cấu hình mặc định.")
            self.config = self.default_config()
            self.save()
            
    # ----------------------------
    # 2️ Lưu cấu hình ra file
    # ----------------------------
    def save(self):
        """Ghi cấu hình hiện tại xuống file"""
        try:
            with open(self.path, "w", encoding="utf-8") as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
            log.info(f" Đã lưu cấu hình vào {self.path}")
        except Exception as e:
            print(f" Không thể ghi file cấu hình: {e}")

    # ----------------------------
    # 3️ Lấy giá trị theo key
    # ----------------------------
    def get(self, *keys, default=None):
        """Lấy giá trị theo key lồng nhau, ví dụ: get('communication', 'baudrate')"""
        data = self.config
        try:
            for key in keys:
                data = data[key]
            return data
        except (KeyError, TypeError):
            return default
        

    # ----------------------------
    # 4️ Cập nhật giá trị cấu hình
    # ----------------------------
    def set(self, *keys, value):
        """Cập nhật giá trị theo key lồng nhau"""
        if not keys:
            return
        d = self.config
        for key in keys[:-1]:
            d = d.setdefault(key, {})
        d[keys[-1]] = value
        log.info(f" Đã cập nhật {'.'.join(keys)} = {value}")

    # ----------------------------
    # 5️ In cấu hình hiện tại
    # ----------------------------
    def print_config(self):
        """In toàn bộ cấu hình hiện tại ra console"""
        print("\n=== GROUND STATION CONFIGURATION ===")
        print(json.dumps(self.config, indent=2, ensure_ascii=False))
        
    # ----------------------------
    # 6️ Cấu hình mặc định
    # ----------------------------
    def default_config(self):
        """Trả về cấu hình mặc định"""
        return {
            "station": {
                "ground_id": 0,
                "station_name": "UET_GroundStation",
                "operator": "Team CubeLab",
                "auto_connect": True
            },
            "communication": {
                "com_port": "COM3",
                "baudrate": 9600,
                "satellite_id": 1,
                "packet_header": "AA55",
                "max_retry": 3,
                "timeout_ms": 1500
            },
            "telemetry": {
                "save_raw": True,
                "save_csv": True,
                "log_folder": "./data/logs",
                "telemetry_interval_s": 2,
                "time_sync": True
            },
            "download": {
                "image_folder": "./data/images",
                "max_packet_per_frame": 64,
                "enable_auto_reassemble": True
            },
            "command": {
                "available_modes": ["SCIENCE", "COMM", "SAFE", "SLEEP", "CONFIG"],
                "default_mode": "STANDBY",
                "ack_timeout_ms": 2000
            },
            "config": {
                "default_settings": True,
                "auto_save_changes": True,
                "flash_sync": False
            },
            "visualization": {
                "enable_realtime_graph": True,
                "graph_refresh_rate_ms": 500,
                "theme": "dark"
            }
        }

    # ----------------------------
    # 7️ Đồng bộ cấu hình xuống vệ tinh (qua LoRa)
    # ----------------------------
    def sync_to_satellite(self):
        """Gửi cấu hình hiện tại xuống CubeSat (qua CMD_CONFIG)"""
        try:
            lora = LoRa(str(self.path))
            cmd = CommandManager(lora)

            payload = {
                "baud": self.get("communication", "baudrate"),
                "telemetry_period": self.get("telemetry", "telemetry_interval_s"),
                "enable_camera": True
            }
            print(" Đang gửi cấu hình xuống vệ tinh...")
            cmd.send_config(payload)
            lora.close()

        except Exception as e:
            print(f" Lỗi khi đồng bộ cấu hình với vệ tinh: {e}")
            
    # ----------------------------
    # 8️ Đặt lại mặc định
    # ----------------------------
    def reset_to_default(self):
        """Đặt lại cấu hình mặc định"""
        confirm = input(" Bạn có chắc muốn đặt lại cấu hình mặc định? (y/n): ")
        if confirm.lower() == "y":
            self.config = self.default_config()
            self.save()
            print("Đã khôi phục cấu hình mặc định.")