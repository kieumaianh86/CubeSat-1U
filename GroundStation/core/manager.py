"""
app_manager.py
--------------------
Module quản lý toàn bộ luồng điều khiển chính của Ground Station Console.
Tách riêng để giảm độ phức tạp của hàm main().
"""

import time
import threading
from include.init import InitManager
from include.command import CommandManager
from include.download import DownloadManager
from include.listen import TelemetryListener
from include.config import ConfigManager


class GroundStationApp:
    def __init__(self, logger):
        self.log = logger
        self.running = True
        self.init_system()

    # ----------------------------
    # 1️ Khởi tạo toàn hệ thống
    # ----------------------------
    def init_system(self):
        self.log.info("=== KHỞI ĐỘNG GROUND STATION ===")
        try:
            self.init = InitManager("config/settings.json")
            self.lora = self.init.lora
            self.cfg = self.init.cfg

            # Tạo các manager nghiệp vụ
            self.cmd_mgr = CommandManager(self.lora)
            self.dl_mgr = DownloadManager(self.lora)
            self.listener = TelemetryListener(self.lora)
            self.config_mgr = ConfigManager("config/settings.json")

            self.log.info(" Hệ thống đã khởi tạo thành công.")
        except Exception as e:
            self.log.error(f" Lỗi khi khởi tạo hệ thống: {e}")
            raise e

    # ----------------------------
    # 2️ In menu
    # ----------------------------
    def show_menu(self):
        print("\n" + "=" * 60)
        print("  GROUND STATION CONSOLE – CubeSat 1U")
        print("=" * 60)
        print("1️  Gửi lệnh điều khiển")
        print("2️  Bật chế độ nghe Telemetry")
        print("3️  Tải dữ liệu từ vệ tinh")
        print("4️  Xem cấu hình hiện tại")
        print("5️  Thoát")
        print("-" * 60)

    # ----------------------------
    # 3️ Chạy vòng lặp chính
    # ----------------------------
    def run(self):
        while self.running:
            self.show_menu()
            choice = input(" Nhập lựa chọn (1–5): ").strip()
            self.handle_choice(choice)

    # ----------------------------
    # 4️ Xử lý lựa chọn
    # ----------------------------
    def handle_choice(self, choice):
        if choice == "1":
            self._handle_command_mode()
        elif choice == "2":
            self._handle_listen_mode()
        elif choice == "3":
            self._handle_download_mode()
        elif choice == "4":
            self._handle_show_config()
        elif choice == "5":
            self._handle_exit()
        else:
            print("Lựa chọn không hợp lệ. Vui lòng nhập 1–5.")

    # ----------------------------
    # 5️ Các chế độ con
    # ----------------------------
    def _handle_command_mode(self):
        cmd = input("🔹 Nhập lệnh (ví dụ: SAFE, SCIENCE, RESET): ").strip().upper()
        if not cmd:
            print("Lệnh không hợp lệ.")
            return
        self.log.cmd(f"Gửi lệnh: {cmd}", "TX")
        self.cmd_mgr.send_command(cmd)

    def _handle_listen_mode(self):
        print("  Đang nghe Telemetry... (nhấn Ctrl+C để dừng)")
        listen_thread = threading.Thread(target=self.listener.run, daemon=True)
        listen_thread.start()
        try:
            while listen_thread.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.listener.stop()
            print("\n Dừng chế độ nghe Telemetry.")
            self.log.info("Dừng chế độ nghe Telemetry.")

    def _handle_download_mode(self):
        try:
            session = int(input("🔹 Nhập session ID: "))
            packets = int(input("🔹 Số gói dự kiến: "))
            self.dl_mgr.start_session(session, packets)
            self.dl_mgr.run()
        except ValueError:
            print(" Giá trị nhập không hợp lệ.")
        except KeyboardInterrupt:
            print("\n Dừng tải dữ liệu.")

    def _handle_show_config(self):
        self.cfg.load()
        print("\n Cấu hình hiện tại:")
        print(self.cfg.data)
        self.log.info("Hiển thị cấu hình hệ thống.")

    def _handle_exit(self):
        print(" Đang tắt hệ thống...")
        self.init.shutdown()
        self.log.info("Hệ thống Ground Station đã tắt.")
        self.running = False