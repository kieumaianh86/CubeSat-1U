"""
main.py
-----------------------
Console Ground Station cho CubeSat 1U.
Hỗ trợ:
- Gửi lệnh (SCIENCE, SAFE, COMM, RESET…)
- Nhận Telemetry, Data, Science Packet
- Ping
- Lưu Telemetry
"""

import time
from com.lora_driver import LoRa
from include.command import CommandManager
from include.packet_parser import handle_packet
from util.logger import Logger

log = Logger()


# ---------------------------------------------------------------------
#  Hàm in menu
# ---------------------------------------------------------------------
def show_menu():
    print("\n================== GROUND STATION MENU ==================")
    print("1) Gửi lệnh điều khiển")
    print("2) Ping vệ tinh")
    print("3) Lắng nghe dữ liệu")
    print("4) Gửi cấu hình")
    print("5) Thoát")
    print("==========================================================\n")


# ---------------------------------------------------------------------
#  Gửi lệnh người dùng nhập vào
# ---------------------------------------------------------------------
def handle_send_command(cmd_mgr: CommandManager):
    cmd = input("Nhập lệnh (SCIENCE, SAFE, COMM, RESET, SLEEP, PING...): ").strip()
    cmd_mgr.send_command(cmd)


# ---------------------------------------------------------------------
#  Ping vệ tinh
# ---------------------------------------------------------------------
def handle_ping(cmd_mgr: CommandManager):
    print("📡 Thực hiện Ping...")
    cmd_mgr.ping()


# ---------------------------------------------------------------------
#  Lắng nghe dữ liệu LoRa
# ---------------------------------------------------------------------
def handle_listen(lora: LoRa):
    print("🔍 Đang lắng nghe gói tin từ CubeSat (nhấn Ctrl+C để thoát)...")
    try:
        while True:
            frame = lora.receive()
            if frame:
                packet = handle_packet(frame)
                if packet:
                    print(f"📥 GÓI NHẬN: {packet}")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n⛔ Dừng lắng nghe.")


# ---------------------------------------------------------------------
#  Gửi cấu hình CONFIG kèm JSON
# ---------------------------------------------------------------------
def handle_send_config(cmd_mgr: CommandManager):
    print("Nhập cấu hình JSON (ví dụ: {\"mode\": \"highspeed\", \"interval\": 5})")
    raw = input("CONFIG> ").strip()

    try:
        import json
        config_dict = json.loads(raw)
        cmd_mgr.send_config(config_dict)
    except Exception:
        print("⚠️ JSON không hợp lệ.")


# ---------------------------------------------------------------------
#  MAIN LOOP
# ---------------------------------------------------------------------
def main():
    print("🚀 Khởi động Ground Station...")

    # Khởi tạo kết nối LoRa
    lora = LoRa(settings_path="config/settings.json")
    cmd_mgr = CommandManager(lora)

    while True:
        show_menu()
        choice = input("Chọn chức năng: ").strip()

        if choice == "1":
            handle_send_command(cmd_mgr)

        elif choice == "2":
            handle_ping(cmd_mgr)

        elif choice == "3":
            handle_listen(lora)

        elif choice == "4":
            handle_send_config(cmd_mgr)

        elif choice == "5":
            print("👋 Thoát chương trình Ground Station.")
            break

        else:
            print("⚠️ Lựa chọn không hợp lệ.")


# -------------------------------------------------------------
#  Chạy chương trình
# -------------------------------------------------------------
if __name__ == "__main__":
    main()
