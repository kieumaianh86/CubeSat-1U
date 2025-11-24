"""
listen.py
--------------------
Chương trình lắng nghe dữ liệu từ vệ tinh CubeSat qua LoRa.
Tự động phân tích gói dữ liệu, ghi log, hiển thị Telemetry theo thời gian thực.
"""

import threading
import time
from datetime import datetime
from com.lora_driver import LoRa
from util.frame import parse_frame
from util.logger import Logger
log = Logger()


class Listener:
    def __init__(self, lora: LoRa, log_enable=True):
        """Khởi tạo Listener với đối tượng LoRa"""
        self.lora = lora
        self.running = False
        self.thread = None
        self.log_enable = log_enable
        self.log_file = self.lora.log_dir / f"telemetry_{time.strftime('%Y%m%d')}.csv"

    # ----------------------------
    # 1️ Bắt đầu lắng nghe
    # ----------------------------
    def start(self):
        """Khởi động chế độ lắng nghe trong luồng riêng"""
        if self.running:
            print(" Listener đã chạy.")
            return
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()
        print(" Listener bắt đầu lắng nghe dữ liệu từ vệ tinh...")

    # ----------------------------
    # 2️ Dừng lắng nghe
    # ----------------------------
    def stop(self):
        """Dừng lắng nghe"""
        if not self.running:
            print(" Listener chưa hoạt động.")
            return
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)
        print(" Listener đã dừng.")
        
    # ----------------------------
    # 3️ Vòng lặp lắng nghe chính
    # ----------------------------
    def _listen_loop(self):
        """Luồng chính: liên tục đọc dữ liệu từ LoRa"""
        while self.running:
            try:
                packet = self.lora.receive()
                if not packet:
                    continue

                pkt_type = packet["type"]
                data = packet["data"]

                # Phân loại gói theo type
                if pkt_type == 0x01:
                    self._handle_telemetry(data)
                elif pkt_type == 0x03:
                    self._handle_ack(data)
                elif pkt_type == 0x04:
                    self._handle_download_fragment(data)
                elif pkt_type == 0x05:
                    self._handle_warning(data)
                else:
                    self._handle_unknown(pkt_type, data)
                    
            except Exception as e:
                print(f" Lỗi trong Listener: {e}")
                time.sleep(0.2)

    # ----------------------------
    # 4️ Xử lý gói Telemetry
    # ----------------------------
    def _handle_telemetry(self, data: bytes):
        """Xử lý và ghi log gói Telemetry"""
        try:
            text = data.decode(errors="ignore").strip()
            log.cmd(f"Nhận Telemetry: {text}")
            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f" [{timestamp}] TELEMETRY: {text}")

            if self.log_enable:
                with open(self.log_file, "a", encoding="utf-8") as f:
                    f.write(f"{timestamp}, {text}\n")

        except Exception as e:
            print(f" Lỗi khi ghi log Telemetry: {e}")

    # ----------------------------
    # 5️ Xử lý ACK từ CubeSat
    # ----------------------------
    def _handle_ack(self, data: bytes):
        """Hiển thị ACK xác nhận"""
        msg = data.decode(errors="ignore").strip()
        log.cmd(f"Nhận ACK: {msg}", "RX")
        print(f" ACK từ CubeSat: {msg}")

    # ----------------------------
    # 6️ Xử lý gói download fragment
    # ----------------------------
    def _handle_download_fragment(self, data: bytes):
        """Thông báo gói dữ liệu tải về"""
        try:
            session_id = int.from_bytes(data[:2], "little")
            seq_id = int.from_bytes(data[2:4], "little")
            print(f" Gói tải xuống: Session={session_id},_Seq={seq_id}, Len={len(data) - 4} bytes")
        except Exception:
            print(" Nhận gói tải xuống (dữ liệu không xác định)")

    # ----------------------------
    # 7️ Xử lý cảnh báo (warning)
    # ----------------------------
    def _handle_warning(self, data: bytes):
        """Thông báo cảnh báo từ CubeSat"""
        msg = data.decode(errors="ignore")
        log.warn(f"Cảnh báo từ CubeSat: {msg}", "RX")
        print(f" CẢNH BÁO TỪ CUBESAT: {msg}")

    # ----------------------------
    # 8️ Xử lý gói không xác định
    # ----------------------------
    def _handle_unknown(self, pkt_type, data: bytes):
        """Xử lý gói không nhận dạng được"""
        log.warn(f"Nhận gói không xác định (type={pkt_type:#02x}, len={len(data)})", "RX")