"""
download.py
--------------------
Quản lý quá trình tải dữ liệu (telemetry, ảnh, log, dữ liệu khoa học)
từ vệ tinh CubeSat về trạm mặt đất qua LoRa.
"""

import os
import time
from pathlib import Path
from com.lora_driver import LoRa
from util.frame import parse_frame
from util.logger import Logger
log = Logger()


class DownloadManager:
    def __init__(self, lora: LoRa):
        """Khởi tạo bộ tải dữ liệu"""
        self.lora = lora
        self.session_active = False
        self.output_dir = Path(self.lora.log_dir) / "downloads"
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.buffer = {}
        self.expected_packets = 0
        self.session_id = 0

    # ----------------------------
    # 1️ Bắt đầu phiên tải dữ liệu
    # ----------------------------
    def start_session(self, session_id: int, expected_packets: int):
        """Bắt đầu phiên tải dữ liệu mới"""
        self.session_id = session_id
        self.expected_packets = expected_packets
        self.buffer = {}
        self.session_active = True

        print(f"\n Bắt đầu phiên tải dữ liệu #{session_id}")
        print(f" Số gói dự kiến: {expected_packets}")

        # Gửi lệnh yêu cầu vệ tinh bắt đầu truyền
        log.info(f"Bắt đầu phiên tải dữ liệu #{session_id} với {expected_packets} gói")
        self.lora.send(pkg_type=0x02, data=f"DOWNLOAD:{session_id}".encode())
    
    # ----------------------------
    # 2️ Nhận gói dữ liệu
    # ----------------------------
    def receive_packet(self):
        """Nhận gói dữ liệu từ vệ tinh và lưu vào bộ đệm"""
        packet = self.lora.receive()
        if not packet:
            return None

        if packet["type"] != 0x04:
            # Không phải gói dữ liệu download
            return None
        seq_id = int.from_bytes(packet["data"][2:4], "little")
        log.data(f"Nhận gói dữ liệu seq_id={seq_id}", "RX")

        try:
            # Giả sử cấu trúc data: [2 byte session_id][2 byte seq_id][payload]
            data = packet["data"]
            session_id = int.from_bytes(data[:2], "little")
            seq_id = int.from_bytes(data[2:4], "little")
            payload = data[4:]

            if session_id != self.session_id:
                print(f" Gói không khớp session (nhận {session_id}, mong {self.session_id})")
                return None
            
            self.buffer[seq_id] = payload
            print(f" Nhận gói {seq_id}/{self.expected_packets}")

            return seq_id

        except Exception as e:
            print(f" Lỗi khi đọc gói dữ liệu: {e}")
            return None

    # ----------------------------
    # 3️ Ghép dữ liệu và lưu file
    # ----------------------------
    def reassemble(self, filename="download.bin"):
        """Ghép các gói dữ liệu đã nhận và lưu ra file"""
        if not self.buffer:
            print(" Không có dữ liệu để ghép.")
            return None

        sorted_keys = sorted(self.buffer.keys())
        full_data = b''.join(self.buffer[k] for k in sorted_keys)

        file_path = self.output_dir / filename
        with open(file_path, "wb") as f:
            f.write(full_data)

        log.info(f"Dữ liệu tải về đã lưu tại: {file_path}")
        return file_path

    # ----------------------------
    # 4️ Kiểm tra tiến trình tải
    # ----------------------------
    def progress(self):
        """Hiển thị tiến độ tải hiện tại"""
        received = len(self.buffer)
        total = self.expected_packets
        percent = (received / total) * 100 if total > 0 else 0
        print(f" Tiến độ: {received}/{total} gói ({percent:.1f}%)")

    # ----------------------------
    # 5️ Kết thúc và xác nhận
    # ----------------------------
    def finish(self):
        """Kết thúc phiên tải dữ liệu"""
        if not self.session_active:
            return

        self.session_active = False
        print("\n Phiên tải dữ liệu hoàn tất.")
        self.progress()
        self.lora.send(pkg_type=0x02, data=b"DOWNLOAD_ACK")

    # ----------------------------
    # 6️ Tải toàn bộ (tự động)
    # ----------------------------
    def auto_download(self, session_id=1, expected_packets=10, save_as="data.bin"):
        """Quy trình tự động: bắt đầu → nhận → ghép → lưu"""
        self.start_session(session_id, expected_packets)
        start = time.time()

        while len(self.buffer) < expected_packets and (time.time() - start) < 120:
            self.receive_packet()
            self.progress()

        self.finish()
        return self.reassemble(filename=save_as)