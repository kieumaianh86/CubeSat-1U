"""
main.py
--------------------
Điểm vào chính của phần mềm Trạm Mặt Đất CubeSat 1U.
"""

from core.manager import GroundStationApp
from util.logger import Logger


def main():
    """Chạy ứng dụng Ground Station"""
    log = Logger("./data/logs")
    app = GroundStationApp(log)
    app.run()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n Người dùng dừng chương trình.")
    except Exception as ex:
        print(f" Lỗi không xác định: {ex}")