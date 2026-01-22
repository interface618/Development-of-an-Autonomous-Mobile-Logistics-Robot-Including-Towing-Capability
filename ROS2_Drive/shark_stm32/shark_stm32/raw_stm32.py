#!/usr/bin/env python3
import serial
import time

def main():
    # 포트와 보레이트는 환경에 맞게 수정하세요
    port = "/dev/ttyACM0"
    baud = 115200

    # 시리얼 포트 열기
    ser = serial.Serial(port, baud, timeout=1)
    print(f"[INFO] Connected to {port} @ {baud}")

    # 문자 '3' 보내기 (개행은 상황에 따라 붙여도 되고 안 붙여도 됩니다)
    ser.write(b'4\n')          # 그냥 '3' 하나만
    # ser.write(b'3\n')      # 줄바꿈까지 포함해서 보내고 싶으면 이렇게

    print("[INFO] Sent '3'")

    # 잠시 대기 후 포트 닫기
    time.sleep(0.1)
    ser.close()

if __name__ == "__main__":
    main()
