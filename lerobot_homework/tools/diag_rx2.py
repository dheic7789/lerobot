# 数据链路诊断 2：扩展波特率列表，检测有效帧头（FF FF）
import serial
import time

FRAME = bytes([0xFF, 0xFF, 0xFE, 0x02, 0x01, 0xFE])  # 广播 Ping（协议1）

for baud in (2_000_000, 4_000_000, 1_000_000, 128_000, 38_400, 9_600):
    try:
        s = serial.Serial("COM3", baud, timeout=0.2)
    except Exception as e:
        print(f"baud {baud:>9}: OPEN FAIL {e}")
        continue
    s.reset_input_buffer()
    n = s.write(FRAME)
    time.sleep(0.3)
    got = s.in_waiting
    data = s.read(s.in_waiting) if got else b""
    s.close()
    valid = data.startswith(b"\xff\xff")
    print(f"baud {baud:>9}: sent={n} rx={got} valid_frame={valid} raw={data.hex()[:60]}")
