# 数据链路诊断：对每个波特率发广播 Ping（Feetech 协议1），统计 RX 返回字节数
# 有字节 = 链路物理通（问题在接线方向/波特率）；0 字节 = TX 没到舵机或 RX 没回来
import serial
import time

FRAME = bytes([0xFF, 0xFF, 0xFE, 0x02, 0x01, 0xFE])  # 广播 Ping

for baud in (1_000_000, 500_000, 250_000, 921_600, 115_200, 57_600):
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
    print(f"baud {baud:>9}: sent={n} rx_bytes={got} raw={data.hex()[:60]}")
