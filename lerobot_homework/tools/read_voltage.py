# 读取 SO-ARM101 全部舵机 Present_Voltage（寄存器 62，单位 0.1V）
# 额定 6~8.4V；USB 5V 直供会显示 4.x~5V（欠压 → 舵机指示灯闪 / Input voltage error）
# 用法：python lerobot_homework\tools\read_voltage.py
import serial
import time


def read_voltage(s, servo_id, addr=62, length=1):
    inst = 0x02  # Read
    pkt_len = 2
    chk = (~(servo_id + pkt_len + inst + addr + length)) & 0xFF
    frame = bytes([0xFF, 0xFF, servo_id, pkt_len, inst, addr, length, chk])
    s.reset_input_buffer()
    s.write(frame)
    time.sleep(0.2)
    got = s.in_waiting
    data = s.read(got)
    if len(data) >= 5 and data[0] == 0xFF and data[1] == 0xFF:
        err = data[4]
        if err == 0 and len(data) >= 7:
            raw = data[5]
            return raw * 0.1
        return f"ERR=0x{err:02x}"
    return "no response"


s = serial.Serial("COM3", 1_000_000, timeout=0.3)
for sid in range(1, 7):
    v = read_voltage(s, sid)
    tag = "  OK" if isinstance(v, float) and 6.0 <= v <= 8.4 else "  <-- 电压异常" if isinstance(v, float) else ""
    print(f"ID{sid}: {v} V{tag}")
s.close()
