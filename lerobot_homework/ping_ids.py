# 单播 Ping 测试：对 ID 1..6 逐个发 Ping（协议1），检查有效应答帧（FF FF 开头）
# 期望应答：FF FF <ID> 02 <ERR> <CHK>
import serial
import time


def ping_frame(servo_id: int) -> bytes:
    # FF FF ID LEN INST CHK ; LEN=2, INST=0x01(Ping)
    inst = 0x01
    length = 0x02
    chk = (~(servo_id + length + inst)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, inst, chk])


for baud in (1_000_000, 500_000):
    print(f"===== baud {baud} =====")
    try:
        s = serial.Serial("COM3", baud, timeout=0.5)
    except Exception as e:
        print("  open fail:", e)
        continue
    for sid in range(1, 7):
        s.reset_input_buffer()
        s.write(ping_frame(sid))
        time.sleep(0.25)
        got = s.in_waiting
        data = s.read(got) if got else b""
        ok = data.startswith(b"\xff\xff") and len(data) >= 6
        print(f"  ID{sid}: rx={got} valid={ok} raw={data.hex()}")
    s.close()
