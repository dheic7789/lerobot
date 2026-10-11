# 枚举所有串口，并对每个口发 ping ID1（1M / 500k），检查有效应答
import serial
import serial.tools.list_ports
import time


def ping_frame(servo_id: int) -> bytes:
    inst = 0x01
    length = 0x02
    chk = (~(servo_id + length + inst)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, inst, chk])


ports = list(serial.tools.list_ports.comports())
print(f"发现 {len(ports)} 个串口:")
for p in ports:
    print(f"  {p.device}: {p.description} | {p.hwid}")

for p in ports:
    port = p.device
    for baud in (1_000_000, 500_000):
        try:
            s = serial.Serial(port, baud, timeout=0.4)
        except Exception as e:
            print(f"{port} @{baud}: OPEN FAIL {e}")
            continue
        s.reset_input_buffer()
        s.write(ping_frame(1))
        time.sleep(0.3)
        got = s.in_waiting
        data = s.read(got) if got else b""
        s.close()
        ok = data.startswith(b"\xff\xff") and len(data) >= 6
        print(f"{port} @{baud}: rx={got} valid={ok} raw={data.hex()}")
