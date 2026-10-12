# -*- coding: utf-8 -*-
"""Read-only live position monitor for an SCS215 servo bus."""

import argparse
import time

import serial


PORT = "COM5"
BAUDRATE = 1_000_000
DEFAULT_IDS = (1, 2, 3, 4, 5, 6)
POSITION_MODULUS = 1 << 10


def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def read_position(ser, servo_id):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [0x38, 2]))
    reply = ser.read(8)
    if (len(reply) != 8 or reply[:2] != b"\xff\xff"
            or reply[2] != servo_id or reply[3] != 4
            or reply[4] != 0 or sum(reply[2:]) & 0xFF != 0xFF):
        return None
    return (reply[5] << 8) | reply[6]


def modular_delta(previous, current, modulus=POSITION_MODULUS):
    """Return the shortest signed movement between two wrapped encoder values."""
    return (current - previous + modulus // 2) % modulus - modulus // 2


def main():
    parser = argparse.ArgumentParser(
        description="Continuously read SCS215 positions without moving or changing servos."
    )
    parser.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument("--ids", nargs="+", type=int, default=DEFAULT_IDS)
    parser.add_argument("--interval", type=float, default=0.2, help="Seconds between scans")
    args = parser.parse_args()

    if (not args.ids or len(set(args.ids)) != len(args.ids)
            or any(not 1 <= servo_id <= 0xFD for servo_id in args.ids)):
        parser.error("IDs must be unique integers from 1 to 253.")
    if args.interval < 0.05:
        parser.error("--interval must be at least 0.05 seconds.")

    try:
        ser = serial.Serial(args.port, BAUDRATE, timeout=0.08)
    except serial.SerialException as error:
        parser.error(f"Cannot open {args.port}: {error}")

    print("Read-only monitor: no torque, goal-position, or limit writes are sent.")
    print("Press Ctrl+C to stop. Each ID is queried sequentially (SCS Protocol 1).")
    previous = {}
    continuous = {}
    try:
        with ser:
            while True:
                started = time.monotonic()
                fields = []
                for servo_id in args.ids:
                    raw_position = read_position(ser, servo_id)
                    if raw_position is None:
                        fields.append(f"ID{servo_id}=NO_REPLY")
                        continue
                    position = raw_position % POSITION_MODULUS

                    prior = previous.get(servo_id)
                    if prior is None:
                        continuous[servo_id] = position
                        fields.append(f"ID{servo_id}={position} / {continuous[servo_id]}")
                    else:
                        delta = modular_delta(prior, position)
                        continuous[servo_id] += delta
                        fields.append(
                            f"ID{servo_id}={position} / {continuous[servo_id]} ({delta:+d})"
                        )
                    previous[servo_id] = position

                stamp = time.strftime("%H:%M:%S")
                print(f"{stamp} | " + " | ".join(fields), flush=True)
                time.sleep(max(0.0, args.interval - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("\nMonitor stopped; no servo settings were changed.")


if __name__ == "__main__":
    main()