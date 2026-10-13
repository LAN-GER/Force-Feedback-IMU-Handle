"""Check USB JSON replies without sending motor/heater commands or reset signals."""

import argparse
import json
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="Probe the handle USB JSON protocol")
    parser.add_argument("--port", help="Serial port, e.g. COM3")
    parser.add_argument("--list", action="store_true", help="List serial ports without opening them")
    parser.add_argument("--diag", action="store_true", help="Also query cached IMU diagnostics (requires current firmware)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=3.0, help="Receive window per request")
    parser.add_argument("--max-log-lines", type=int, default=60, help="Maximum printed non-telemetry RX lines")
    args = parser.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    try:
        import serial
        from serial.tools.list_ports import comports
    except ImportError:
        parser.exit(1, "pyserial is required. Run this script in the ESP-IDF Python environment.\n")

    ports = list(comports())
    if args.list:
        for port in ports:
            usb = f"USB {port.vid:04X}:{port.pid:04X}" if port.vid is not None else "non-USB"
            print(f"{port.device}: {port.description} ({usb})")
        return
    if not args.port:
        parser.error("specify --port or --list")
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    # Set modem signals BEFORE opening; never toggle DTR/RTS to reset the board.
    connection = serial.Serial(port=None, baudrate=args.baud, timeout=0.1, write_timeout=1)
    connection.dtr = False
    connection.rts = False
    connection.port = args.port
    try:
        connection.open()
    except serial.SerialException as error:
        parser.exit(1, f"Cannot open {args.port}: {error}\nClose the web connection and IDF monitor, then retry.\n")

    buffer = bytearray()
    acknowledgements = set()
    telemetry = 0
    log_lines = 0
    log_suppressed = False
    sensor_failure = False
    reboots = 0

    def send(cmd):
        text = json.dumps({"cmd": cmd}, separators=(",", ":"))
        connection.write((text + "\n").encode("utf-8"))
        print(f"TX {text}", flush=True)

    def receive(seconds):
        nonlocal telemetry, log_lines, log_suppressed, sensor_failure, reboots
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            buffer.extend(connection.read(min(connection.in_waiting or 1, 4096)))
            while b"\n" in buffer:
                raw, _, remaining = buffer.partition(b"\n")
                buffer[:] = remaining
                text = raw.decode("utf-8", errors="replace").strip()
                if not text:
                    continue
                if "Chip ID 不匹配" in text or "drv_bmi088_init()" in text:
                    sensor_failure = True
                if "Rebooting..." in text:
                    reboots += 1
                prefix = text.find(">>")
                value = None
                if prefix >= 0:
                    try:
                        value = json.loads(text[prefix + 2:])
                    except json.JSONDecodeError:
                        pass
                if isinstance(value, dict) and value.get("t") == "telem":
                    telemetry += 1
                    if telemetry <= 3:
                        print(f"RX {text}", flush=True)
                else:
                    log_lines += 1
                    if log_lines <= args.max_log_lines:
                        print(f"RX {text}", flush=True)
                    elif not log_suppressed:
                        print("RX further logs omitted (continuing to count frames/replies)", flush=True)
                        log_suppressed = True
                if isinstance(value, dict) and value.get("t") == "ok":
                    acknowledgements.add(value.get("cmd"))
            if len(buffer) > 8192:
                buffer.clear()

    io_error = None
    try:
        send("get")
        receive(args.seconds)
        if args.diag:
            send("diag")
            receive(args.seconds)
        send("sub")
        receive(args.seconds)
        send("unsub")
        receive(0.5)
    except (serial.SerialException, OSError) as error:
        io_error = error
    finally:
        connection.close()
    print(f"SUMMARY get_ack={'get' in acknowledgements} sub_ack={'sub' in acknowledgements} telemetry_frames={telemetry}")
    if sensor_failure:
        print(f"DIAGNOSIS BMI088 initialization/ID failure; reboot_count={reboots}. Check IMU wiring and flash the non-fatal diagnostic firmware.")
    if io_error is not None:
        parser.exit(1, f"USB I/O failed: {io_error}\n")
    if not {"get", "sub"}.issubset(acknowledgements) or telemetry == 0:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
