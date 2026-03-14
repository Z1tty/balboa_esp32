#!/usr/bin/env python3
"""
Capture raw serial output from ESP32 to file for offline analysis.
Usage: python3 capture_raw.py [port] [duration_sec]
"""

import serial
import serial.tools.list_ports
import sys
import time
import datetime
import os

BAUD = 115200
DEFAULT_DURATION = 60  # seconds

def find_esp32_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if "CP210" in (p.description or "") or "CH340" in (p.description or "") or "SLAB" in (p.description or "") or "usbserial" in (p.device or ""):
            return p.device
    if ports:
        return ports[0].device
    return None

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_esp32_port()
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_DURATION

    if not port:
        print("ERROR: ESP32 port not found. Specify manually: python3 capture_raw.py /dev/tty.usbserial-XXX")
        sys.exit(1)

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = "/private/tmp/claude-504/balboa_logs"
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, f"raw_capture_{ts}.log")

    print(f"Port:     {port}")
    print(f"Duration: {duration} s")
    print(f"Log file: {log_file}")
    print()

    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR opening port: {e}")
        sys.exit(1)

    time.sleep(1.5)  # let ESP32 boot messages flush
    ser.reset_input_buffer()

    start = time.time()
    lines_written = 0

    print(f"=== RECORDING for {duration} s — press Jets1 and Lights on panel NOW ===")
    print()

    with open(log_file, "w") as f:
        f.write(f"# capture start: {datetime.datetime.now().isoformat()}\n")
        f.write(f"# port: {port}\n\n")

        while time.time() - start < duration:
            line = ser.readline()
            if line:
                elapsed = time.time() - start
                try:
                    decoded = line.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    decoded = repr(line)
                entry = f"[{elapsed:7.3f}] {decoded}"
                f.write(entry + "\n")
                f.flush()
                print(entry)
                lines_written += 1

        remaining = duration - (time.time() - start)
        # drain for up to 1 more second
        drain_end = time.time() + 1.0
        while time.time() < drain_end:
            line = ser.readline()
            if line:
                elapsed = time.time() - start
                try:
                    decoded = line.decode("utf-8", errors="replace").rstrip()
                except Exception:
                    decoded = repr(line)
                entry = f"[{elapsed:7.3f}] {decoded}"
                f.write(entry + "\n")
                f.flush()
                print(entry)
                lines_written += 1

        f.write(f"\n# capture end: {datetime.datetime.now().isoformat()}, {lines_written} lines\n")

    ser.close()
    print()
    print(f"=== DONE — {lines_written} lines saved to {log_file} ===")

if __name__ == "__main__":
    main()
