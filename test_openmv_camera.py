#!/usr/bin/env python3
"""
Test OpenMV Cam via USB serial - tự upload script và stream JPEG qua REPL.
Không cần OpenMV IDE.

Usage:
    python3 test_openmv_camera.py [--port /dev/ttyACM0]
"""

import argparse
import serial
import struct
import time
import sys
import cv2
import numpy as np

PORT = "/dev/ttyACM0"
BAUD = 115200

STREAM_SCRIPT = b"""\
import sensor, image, ustruct, pyb
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
usb = pyb.USB_VCP()
usb.setinterrupt(-1)
while True:
    img = sensor.snapshot()
    buf = img.compress(quality=50)
    sz = len(buf)
    usb.write(ustruct.pack('<I', sz))
    usb.write(buf)
"""


def enter_raw_repl(ser: serial.Serial) -> bool:
    """Interrupt running script and enter MicroPython raw REPL."""
    # Ctrl+C twice to stop any running script
    ser.write(b'\x03\x03')
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Ctrl+A to enter raw REPL
    ser.write(b'\x01')
    time.sleep(0.2)
    resp = ser.read(ser.in_waiting or 1)
    if b'raw REPL' not in resp and b'>' not in resp:
        # try once more
        ser.write(b'\x01')
        time.sleep(0.5)
        resp += ser.read(ser.in_waiting or 1)

    if b'raw REPL' in resp or b'>' in resp:
        print("[OK]   Entered raw REPL")
        return True
    print(f"[WARN] Unexpected raw REPL response: {resp!r}")
    return True  # continue anyway


def run_script_raw_repl(ser: serial.Serial, script: bytes) -> bool:
    """Send script via raw REPL and start execution."""
    ser.write(script)
    ser.write(b'\x04')  # Ctrl+D = execute
    time.sleep(0.5)

    # Read response: should start with b'OK'
    resp = b''
    deadline = time.time() + 3.0
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            resp += chunk
            if b'OK' in resp:
                break
        else:
            time.sleep(0.05)

    if b'OK' in resp:
        print("[OK]   Script uploaded and running")
        return True
    elif b'Error' in resp or b'Traceback' in resp:
        print(f"[FAIL] Script error: {resp.decode('utf-8', errors='replace')}")
        return False
    else:
        print(f"[WARN] Unexpected response: {resp!r}, continuing anyway")
        return True


def stream_frames(ser: serial.Serial, num_frames: int, show: bool):
    """Receive and display JPEG frames from the running script."""
    print(f"[INFO] Waiting for first frame...")

    if show:
        cv2.namedWindow("OpenMV Camera", cv2.WINDOW_NORMAL)

    frame_count = 0
    fail_count  = 0
    t0 = time.time()

    try:
        while frame_count < num_frames:
            # 4-byte little-endian size header
            header = ser.read(4)
            if len(header) < 4:
                fail_count += 1
                if fail_count >= 5:
                    print("[FAIL] Too many timeouts - check if sensor.reset() succeeded on camera")
                    break
                continue

            size = struct.unpack('<I', header)[0]
            if size == 0 or size > 300_000:
                # Likely stale REPL output, flush and retry
                ser.reset_input_buffer()
                fail_count += 1
                continue

            # Read JPEG payload
            data = b''
            remaining = size
            while remaining > 0:
                chunk = ser.read(remaining)
                if not chunk:
                    break
                data += chunk
                remaining -= len(chunk)

            if len(data) != size:
                fail_count += 1
                continue

            arr   = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                fail_count += 1
                continue

            frame_count += 1
            fps = frame_count / (time.time() - t0)
            print(f"[OK]   Frame {frame_count:3d}: {frame.shape[1]}x{frame.shape[0]}  "
                  f"{size:6d} bytes  {fps:.1f} fps")

            if show:
                cv2.imshow("OpenMV Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("[INFO] Quit by user")
                    break

    finally:
        if show:
            cv2.destroyAllWindows()

    print(f"\n[RESULT] {frame_count} frames OK, {fail_count} errors")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",       default=PORT,  help="Serial port")
    parser.add_argument("--baud",       default=BAUD,  type=int)
    parser.add_argument("--frames",     default=100,   type=int)
    parser.add_argument("--no-display", action="store_true")
    args = parser.parse_args()

    print("=" * 60)
    print("  OpenMV Camera Test  (auto-upload via REPL)")
    print("=" * 60)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=3.0)
    except serial.SerialException as e:
        print(f"[FAIL] Cannot open {args.port}: {e}")
        print("       Try: sudo chmod 666 /dev/ttyACM0")
        sys.exit(1)

    print(f"[OK]   Opened {args.port}")

    with ser:
        if not enter_raw_repl(ser):
            sys.exit(1)
        if not run_script_raw_repl(ser, STREAM_SCRIPT):
            sys.exit(1)
        # Flush any REPL echo before reading frames
        time.sleep(0.2)
        ser.reset_input_buffer()
        stream_frames(ser, args.frames, show=not args.no_display)


if __name__ == "__main__":
    main()
