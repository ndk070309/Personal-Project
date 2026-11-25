"""Receive CSV files exported by the Arduino Nano over USB Serial."""

import re
import sys
import time
from pathlib import Path

import serial

# Change this to the Arduino's port in Windows Device Manager / Arduino IDE.
SERIAL_PORT = "COM4"
BAUD_RATE = 9600
OUTPUT_DIR = Path("sd_export")
READ_TIMEOUT_SECONDS = 1
NO_DATA_TIMEOUT_SECONDS = 60

FILE_MARKER = re.compile(r"^===FILE:(.+)===$")


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    saved_files = []
    current_file = None
    output_handle = None
    export_started = False
    last_activity = time.monotonic()

    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT_SECONDS) as ser:
            # Opening a serial port often resets an Arduino Nano.
            time.sleep(3)
            ser.reset_input_buffer()
            ser.write(b"EXPORT\n")
            ser.flush()
            print("EXPORT command sent. Waiting for the Arduino...\n")

            while True:
                raw = ser.readline()
                if not raw:
                    if time.monotonic() - last_activity > NO_DATA_TIMEOUT_SECONDS:
                        raise TimeoutError(
                            "No serial data received for 60 seconds. "
                            "Check the COM port, baud rate, SD card, and Arduino output."
                        )
                    continue

                last_activity = time.monotonic()
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")

                if line == "===EXPORT_START===":
                    export_started = True
                    print("Export started.")
                    continue

                if not export_started:
                    # Ignore Arduino startup/debug messages before the export marker.
                    continue

                match = FILE_MARKER.match(line)
                if match:
                    if output_handle:
                        output_handle.close()
                    # Use only the filename, never a path supplied over Serial.
                    filename = Path(match.group(1)).name
                    if not filename.lower().endswith(".csv"):
                        current_file = None
                        output_handle = None
                        print(f"Skipping unexpected file name: {filename}")
                        continue
                    current_file = OUTPUT_DIR / filename
                    output_handle = current_file.open(
                        "w", encoding="utf-8", newline=""
                    )
                    saved_files.append(current_file)
                    print(f"Receiving {filename}...")
                    continue

                if line == "===FILE_END===":
                    if output_handle:
                        output_handle.close()
                        output_handle = None
                    current_file = None
                    continue

                if line == "===EXPORT_FINISHED===":
                    if output_handle:
                        output_handle.close()
                        output_handle = None
                    break

                if line.startswith("===EXPORT_ERROR:"):
                    print(f"Arduino reported: {line}")
                    continue

                if line.startswith("===EXPORT_FILE_COUNT:"):
                    print(line)
                    continue

                if output_handle is not None:
                    output_handle.write(line + "\n")

    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        print("Close Arduino Serial Monitor/Plotter and check SERIAL_PORT.", file=sys.stderr)
        return 1
    except (TimeoutError, OSError) as exc:
        print(f"Export failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if output_handle:
            output_handle.close()

    if not export_started:
        print("The Arduino did not send the export start marker.", file=sys.stderr)
        return 1

    print(f"\nExport complete. Saved {len(saved_files)} CSV file(s) in: {OUTPUT_DIR.resolve()}")
    for path in saved_files:
        print(f"  - {path.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
