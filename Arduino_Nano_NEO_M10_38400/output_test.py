
import serial
import re
import csv
import time
from pathlib import Path

SERIAL_PORT = "COM4"
BAUD_RATE = 9600

OUTPUT_FILE = Path("sensor_data.csv")

# Arduino sends 19 CSV columns (18 commas).
CSV_COLUMNS = [
    "thoi_gian",
    "vi_do",
    "kinh_do",
    "cao_do_m",
    "toc_do_kmh",
    "ve_tinh",
    "pm1_ugm3",
    "pm25_ugm3",
    "pm10_ugm3",
    "mq2_adc",
    "mq2_ppm",
    "mq135_adc",
    "mq135_ppm",
    "aqi_uba",
    "tvoc_ppb",
    "eco2_ppm",
    "nhiet_do_c",
    "do_am_pct",
    "ens_trang_thai"
]

# Match the timestamp at the beginning of a sensor data row.
TIMESTAMP_PATTERN = re.compile(
    r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},"
)

def is_sensor_data(line):
    if not TIMESTAMP_PATTERN.match(line):
        return False

    return len(line.split(",")) == len(CSV_COLUMNS)


try:
    print(f"Connecting to {SERIAL_PORT}...")

    with serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=1
    ) as ser:

        time.sleep(3)
        ser.reset_input_buffer()

        file_exists = OUTPUT_FILE.exists() and OUTPUT_FILE.stat().st_size > 0

        with open(
            OUTPUT_FILE,
            "a",
            newline="",
            encoding="utf-8"
        ) as file:

            writer = csv.writer(file)

            if not file_exists:
                writer.writerow(CSV_COLUMNS)

            print("Recording sensor data...")
            print("Press Ctrl+C to stop.\n")

            saved_count = 0

            while True:
                raw = ser.readline()

                if not raw:
                    continue

                line = raw.decode(
                    "utf-8",
                    errors="replace"
                ).strip()

                # Ignore all non-sensor messages.
                if not is_sensor_data(line):
                    continue

                writer.writerow(line.split(","))
                file.flush()

                saved_count += 1

                print(f"Sensor record saved: {saved_count}")

except KeyboardInterrupt:
    print("\nRecording stopped.")

except serial.SerialException as e:
    print(f"Serial error: {e}")

except Exception as e:
    print(f"Error: {e}")
