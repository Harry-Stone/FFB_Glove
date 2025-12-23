import serial
import struct
import time

def read_witmotion_angles(port="/dev/ttyUSB2", baud=9600, timeout=1.0):
    ser = serial.Serial(port, baud, timeout=timeout)

    try:
        while True:
            # Sync to header
            if ser.read(1) != b'\x55':
                continue

            pid = ser.read(1)
            if not pid:
                continue

            if pid[0] == 0x53:  # Euler angles
                body = ser.read(9)
                if len(body) != 9:
                    continue

                data = body[:8]
                checksum = body[8]

                # checksum validation
                total = (0x55 + 0x53 + sum(data)) & 0xFF
                if total != checksum:
                    continue

                raw_roll, raw_pitch, raw_yaw, _ = struct.unpack("<hhhh", data)

                roll  = raw_roll  * 180.0 / 32768.0
                pitch = raw_pitch * 180.0 / 32768.0
                yaw   = raw_yaw   * 180.0 / 32768.0

                yield roll, pitch, yaw
            else:
                # Skip other packet types
                ser.read(9)

    finally:
        ser.close()


if __name__ == "__main__":
    for r, p, y in read_witmotion_angles(port="/dev/ttyUSB2", baud=9600):
        print(f"Roll: {r:.2f}, Pitch: {p:.2f}, Yaw: {y:.2f}")
        time.sleep(0.05)
