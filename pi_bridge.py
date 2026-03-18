import asyncio
import json
import math
import struct
import time
from collections import deque
from typing import Optional

import smbus2
from bluez_peripheral.advert import Advertisement
from bluez_peripheral.agent import NoIoAgent
from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import Characteristic, CharacteristicFlags
from bluez_peripheral.util import get_message_bus


# =========================================================
# CONFIG
# =========================================================
LIVE_DOA_PATH = "/home/pi/krakensdr_doa/_share/live_doa.json"

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
TRACK_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"

DEVICE_NAME = "KrakenBridge"
CONFIDENCE_MIN = 40
UPDATE_RATE_HZ = 5

MMC5603_ADDR = 0x30
LSM6DSOX_ADDR = 0x6A


# =========================================================
# SENSOR CLASSES
# =========================================================
class MMC5603:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)

        # Replace with your real calibration values
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        self.x_scale = 1.0
        self.y_scale = 1.0
        self.z_scale = 1.0

        self._init_sensor()

    def _init_sensor(self):
        # Example init; confirm with your actual working setup
        self.bus.write_byte_data(MMC5603_ADDR, 0x1B, 0x01)

    def read_raw(self):
        data = self.bus.read_i2c_block_data(MMC5603_ADDR, 0x00, 6)
        x = (data[0] << 8) | data[1]
        y = (data[2] << 8) | data[3]
        z = (data[4] << 8) | data[5]
        return x, y, z

    def read_calibrated(self):
        x, y, z = self.read_raw()
        x = (x - self.x_offset) * self.x_scale
        y = (y - self.y_offset) * self.y_scale
        z = (z - self.z_offset) * self.z_scale
        return x, y, z


class LSM6DSOX:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        self._init_sensor()

    def _init_sensor(self):
        # Example init; keep the values that already work for you
        self.bus.write_byte_data(LSM6DSOX_ADDR, 0x10, 0x80)  # accel
        self.bus.write_byte_data(LSM6DSOX_ADDR, 0x11, 0x80)  # gyro

    def _twos_complement(self, val, bits):
        if val & (1 << (bits - 1)):
            val -= 1 << bits
        return val

    def read_accel(self):
        data = self.bus.read_i2c_block_data(LSM6DSOX_ADDR, 0x28, 6)
        x = self._twos_complement((data[1] << 8) | data[0], 16)
        y = self._twos_complement((data[3] << 8) | data[2], 16)
        z = self._twos_complement((data[5] << 8) | data[4], 16)
        return x, y, z

    def read_gyro(self):
        data = self.bus.read_i2c_block_data(LSM6DSOX_ADDR, 0x22, 6)
        x = self._twos_complement((data[1] << 8) | data[0], 16)
        y = self._twos_complement((data[3] << 8) | data[2], 16)
        z = self._twos_complement((data[5] << 8) | data[4], 16)
        return x, y, z


# =========================================================
# ORIENTATION / FUSION HELPERS
# =========================================================
def tilt_compensated_heading(mag_x, mag_y, mag_z, accel_x, accel_y, accel_z):
    norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    if norm == 0:
        raise ValueError("Accelerometer norm is zero")

    ax = accel_x / norm
    ay = accel_y / norm
    az = accel_z / norm

    pitch = math.asin(-ax)
    roll = math.atan2(ay, az)

    # Use pitch and roll here
    xh = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
    yh = (
        mag_x * math.sin(roll) * math.sin(pitch)
        + mag_y * math.cos(roll)
        - mag_z * math.sin(roll) * math.cos(pitch)
    )

    heading = math.degrees(math.atan2(yh, xh))
    if heading < 0:
        heading += 360.0

    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)

    return heading, pitch_deg, roll_deg


def combine_heading(kraken_bearing_deg, imu_heading_deg):
    return (imu_heading_deg + kraken_bearing_deg) % 360.0


# =========================================================
# STATE
# =========================================================
class State:
    def __init__(self):
        self.packet = b"\x00" * 16
        self.history = deque(maxlen=5)


state = State()


# =========================================================
# INIT DEVICES
# =========================================================
mag_sensor = MMC5603(bus=1)
imu_sensor = LSM6DSOX(bus=1)


# =========================================================
# READ INPUTS
# =========================================================
def read_kraken_doa() -> Optional[dict]:
    try:
        with open(LIVE_DOA_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"[KRAKEN] read error: {e}")
        return None


def read_imu() -> Optional[dict]:
    try:
        mag_x, mag_y, mag_z = mag_sensor.read_calibrated()
        accel_x, accel_y, accel_z = imu_sensor.read_accel()

        heading, pitch, roll = tilt_compensated_heading(
            mag_x, mag_y, mag_z,
            accel_x, accel_y, accel_z
        )

        return {
            "heading": heading,
            "pitch": pitch,
            "roll": roll,
        }
    except Exception as e:
        print(f"[IMU] read error: {e}")
        return None


# =========================================================
# PIPELINE
# =========================================================
def smooth(angle):
    state.history.append(angle)
    return sum(state.history) / len(state.history)


def build_output(doa, imu):
    final_heading = combine_heading(doa["bearing"], imu["heading"])
    final_heading = smooth(final_heading)

    return {
        "final_heading": final_heading,
        "kraken_bearing": float(doa["bearing"]),
        "imu_heading": float(imu["heading"]),
        "confidence": int(doa["confidence"]),
        "power": float(doa["power"]),
        "pitch": float(imu["pitch"]),
        "roll": float(imu["roll"]),
        "timestamp": int(doa["timestamp"]),
    }


def build_packet(data):
    # uint16 final_heading_x10
    # uint8 confidence
    # int16 power_x10
    # int16 pitch_x10
    # int16 roll_x10
    # uint32 timestamp
    return struct.pack(
        ">HbhhhI",
        int(data["final_heading"] * 10),
        int(data["confidence"]),
        int(data["power"] * 10),
        int(data["pitch"] * 10),
        int(data["roll"] * 10),
        int(data["timestamp"]),
    )


# =========================================================
# BLE
# =========================================================
class TrackingCharacteristic(Characteristic):
    def __init__(self, service):
        super().__init__(
            service=service,
            uuid=TRACK_CHAR_UUID,
            flags=[
                CharacteristicFlags.READ,
                CharacteristicFlags.NOTIFY,
            ],
        )

    async def read_value(self):
        return state.packet


class KrakenService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)
        self.tracking = TrackingCharacteristic(self)


# =========================================================
# MAIN LOOP
# =========================================================
async def main_loop():
    interval = 1.0 / UPDATE_RATE_HZ

    while True:
        doa = read_kraken_doa()
        imu = read_imu()

        if doa and imu and doa["confidence"] >= CONFIDENCE_MIN:
            output = build_output(doa, imu)
            state.packet = build_packet(output)

            print(
                f"[OUT] final={output['final_heading']:.1f} "
                f"kraken={output['kraken_bearing']:.1f} "
                f"imu={output['imu_heading']:.1f} "
                f"conf={output['confidence']} "
                f"pitch={output['pitch']:.1f} "
                f"roll={output['roll']:.1f}"
            )
        else:
            print("[OUT] no valid signal or IMU data")

        await asyncio.sleep(interval)


async def main():
    bus = await get_message_bus()

    service = KrakenService()
    await service.register(bus)
    await NoIoAgent().register(bus)

    advert = Advertisement(
        local_name=DEVICE_NAME,
        service_uuids=[SERVICE_UUID],
    )
    await advert.register(bus)

    print(f"[BLE] Advertising as {DEVICE_NAME}")
    await main_loop()


if __name__ == "__main__":
    asyncio.run(main())
