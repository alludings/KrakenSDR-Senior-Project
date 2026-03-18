import asyncio
import json
import struct
import time
from collections import deque
from typing import Optional

import requests

from bluez_peripheral.advert import Advertisement
from bluez_peripheral.agent import NoIoAgent
from bluez_peripheral.gatt.service import Service
from bluez_peripheral.gatt.characteristic import Characteristic, CharacteristicFlags
from bluez_peripheral.util import get_message_bus
from bluez_peripheral.uuid16 import UUID16


# -----------------------------
# Config
# -----------------------------
KRAKEN_SETTINGS_URL = "http://127.0.0.1:8042/settings"

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
TRACKING_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
CONTROL_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"

DEVICE_NAME = "KrakenBridge"
CONFIDENCE_MIN = 40
PUBLISH_HZ = 5.0


# -----------------------------
# Shared state
# -----------------------------
class BridgeState:
    def __init__(self) -> None:
        self.latest_packet: bytes = b"\x00" * 9
        self.current_bearing: Optional[float] = None
        self.current_confidence: int = 0
        self.current_power: float = 0.0
        self.last_update_ts: int = 0
        self.bearing_history: deque[float] = deque(maxlen=5)


state = BridgeState()


# -----------------------------
# Kraken helpers
# -----------------------------
def get_kraken_settings() -> dict:
    r = requests.get(KRAKEN_SETTINGS_URL, timeout=2)
    r.raise_for_status()
    return r.json()


def post_kraken_settings(updated_settings: dict) -> None:
    r = requests.post(KRAKEN_SETTINGS_URL, json=updated_settings, timeout=2)
    r.raise_for_status()


def set_center_frequency(freq_hz: int) -> None:
    settings = get_kraken_settings()

    # Adjust this key name if your settings JSON uses a different field.
    settings["center_freq"] = freq_hz

    post_kraken_settings(settings)


def trigger_recalibration() -> None:
    settings = get_kraken_settings()

    # Adjust this to match the real Kraken settings key you decide to use.
    settings["recalibrate"] = True

    post_kraken_settings(settings)


def read_doa_result() -> Optional[dict]:
    """
    Replace this with your real Kraken data source.

    Options:
    1. Read a local file written by the Kraken app
    2. Read from a socket/IPC stream
    3. Lightly modify Kraken code to write a tiny JSON status file

    Expected return format:
    {
        "bearing": 127.4,
        "confidence": 83,
        "power": -41.2,
        "timestamp": 1710700000
    }
    """

    # Placeholder sample data
    now = int(time.time())
    fake_bearing = (now * 7) % 360
    return {
        "bearing": float(fake_bearing),
        "confidence": 75,
        "power": -38.5,
        "timestamp": now,
    }


# -----------------------------
# Filtering / packing
# -----------------------------
def smooth_bearing(new_bearing: float) -> float:
    state.bearing_history.append(new_bearing)
    return sum(state.bearing_history) / len(state.bearing_history)


def build_tracking_packet(
    bearing_deg: float,
    confidence: int,
    power_dbm: float,
    timestamp_s: int,
) -> bytes:
    """
    Compact binary payload:
      uint16 bearing_x10
      uint8  confidence
      int16  power_x10
      uint32 timestamp

    Total = 2 + 1 + 2 + 4 = 9 bytes
    """
    bearing_x10 = max(0, min(3599, int(round(bearing_deg * 10))))
    confidence_u8 = max(0, min(100, int(confidence)))
    power_x10 = int(round(power_dbm * 10))

    return struct.pack(">HbhI", bearing_x10, confidence_u8, power_x10, timestamp_s)


async def tracking_loop() -> None:
    interval = 1.0 / PUBLISH_HZ

    while True:
        try:
            doa = read_doa_result()

            if doa and doa["confidence"] >= CONFIDENCE_MIN:
                smoothed = smooth_bearing(doa["bearing"])

                state.current_bearing = smoothed
                state.current_confidence = int(doa["confidence"])
                state.current_power = float(doa["power"])
                state.last_update_ts = int(doa["timestamp"])

                state.latest_packet = build_tracking_packet(
                    bearing_deg=smoothed,
                    confidence=state.current_confidence,
                    power_dbm=state.current_power,
                    timestamp_s=state.last_update_ts,
                )

                print(
                    f"[TRACK] bearing={smoothed:.1f} "
                    f"conf={state.current_confidence} "
                    f"power={state.current_power:.1f}"
                )
            else:
                print("[TRACK] no valid lock")

        except Exception as e:
            print(f"[TRACK] error: {e}")

        await asyncio.sleep(interval)


# -----------------------------
# BLE GATT
# -----------------------------
class TrackingCharacteristic(Characteristic):
    def __init__(self, service: Service):
        super().__init__(
            service=service,
            uuid=TRACKING_CHAR_UUID,
            flags=[
                CharacteristicFlags.READ,
                CharacteristicFlags.NOTIFY,
            ],
        )

    async def read_value(self) -> bytes:
        return state.latest_packet


class ControlCharacteristic(Characteristic):
    def __init__(self, service: Service):
        super().__init__(
            service=service,
            uuid=CONTROL_CHAR_UUID,
            flags=[
                CharacteristicFlags.WRITE,
                CharacteristicFlags.WRITE_WITHOUT_RESPONSE,
            ],
        )

    async def write_value(self, value: bytes, options: dict) -> None:
        try:
            cmd = json.loads(value.decode("utf-8"))
            print(f"[BLE CMD] {cmd}")

            action = cmd.get("cmd")

            if action == "set_freq":
                freq_hz = int(cmd["hz"])
                set_center_frequency(freq_hz)
                print(f"[BLE CMD] center frequency -> {freq_hz}")

            elif action == "recalibrate":
                trigger_recalibration()
                print("[BLE CMD] recalibration requested")

            elif action == "start":
                print("[BLE CMD] start requested")
                # Add your own runtime enable logic here.

            elif action == "stop":
                print("[BLE CMD] stop requested")
                # Add your own runtime disable logic here.

            else:
                print(f"[BLE CMD] unknown command: {action}")

        except Exception as e:
            print(f"[BLE CMD] error: {e}")


class KrakenService(Service):
    def __init__(self):
        super().__init__(SERVICE_UUID, True)
        self.tracking = TrackingCharacteristic(self)
        self.control = ControlCharacteristic(self)


# -----------------------------
# Main
# -----------------------------
async def main() -> None:
    bus = await get_message_bus()

    service = KrakenService()

    await service.register(bus)
    await NoIoAgent().register(bus)

    advert = Advertisement(
        local_name=DEVICE_NAME,
        service_uuids=[SERVICE_UUID],
        appearance=0,
        timeout=0,
    )
    await advert.register(bus)

    print(f"[BLE] advertising as {DEVICE_NAME}")
    print("[BLE] tracking characteristic:", TRACKING_CHAR_UUID)
    print("[BLE] control characteristic:", CONTROL_CHAR_UUID)

    await tracking_loop()


if __name__ == "__main__":
    asyncio.run(main())