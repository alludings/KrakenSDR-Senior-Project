#!/usr/bin/env python3

# BLE CLIENT
# MADE BY NATHAN HOANG
# Last edited: 4/22/26
# Last change: Rewritten to use Adafruit libraries 

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service

import json
import math
import struct
import os
from collections import deque

import board
import busio
import adafruit_mmc56x3
import adafruit_lsm6ds.lsm6dsox

from gi.repository import GLib
from gpiozero import Button

# =========================================================
# KILL BUTTON
# =========================================================
shutdown_button = Button(17, pull_up=True, hold_time=2)


def shutdown_pi():
    print("Shutdown triggered")
    os.system("sudo shutdown now")


shutdown_button.when_held = shutdown_pi()

# =========================================================
# BLE / BLUEZ CONSTANTS
# =========================================================
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
HEADING_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"

BLUEZ_SERVICE_NAME = "org.bluez"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"

GATT_MANAGER_IFACE = "org.bluez.GattManager1"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"

GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
LE_ADVERTISEMENT_IFACE = "org.bluez.LEAdvertisement1"

# =========================================================
# APP CONFIG
# =========================================================
LIVE_DOA_PATH = "/home/krakenrf/krakensdr_doa/krakensdr_doa/_share/live_doa.json"

# Bridge behavior
CONFIDENCE_MIN = 0.40
UPDATE_MS = 200  # 5 Hz
SMOOTHING_WINDOW = 5

mainloop = None

# =========================================================
# SENSOR SETUP (Adafruit libraries)
# =========================================================
i2c = busio.I2C(board.SCL, board.SDA)

mag_sensor = adafruit_mmc56x3.MMC5603(i2c)
imu_sensor = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(i2c)

heading_history = deque(maxlen=SMOOTHING_WINDOW)

#calibration placeholders
MAG_X_OFFSET = 0.0
MAG_Y_OFFSET = 0.0
MAG_Z_OFFSET = 0.0
MAG_X_SCALE = 1.0
MAG_Y_SCALE = 1.0
MAG_Z_SCALE = 1.0

# =========================================================
# SENSOR / FUSION HELPERS
# =========================================================
def normalize_angle(angle_deg: float) -> float:
    return angle_deg % 360.0


def circular_mean_deg(values):
    if not values:
        return 0.0

    sin_sum = sum(math.sin(math.radians(v)) for v in values)
    cos_sum = sum(math.cos(math.radians(v)) for v in values)

    angle = math.degrees(math.atan2(sin_sum, cos_sum))
    return normalize_angle(angle)


def tilt_compensated_heading(mag_x, mag_y, mag_z, accel_x, accel_y, accel_z):
    norm = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    if norm == 0:
        raise ValueError("Accelerometer norm is zero")

    ax = accel_x / norm
    ay = accel_y / norm
    az = accel_z / norm

    pitch = math.asin(-ax)
    roll = math.atan2(ay, az)

    xh = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
    yh = (
        mag_x * math.sin(roll) * math.sin(pitch)
        + mag_y * math.cos(roll)
        - mag_z * math.sin(roll) * math.cos(pitch)
    )

    heading = math.degrees(math.atan2(yh, xh))
    heading = normalize_angle(heading)

    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)
    return heading, pitch_deg, roll_deg


# =========================================================
# KRAKEN + IMU READERS
# =========================================================
def read_kraken_doa():
    try:
        with open(LIVE_DOA_PATH, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return None
    except Exception as e:
        print("[KRAKEN] read error:", e)
        return None


def read_imu():
    try:
        mag_x_raw, mag_y_raw, mag_z_raw = mag_sensor.magnetic
        accel_x_raw, accel_y_raw, accel_z_raw = imu_sensor.acceleration

        # Magnetometer remap:
        # new x = old y
        # new y = old x
        # new z = -old z
        mag_x = mag_y_raw
        mag_y = mag_x_raw
        mag_z = -mag_z_raw

        # Accelerometer remap:
        # new x = -old x
        # new y = old y
        # new z = -old z
        accel_x = -accel_x_raw
        accel_y = accel_y_raw
        accel_z = -accel_z_raw

        # Optional calibration after remap
        mag_x = (mag_x - MAG_X_OFFSET) * MAG_X_SCALE
        mag_y = (mag_y - MAG_Y_OFFSET) * MAG_Y_SCALE
        mag_z = (mag_z - MAG_Z_OFFSET) * MAG_Z_SCALE

        heading, pitch, roll = tilt_compensated_heading(
            mag_x, mag_y, mag_z,
            accel_x, accel_y, accel_z
        )

        print(
            f"[RAW] mag=({mag_x:.2f}, {mag_y:.2f}, {mag_z:.2f}) "
            f"accel=({accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f})"
        )

        return {
            "heading": heading,
            "pitch": pitch,
            "roll": roll,
        }
    except Exception as e:
        print("[IMU] read error:", e)
        return None


def compute_final_heading():
    imu = read_imu()
    if imu is None:
        print("[OUT] no IMU")
        return None

    imu_heading = float(imu["heading"])
    doa = read_kraken_doa()

    # Fallback: no Kraken JSON yet, so use IMU only
    if not doa:
        print(f"[OUT] IMU only: {imu_heading:.1f}")
        return imu_heading

    confidence = float(doa.get("confidence", 0.0))
    if confidence < CONFIDENCE_MIN:
        print(f"[OUT] low confidence ({confidence:.2f}), using IMU only: {imu_heading:.1f}")
        return imu_heading

    kraken_bearing = float(doa["bearing"])

    # Fusion: absolute signal heading = imu heading + kraken relative bearing
    final_heading = normalize_angle(imu_heading + kraken_bearing)

    heading_history.append(final_heading)
    final_heading_smoothed = circular_mean_deg(list(heading_history))

    print(
        f"[OUT] imu={imu_heading:.1f} "
        f"kraken={kraken_bearing:.1f} "
        f"final={final_heading_smoothed:.1f} "
        f"conf={confidence:.2f} "
        f"pitch={imu['pitch']:.1f} "
        f"roll={imu['roll']:.1f}"
    )

    return final_heading_smoothed


# =========================================================
# BLE / GATT HELPERS
# =========================================================
def find_adapter(bus):
    obj = bus.get_object(BLUEZ_SERVICE_NAME, "/")
    om = dbus.Interface(obj, DBUS_OM_IFACE)
    objects = om.GetManagedObjects()

    for path, ifaces in objects.items():
        if LE_ADVERTISING_MANAGER_IFACE in ifaces and GATT_MANAGER_IFACE in ifaces:
            return path
    raise RuntimeError("No BLE adapter found that supports advertising + GATT")


class Advertisement(dbus.service.Object):
    PATH_BASE = "/com/mbcompass/advertisement"

    def __init__(self, bus, index):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = "peripheral"
        self.service_uuids = [SERVICE_UUID]
        self.local_name = "PiCompass"
        self.include_tx_power = True
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise dbus.exceptions.DBusException(
                "org.freedesktop.DBus.Error.InvalidArgs",
                "Wrong interface"
            )

        return {
            "Type": self.ad_type,
            "ServiceUUIDs": dbus.Array(self.service_uuids, signature="s"),
            "LocalName": self.local_name,
            "IncludeTxPower": dbus.Boolean(self.include_tx_power),
        }

    @dbus.service.method(LE_ADVERTISEMENT_IFACE, in_signature="", out_signature="")
    def Release(self):
        print("Advertisement released")


class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = "/com/mbcompass/app"
        self.bus = bus
        self.services = []
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for chrc in service.characteristics:
                response[chrc.get_path()] = chrc.get_properties()
        return response


class Service(dbus.service.Object):
    PATH_BASE = "/com/mbcompass/service"

    def __init__(self, bus, index, uuid, primary=True):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, chrc):
        self.characteristics.append(chrc)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": dbus.Boolean(self.primary),
                "Characteristics": dbus.Array(
                    [c.get_path() for c in self.characteristics],
                    signature="o"
                ),
            }
        }


class Characteristic(dbus.service.Object):
    PATH_BASE = "/com/mbcompass/char"

    def __init__(self, bus, index, uuid, flags, service):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.service = service
        self.notifying = False
        self.value = dbus.Array([], signature="y")
        super().__init__(bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": dbus.Array(self.flags, signature="s"),
                "Value": self.value,
            }
        }

    def _properties_changed(self):
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {"Value": self.value},
            []
        )

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise dbus.exceptions.DBusException(
                "org.freedesktop.DBus.Error.InvalidArgs",
                "Wrong interface"
            )
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        self.value = dbus.Array(value, signature="y")
        self._properties_changed()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="", out_signature="")
    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True
        print("Phone subscribed to notifications")

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="", out_signature="")
    def StopNotify(self):
        self.notifying = False
        print("Phone unsubscribed from notifications")

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class HeadingCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        super().__init__(
            bus=bus,
            index=index,
            uuid=HEADING_CHAR_UUID,
            flags=["read", "notify"],
            service=service
        )

    def set_heading(self, heading_deg: float):
        heading_deg = (heading_deg + 180.0) % 360.0
        payload = struct.pack("<f", float(heading_deg))
        self.value = dbus.Array([dbus.Byte(b) for b in payload], signature="y")

        if self.notifying:
            self._properties_changed()


def register_app_and_advert(bus, adapter_path, app, advert):
    service_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        GATT_MANAGER_IFACE
    )
    ad_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        LE_ADVERTISING_MANAGER_IFACE
    )

    service_manager.RegisterApplication(
        app.get_path(),
        {},
        reply_handler=lambda: print("GATT app registered"),
        error_handler=lambda e: print("Failed to register app:", e)
    )

    ad_manager.RegisterAdvertisement(
        advert.get_path(),
        {},
        reply_handler=lambda: print("Advertisement registered"),
        error_handler=lambda e: print("Failed to register advert:", e)
    )


# =========================================================
# MAIN
# =========================================================
def main():
    global mainloop

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = find_adapter(bus)
    print("Using adapter:", adapter_path)

    app = Application(bus)
    service = Service(bus, 0, SERVICE_UUID, primary=True)
    heading_char = HeadingCharacteristic(bus, 0, service)
    service.add_characteristic(heading_char)
    app.add_service(service)

    advert = Advertisement(bus, 0)
    register_app_and_advert(bus, adapter_path, app, advert)

    def tick():
        final_heading = compute_final_heading()
        if final_heading is not None:
            heading_char.set_heading(final_heading)
        return True

    GLib.timeout_add(UPDATE_MS, tick)

    mainloop = GLib.MainLoop()
    print("Pi BLE Compass Server running. Advertising as 'PiCompass'.")
    print("Service UUID:", SERVICE_UUID)
    print("Heading Char UUID:", HEADING_CHAR_UUID)
    mainloop.run()


if __name__ == "__main__":
    main()
