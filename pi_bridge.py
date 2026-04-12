#!/usr/bin/env python3

import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
import json
import math
import smbus2
import struct
from collections import deque
from gi.repository import GLib

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
LIVE_DOA_PATH = "/home/pi/krakensdr_doa/_share/live_doa.json"

# Sensor I2C addresses
MMC5603_ADDR = 0x30
LSM6DSOX_ADDR = 0x6A

# Bridge behavior
CONFIDENCE_MIN = 40
UPDATE_MS = 200  # 5 Hz
SMOOTHING_WINDOW = 5

mainloop = None


# =========================================================
# SENSOR CLASSES
# =========================================================
class MMC5603:
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)

        # Replace these with your real calibration values
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        self.x_scale = 1.0
        self.y_scale = 1.0
        self.z_scale = 1.0

        self._init_sensor()

    def _init_sensor(self):
        # Keep this if it already works in your setup
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
        # Keep the register values that already work for you
        self.bus.write_byte_data(LSM6DSOX_ADDR, 0x10, 0x80)  # accel
        self.bus.write_byte_data(LSM6DSOX_ADDR, 0x11, 0x80)  # gyro

    @staticmethod
    def _twos_complement(val, bits):
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
# GLOBAL DEVICES / STATE
# =========================================================
mag_sensor = MMC5603(bus=1)
imu_sensor = LSM6DSOX(bus=1)
heading_history = deque(maxlen=SMOOTHING_WINDOW)


# =========================================================
# KRAKEN + IMU READERS
# =========================================================
def read_kraken_doa():
    try:
        with open(LIVE_DOA_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        print("[KRAKEN] read error:", e)
        return None


def read_imu():
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

    # Fusion: absolute signal heading = tablet heading + relative Kraken bearing
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
        # Android should decode this as LITTLE_ENDIAN float
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
