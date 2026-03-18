#!/usr/bin/env python3
import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
import struct
import time
from gi.repository import GLib

# ----UUIDs (must match Android) ----
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

mainloop = None


def find_adapter(bus):
    obj = bus.get_object(BLUEZ_SERVICE_NAME, "/")
    om = dbus.Interface(obj, DBUS_OM_IFACE)
    objects = om.GetManagedObjects()

    for path, ifaces in objects.items():
        if LE_ADVERTISING_MANAGER_IFACE in ifaces and GATT_MANAGER_IFACE in ifaces:
            return path
    raise RuntimeError("No BLE adapter found that supports advertising + GATT (is bluetooth enabled?)")


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
            raise dbus.exceptions.DBusException("org.freedesktop.DBus.Error.InvalidArgs", "Wrong interface")

        props = {
            "Type": self.ad_type,
            "ServiceUUIDs": dbus.Array(self.service_uuids, signature="s"),
            "LocalName": self.local_name,
            "IncludeTxPower": dbus.Boolean(self.include_tx_power),
        }
        return props

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
                "Characteristics": dbus.Array([c.get_path() for c in self.characteristics], signature="o"),
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
            raise dbus.exceptions.DBusException("org.freedesktop.DBus.Error.InvalidArgs", "Wrong interface")
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, options):
        return self.value

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}", out_signature="")
    def WriteValue(self, value, options):
        # Not needed for heading notify, but present for completeness
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
        self.heading = 0.0

    def set_heading(self, heading_deg: float):
        # 4 bytes little-endian float to match Android ByteBuffer LITTLE_ENDIAN .float
        payload = struct.pack("<f", float(heading_deg))
        self.value = dbus.Array([dbus.Byte(b) for b in payload], signature="y")

        # If a phone subscribed, push notification by firing PropertiesChanged
        if self.notifying:
            self._properties_changed()


def register_app_and_advert(bus, adapter_path, app, advert):
    service_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), GATT_MANAGER_IFACE)
    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), LE_ADVERTISING_MANAGER_IFACE)

    service_manager.RegisterApplication(app.get_path(), {}, reply_handler=lambda: print("GATT app registered"),
                                       error_handler=lambda e: print("Failed to register app:", e))
    ad_manager.RegisterAdvertisement(advert.get_path(), {}, reply_handler=lambda: print("Advertisement registered"),
                                    error_handler=lambda e: print("Failed to register advert:", e))


def main():
    global mainloop

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()

    adapter_path = find_adapter(bus)
    print("Using adapter:", adapter_path)

    # Build GATT app
    app = Application(bus)
    service = Service(bus, 0, SERVICE_UUID, primary=True)
    heading_char = HeadingCharacteristic(bus, 0, service)
    service.add_characteristic(heading_char)
    app.add_service(service)

    # Advertise
    advert = Advertisement(bus, 0)

    register_app_and_advert(bus, adapter_path, app, advert)

    # Periodic update (demo): rotate heading
    def tick():
        heading_char.heading = (heading_char.heading + 5.0) % 360.0
        heading_char.set_heading(heading_char.heading)
        return True  # keep running

    GLib.timeout_add(200, tick)  # 5 Hz (200 ms). Change to 50 for 20 Hz.

    mainloop = GLib.MainLoop()
    print("Pi BLE Compass Server running. Advertising as 'PiCompass'.")
    print("Service UUID:", SERVICE_UUID)
    print("Heading Char UUID:", HEADING_CHAR_UUID)
    mainloop.run()


if __name__ == "__main__":
    main()
