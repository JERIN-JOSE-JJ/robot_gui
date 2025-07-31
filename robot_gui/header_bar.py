import gi
import os
import subprocess
from datetime import datetime

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GLib

ICON_DIR = os.path.join(os.path.dirname(__file__), 'assets', 'icons')


class CustomHeaderBarBox(Gtk.Box):
    def __init__(self):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL)
        self.set_name("custom-header")
        self.set_margin_top(10)
        self.set_margin_bottom(10)
        self.set_margin_start(10)
        self.set_margin_end(10)
        self.set_spacing(10)

        # Left → Date Label
        self.date_label = Gtk.Label(label=self.get_date())
        self.date_label.set_name("date-label")
        self.date_label.set_halign(Gtk.Align.START)
        self.pack_start(self.date_label, False, False, 0)

        # Center → Time Label
        self.time_label = Gtk.Label(label=self.get_time())
        self.time_label.set_name("time-label")
        self.time_label.set_halign(Gtk.Align.CENTER)
        self.time_label.set_hexpand(True)
        self.pack_start(self.time_label, True, True, 0)

        # Right → Battery and WiFi icons
        self.wifi_icon = Gtk.Image()
        self.battery_icon = Gtk.Image()

        icon_box = Gtk.Box(spacing=10)
        icon_box.pack_start(self.wifi_icon, False, False, 0)
        icon_box.pack_start(self.battery_icon, False, False, 0)
        icon_box.set_halign(Gtk.Align.END)
        self.pack_start(icon_box, False, False, 0)

        # Start timers
        self.update_time()
        self.update_status_icons()
        GLib.timeout_add_seconds(1, self.update_time)
        GLib.timeout_add_seconds(30, self.update_status_icons)

    def get_time(self):
        return datetime.now().strftime("%H:%M:%S")

    def get_date(self):
        return datetime.now().strftime("%B %d, %Y")

    def update_time(self):
        self.time_label.set_text(self.get_time())
        self.date_label.set_text(self.get_date())
        return True

    def update_status_icons(self):
        self.update_battery_icon()
        self.update_wifi_icon()
        return True

    def update_battery_icon(self):
        try:
            with open("/sys/class/power_supply/BAT0/capacity") as f:
                level = int(f.read().strip())
            with open("/sys/class/power_supply/BAT0/status") as f:
                status = f.read().strip().lower()

            if status == "charging":
                icon = "battery_charging.svg"
            elif level >= 80:
                icon = "battery_full.svg"
            elif level >= 40:
                icon = "battery_medium.svg"
            else:
                icon = "battery_low.svg"
        except Exception:
            icon = "battery_full.svg"

        self.battery_icon.set_from_file(os.path.join(ICON_DIR, icon))

    def update_wifi_icon(self):
        try:
            output = subprocess.check_output(
                ["nmcli", "-t", "-f", "active,ssid,signal", "dev", "wifi"],
                text=True
            ).splitlines()

            signal = 0
            for line in output:
                parts = line.strip().split(":")
                if parts[0] == "yes":
                    signal = int(parts[2])
                    break

            if signal >= 70:
                icon = "wifi_full.svg"
            elif signal >= 40:
                icon = "wifi_medium.svg"
            elif signal > 0:
                icon = "wifi_low.svg"
            else:
                icon = "wifi_none.svg"
        except Exception:
            icon = "wifi_none.svg"

        self.wifi_icon.set_from_file(os.path.join(ICON_DIR, icon))
