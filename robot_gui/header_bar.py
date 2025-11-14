import gi
import os
import subprocess
import threading  # NEW: Import threading
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
        self.battery_icon = Gtk.Image()

        # NEW: Create a button for the WiFi icon
        self.wifi_icon = Gtk.Image()
        self.wifi_button = Gtk.Button(relief=Gtk.ReliefStyle.NONE)
        self.wifi_button.set_image(self.wifi_icon)
        self.wifi_button.connect("clicked", self.on_wifi_button_clicked)
        self.wifi_button.set_can_focus(False)

        # NEW: Create the Popover
        self.create_wifi_popover()

        # MODIFIED: Pack the new wifi_button
        icon_box = Gtk.Box(spacing=10)
        icon_box.pack_start(self.wifi_button, False, False, 0)
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
        # (Your original code... unchanged)
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
            icon = "battery_full.svg" # Default icon

        self.battery_icon.set_from_file(os.path.join(ICON_DIR, icon))

    def update_wifi_icon(self):
        # (Your original code... unchanged)
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

    # -----------------------------------------------------------------
    # NEW: WiFi Connection Functionality
    # -----------------------------------------------------------------

    def create_wifi_popover(self):
        """Creates the Gtk.Popover and its contents."""
        self.wifi_popover = Gtk.Popover.new(self.wifi_button)
        self.wifi_popover.set_position(Gtk.PositionType.BOTTOM)

        # This box will hold the list and the loading spinner
        self.popover_vbox = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.popover_vbox.set_border_width(10)

        self.wifi_list_box = Gtk.ListBox()
        self.wifi_list_box.set_selection_mode(Gtk.SelectionMode.NONE)
        self.wifi_list_box.connect("row-activated", self.on_wifi_row_activated)

        # A spinner to show while scanning
        self.wifi_spinner = Gtk.Spinner()
        self.popover_vbox.pack_start(self.wifi_spinner, True, True, 0)
        self.popover_vbox.pack_start(self.wifi_list_box, True, True, 0)

        self.wifi_popover.add(self.popover_vbox)
        self.popover_vbox.show_all()

    def on_wifi_button_clicked(self, widget):
        """Called when the WiFi button is clicked."""
        self.wifi_popover.popup()
        # Show loading state
        self.wifi_list_box.hide()
        self.wifi_spinner.show()
        self.wifi_spinner.start()

        # Run the scan in a separate thread to avoid freezing the GUI
        thread = threading.Thread(target=self.scan_and_update_wifi_list)
        thread.daemon = True
        thread.start()

    def scan_and_update_wifi_list(self):
        """
        [WORKER THREAD] Scans for WiFi networks using nmcli.
        """
        networks = []
        try:
            # Rescan for fresh results
            subprocess.run(["nmcli", "dev", "wifi", "rescan"], timeout=5)

            # Get the list
            output = subprocess.check_output(
                ["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list"],
                text=True
            ).splitlines()

            seen_ssids = set()
            for line in output:
                if not line.strip():
                    continue
                parts = line.strip().split(":")
                if len(parts) >= 3:
                    ssid = parts[0]
                    if ssid and ssid not in seen_ssids:
                        networks.append({
                            "ssid": ssid,
                            "signal": int(parts[1]),
                            "security": ":".join(parts[2:]) # Handle SSIDs with colons
                        })
                        seen_ssids.add(ssid)
            
            # Sort by signal strength
            networks.sort(key=lambda x: x['signal'], reverse=True)

        except Exception as e:
            print(f"Error scanning WiFi: {e}")
        
        # Pass the results back to the main GUI thread
        GLib.idle_add(self.populate_wifi_list, networks)

    def populate_wifi_list(self, networks):
        """
        [GUI THREAD] Clears the old list and adds the new networks.
        """
        # Clear old list
        for child in self.wifi_list_box.get_children():
            child.destroy()

        if not networks:
            row = Gtk.ListBoxRow()
            row.add(Gtk.Label(label="No networks found.", xalign=0.5))
            self.wifi_list_box.add(row)
        else:
            for net in networks:
                row = Gtk.ListBoxRow()
                row.set_data("network", net) # Attach network data to the row
                
                hbox = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
                hbox.set_border_width(5)
                # You could add a signal icon here
                hbox.pack_start(Gtk.Label(label=net['ssid'], xalign=0), True, True, 0)
                if net['security'] != "--" and net['security']:
                    lock_icon = Gtk.Image.new_from_icon_name("network-wireless-encrypted-symbolic", Gtk.IconSize.BUTTON)
                    hbox.pack_start(lock_icon, False, False, 0)
                row.add(hbox)
                self.wifi_list_box.add(row)

        # Show results
        self.wifi_spinner.stop()
        self.wifi_spinner.hide()
        self.wifi_list_box.show_all()

    def on_wifi_row_activated(self, listbox, row):
        """
        [GUI THREAD] Called when a user clicks a network in the list.
        """
        network = row.get_data("network")
        ssid = network['ssid']
        security = network['security']

        # If security is "--" (open), just connect
        if not security or security == "--":
            self.wifi_popover.popdown()
            self.connect_to_wifi(ssid)
        else:
            # Need a password
            self.ask_for_password(network)

    def ask_for_password(self, network):
        """
        [GUI THREAD] Shows a dialog to ask for the WiFi password.
        """
        ssid = network['ssid']
        dialog = Gtk.Dialog(
            title=f"Connect to {ssid}",
            parent=self.get_toplevel(),
            flags=Gtk.DialogFlags.MODAL | Gtk.DialogFlags.DESTROY_WITH_PARENT
        )
        dialog.add_buttons(
            Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL,
            Gtk.STOCK_CONNECT, Gtk.ResponseType.OK
        )
        dialog.set_default_response(Gtk.ResponseType.OK)

        content_area = dialog.get_content_area()
        content_area.set_border_width(10)
        content_area.set_spacing(10)
        
        content_area.add(Gtk.Label(label=f"Password for {ssid}:"))
        
        password_entry = Gtk.Entry()
        password_entry.set_visibility(False) # Hides password
        password_entry.set_activates_default(True) # Pressing Enter clicks "Connect"
        content_area.add(password_entry)
        
        dialog.show_all()
        response = dialog.run()
        password = password_entry.get_text()
        dialog.destroy()

        if response == Gtk.ResponseType.OK and password:
            self.wifi_popover.popdown()
            self.connect_to_wifi(ssid, password)

    def connect_to_wifi(self, ssid, password=None):
        """
        [GUI THREAD] Starts a new thread to handle the connection.
        """
        # Show a quick "Connecting..." message (optional)
        print(f"Attempting to connect to {ssid}...")
        
        # Run connection in a thread to avoid freezing GUI
        thread = threading.Thread(
            target=self._do_connect_thread,
            args=(ssid, password)
        )
        thread.daemon = True
        thread.start()

    def _do_connect_thread(self, ssid, password):
        """
        [WORKER THREAD] Runs the actual nmcli connect command.
        """
        try:
            cmd = ["nmcli", "dev", "wifi", "connect", ssid]
            if password:
                cmd.extend(["password", password])
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )
            
            if result.returncode == 0:
                message = f"Successfully connected to {ssid}"
                GLib.idle_add(self.show_connection_status, True, message)
            else:
                message = f"Failed to connect: {result.stderr.strip()}"
                GLib.idle_add(self.show_connection_status, False, message)

        except Exception as e:
            message = f"Error connecting: {e}"
            GLib.idle_add(self.show_connection_status, False, message)
        
        # Refresh the icon after the attempt
        GLib.idle_add(self.update_status_icons)

    def show_connection_status(self, success, message):
        """
        [GUI THREAD] Shows a dialog with the connection result.
        """
        if success:
            msg_type = Gtk.MessageType.INFO
            title = "Connection Successful"
        else:
            msg_type = Gtk.MessageType.ERROR
            title = "Connection Failed"

        dialog = Gtk.MessageDialog(
            transient_for=self.get_toplevel(),
            flags=Gtk.DialogFlags.MODAL,
            message_type=msg_type,
            buttons=Gtk.ButtonsType.OK,
            text=title,
        )
        dialog.format_secondary_text(message)
        dialog.run()
        dialog.destroy()