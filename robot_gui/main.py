import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gio, Gdk

import rclpy
import threading
import os

from robot_gui.header_bar import CustomHeaderBarBox
from robot_gui.start_window import HomeScreen  # Import HomeScreen properly
from robot_gui.manual_window import ManualWindow
from robot_gui.autonomy_window import AutonomyWindow
from robot_gui.ros_publisher import ROSPublisher
from robot_gui.camera import CameraWindow
from robot_gui.chat_ai import ChatWindow
from robot_gui.voice_window import VoiceWindow

def load_css():
    css_provider = Gtk.CssProvider()
    style_path = os.path.join(os.path.dirname(__file__), "style.css")
    css_provider.load_from_path(style_path)
    screen = Gdk.Screen.get_default()
    Gtk.StyleContext.add_provider_for_screen(
        screen, css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
    )

def is_raspberry_pi():
    try:
        with open("/proc/device-tree/model", "r") as f:
            return "Raspberry Pi" in f.read()
    except Exception:
        return False

class BottomNavigationBar(Gtk.Box):
    def __init__(self, stack):
        super().__init__(orientation=Gtk.Orientation.HORIZONTAL, spacing=15)
        self.stack = stack
        self.set_border_width(10)
        self.set_halign(Gtk.Align.CENTER)

        btn_specs = [
            ("🏠 Home", "start"),
            ("🕹️ Manual", "manual"),
            ("🤖 Autonomy", "autonomy"),
            ("📷 Camera", "camera"),
            ("💬 Chat", "chat"),
            ("🎤 Voice", "voice"),
            ("⚙ Settings", "settings")
        ]

        for label, page_name in btn_specs:
            btn = Gtk.Button(label=label)
            btn.set_size_request(120, 60)
            btn.connect("clicked", self.on_nav_button_clicked, page_name)
            self.pack_start(btn, False, False, 0)

    def on_nav_button_clicked(self, button, page_name):
        if page_name == "settings":
            print("Settings clicked")
        else:
            self.stack.set_visible_child_name(page_name)

class RobotGUI(Gtk.Application):
    def __init__(self):
        super().__init__(application_id="org.robot.gui")
        self.raspberry_pi = is_raspberry_pi()

    def do_activate(self):
        rclpy.init(args=None)
        ros_node = ROSPublisher()
        threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

        load_css()

        main_grid = Gtk.Grid()
        main_grid.set_row_homogeneous(False)
        main_grid.set_column_homogeneous(True)

        header = CustomHeaderBarBox()
        main_grid.attach(header, 0, 0, 1, 1)

        stack = Gtk.Stack()
        stack.set_transition_type(Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        stack.set_transition_duration(500)
        stack.set_hexpand(True)
        stack.set_vexpand(True)
        stack.set_halign(Gtk.Align.FILL)
        stack.set_valign(Gtk.Align.FILL)

        start  = HomeScreen(stack, ros_node)
        manual = ManualWindow(stack, ros_node)
        auto   = AutonomyWindow(stack, ros_node)
        camera = CameraWindow(stack, ros_node)
        chat   = ChatWindow(stack, ros_node)
        voice  = VoiceWindow(stack, ros_node)

        start.camera_window = camera

        stack.add_named(start,  "start")
        stack.add_named(manual, "manual")
        stack.add_named(auto,   "autonomy")
        stack.add_named(camera, "camera")
        stack.add_named(chat,   "chat")
        stack.add_named(voice,  "voice")
        # Add settings page here if implemented:
        # stack.add_named(settings_window, "settings")

        stack.set_visible_child_name("start")
        main_grid.attach(stack, 0, 1, 1, 1)

        bottom_nav = BottomNavigationBar(stack)
        main_grid.attach(bottom_nav, 0, 2, 1, 1)

        window = Gtk.ApplicationWindow(application=self)
        window.set_title("Robot GUI")
        window.fullscreen()
        window.set_decorated(False)

        window.connect("key-press-event", self.on_key_press)

        window.add(main_grid)
        window.show_all()
        window.resize(1, 1)

        window.connect("delete-event", self.on_window_delete, ros_node)

    def on_key_press(self, widget, event):
        if event.keyval == Gdk.KEY_Escape:
            active = self.get_active_window()
            if active:
                active.set_decorated(True)
                active.unfullscreen()
        return False

    def on_window_delete(self, window, event, ros_node):
        ros_node.destroy_node()
        rclpy.shutdown()
        return False

def main():
    app = RobotGUI()
    app.run()

if __name__ == "__main__":
    main()
