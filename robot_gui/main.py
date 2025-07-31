import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, Gio, Gdk

import rclpy
import threading
import os

from robot_gui.header_bar import CustomHeaderBarBox
from robot_gui.start_window import HomeScreen
from robot_gui.manual_window import ManualWindow
from robot_gui.autonomy_window import AutonomyWindow
from robot_gui.ros_publisher import ROSPublisher

def load_css():
    css_provider = Gtk.CssProvider()
    style_path = os.path.join(os.path.dirname(__file__), "style.css")
    css_provider.load_from_path(style_path)
    screen = Gdk.Screen.get_default()
    Gtk.StyleContext.add_provider_for_screen(
        screen, css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
    )

class RobotGUI(Gtk.Application):
    def __init__(self):
        super().__init__(application_id="org.robot.gui")

    def do_activate(self):
        # ROS 2 Initialization
        rclpy.init(args=None)
        ros_node = ROSPublisher()

        # Spin ROS 2 in background thread
        threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

        # Load CSS
        load_css()

        # Stack for all pages
        stack = Gtk.Stack()
        stack.set_transition_type(Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        stack.set_transition_duration(500)

        # GUI windows with ros_node injected
        start = HomeScreen(stack, ros_node)
        manual = ManualWindow(stack, ros_node)
        auto = AutonomyWindow(stack, ros_node)

        stack.add_named(start, "start")
        stack.add_named(manual, "manual")
        stack.add_named(auto, "autonomy")

        # Header bar
        header = CustomHeaderBarBox()

        # Layout
        main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        main_box.pack_start(header, False, False, 0)
        main_box.pack_start(stack, True, True, 0)

        # Main window
        window = Gtk.ApplicationWindow(application=self)
        window.set_title("Robot GUI")
        window.set_default_size(800, 480)
        window.add(main_box)
        window.show_all()

def main():
    app = RobotGUI()
    app.run()

if __name__ == "__main__":
    main()
