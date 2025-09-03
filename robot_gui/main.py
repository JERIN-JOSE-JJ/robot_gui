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
from robot_gui.camera import CameraWindow
from robot_gui.chat_ai import ChatWindow

def load_css():
    css_provider = Gtk.CssProvider()
    style_path = os.path.join(os.path.dirname(__file__), "style.css")
    css_provider.load_from_path(style_path)
    screen = Gdk.Screen.get_default()
    Gtk.StyleContext.add_provider_for_screen(
        screen, css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION
    )

def is_raspberry_pi():
    """Check if running on Raspberry Pi"""
    try:
        with open('/proc/device-tree/model', 'r') as f:
            return 'Raspberry Pi' in f.read()
    except:
        return False

class RobotGUI(Gtk.Application):
    def __init__(self):
        super().__init__(application_id="org.robot.gui")
        self.raspberry_pi = is_raspberry_pi()

    def do_activate(self):
        # ROS 2 Initialization
        rclpy.init(args=None)
        ros_node = ROSPublisher()

        # Spin ROS 2 in background thread
        threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True).start()

        # Load CSS
        load_css()

        # Create a main grid that will properly handle expansion
        main_grid = Gtk.Grid()
        main_grid.set_row_homogeneous(False)
        main_grid.set_column_homogeneous(True)

        # Header bar
        header = CustomHeaderBarBox()
        main_grid.attach(header, 0, 0, 1, 1)  # row 0, height 1

        # Stack for all pages - this will expand to fill remaining space
        stack = Gtk.Stack()
        stack.set_transition_type(Gtk.StackTransitionType.SLIDE_LEFT_RIGHT)
        stack.set_transition_duration(500)

        
        # Configure stack to expand
        stack.set_hexpand(True)
        stack.set_vexpand(True)
        stack.set_halign(Gtk.Align.FILL)
        stack.set_valign(Gtk.Align.FILL)

        # GUI windows with ros_node injected
        start = HomeScreen(stack, ros_node)
        manual = ManualWindow(stack, ros_node)
        auto = AutonomyWindow(stack, ros_node)
        camera = CameraWindow(stack, ros_node)
        chat = ChatWindow(stack, ros_node)

        stack.add_named(start, "start")
        stack.add_named(manual, "manual")
        stack.add_named(auto, "autonomy")
        stack.add_named(camera, "camera")
        stack.add_named(chat, "chat")

        # Attach stack to grid - row 1, and set it to expand
        main_grid.attach(stack, 0, 1, 1, 1)  # column 0, row 1, width 1, height 1
        
        # Configure grid row properties
        main_grid.set_row_spacing(0)
        main_grid.set_border_width(0)

        # Main window - FULLSCREEN
        window = Gtk.ApplicationWindow(application=self)
        window.set_title("Robot GUI")
        
        # Set fullscreen and remove decorations
        window.fullscreen()
        window.set_decorated(False)
        
        # Add keyboard escape handler (ESC to exit)
        window.connect("key-press-event", self.on_key_press)
        
        # Add the main grid to window
        window.add(main_grid)
        
        # Show all and ensure everything expands
        window.show_all()
        
        # Force size allocation after showing
        window.resize(1, 1)  # Trigger resize to force expansion
        
        # Connect delete event to properly shutdown ROS
        window.connect("delete-event", self.on_window_delete, ros_node)

    def on_key_press(self, widget, event):
        """Handle keyboard events - ESC to exit fullscreen"""
        if event.keyval == Gdk.KEY_Escape:
            window = self.get_active_window()
            window.set_decorated(True)
            window.unfullscreen()
            # Optional: Add a quit dialog or button
        return False

    def on_window_delete(self, window, event, ros_node):
        """Cleanup function when window is closed"""
        ros_node.destroy_node()
        rclpy.shutdown()
        return False

def main():
    app = RobotGUI()
    app.run()

if __name__ == "__main__":
    main()