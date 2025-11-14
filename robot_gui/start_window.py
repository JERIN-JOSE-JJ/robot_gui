import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

class HomeScreen(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.stack = stack
        self.ros_node = ros_node
        self.set_border_width(20)

        welcome_label = Gtk.Label(label="Welcome to ThaaraBot Control")
        welcome_label.set_halign(Gtk.Align.CENTER)
        self.pack_start(welcome_label, True, True, 0)

        # Add other content here, but no navigation buttons
