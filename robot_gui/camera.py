import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk


class CameraWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        self.stack = stack
        self.ros_node = ros_node

        # Placeholder label or your camera widget here
        label = Gtk.Label(label="Camera View")
        self.pack_start(label, True, True, 0)

        # Back button packed last to appear at bottom
        back_btn = Gtk.Button(label="← Back")
        back_btn.connect("clicked", self.on_back_clicked)
        self.pack_start(back_btn, False, False, 10)

    def on_back_clicked(self, button):
        # Switch back to the start page in the stack
        self.stack.set_visible_child_name("start")
