import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

class AutonomyWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.stack = stack
        self.ros_node = ros_node

        label = Gtk.Label(label="Autonomous Mode")
        self.pack_start(label, True, True, 0)

        start_btn = Gtk.Button(label="Start Autonomy")
        start_btn.connect("clicked", self.on_start_autonomy)
        self.pack_start(start_btn, True, True, 0)

        

    def on_start_autonomy(self, widget):
        self.ros_node.publish_mode("autonomous")
        self.ros_node.enable_autonomy(True)

    
