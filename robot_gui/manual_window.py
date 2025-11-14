import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

class ManualWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        self.stack = stack
        self.ros_node = ros_node

        label = Gtk.Label(label="Manual Control Mode")
        label.set_margin_bottom(10)
        self.pack_start(label, False, False, 0)

        content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=40)

        # Movement Buttons
        control_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        button_grid = Gtk.Grid()
        button_grid.set_row_spacing(20)
        button_grid.set_column_spacing(20)

        self.add_control_button(button_grid, "↑ Forward", 1, 0, self.move_forward)
        self.add_control_button(button_grid, "← Left", 0, 1, self.move_left)
        self.add_control_button(button_grid, "■ Stop", 1, 1, self.stop)
        self.add_control_button(button_grid, "→ Right", 2, 1, self.move_right)
        self.add_control_button(button_grid, "↓ Backward", 1, 2, self.move_backward)

        control_box.pack_start(button_grid, False, False, 0)

        # Servo Sliders
        servo_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=30)
        slider_data = [
            ("Head Angle", self.on_head_changed),
            ("Left Arm", self.on_left_hand_changed),
            ("Right Arm", self.on_right_hand_changed)
        ]

        for name, callback in slider_data:
            slider_row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            label = Gtk.Label(label=name)
            label.set_halign(Gtk.Align.CENTER)

            scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
            scale.set_range(0, 180)
            scale.set_value(90)
            scale.set_digits(0)
            scale.connect("value-changed", callback)

            slider_row.pack_start(label, False, False, 5)
            slider_row.pack_start(scale, False, False, 5)
            servo_box.pack_start(slider_row, False, False, 10)

        # Combine layout
        content_box.pack_start(control_box, True, True, 0)
        content_box.pack_start(servo_box, True, True, 0)
        self.pack_start(content_box, True, True, 0)

        # Back Button
        
    def add_control_button(self, grid, label, col, row, callback):
        btn = Gtk.Button(label=label)
        btn.set_size_request(150, 80)
        btn.connect("clicked", callback)
        grid.attach(btn, col, row, 1, 1)

    

    # Movement Commands
    def move_forward(self, button): self.ros_node.publish_cmd("forward")
    def move_backward(self, button): self.ros_node.publish_cmd("backward")
    def move_left(self, button): self.ros_node.publish_cmd("left")
    def move_right(self, button): self.ros_node.publish_cmd("right")
    def stop(self, button): self.ros_node.publish_cmd("stop")

    # Servo Sliders
    def on_head_changed(self, scale): self.ros_node.publish_servo("head", int(scale.get_value()))
    def on_left_hand_changed(self, scale): self.ros_node.publish_servo("left_hand", int(scale.get_value()))
    def on_right_hand_changed(self, scale): self.ros_node.publish_servo("right_hand", int(scale.get_value()))
