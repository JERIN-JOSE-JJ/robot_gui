import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GdkPixbuf, GLib
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        self.stack = stack
        self.ros_node = ros_node

        main_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        self.pack_start(main_box, True, True, 0)

        self.camera_image = Gtk.Image()
        main_box.pack_start(self.camera_image, True, True, 0)

        right_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        right_box.set_size_request(300, -1)
        main_box.pack_start(right_box, False, False, 0)

        detected_label = Gtk.Label(label="Detected Persons")
        right_box.pack_start(detected_label, False, False, 5)

        self.detected_list = Gtk.ListBox()
        right_box.pack_start(self.detected_list, True, True, 0)

        back_btn = Gtk.Button(label="← Back")
        back_btn.set_size_request(150, 60)
        back_btn.set_hexpand(False)
        back_btn.set_halign(Gtk.Align.CENTER)
        back_btn.connect("clicked", self.on_back_clicked)
        self.pack_start(back_btn, False, False, 10)

        self.bridge = CvBridge()

        # Store subscription so we can destroy it later
        self.image_subscriber = self.ros_node.create_subscription(
            Image, '/person_follower/image', self.image_callback, 10
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width, channels = frame.shape
            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                frame.tobytes(),
                GdkPixbuf.Colorspace.RGB,
                False,
                8,
                width,
                height,
                width * channels
            )
            GLib.idle_add(self.camera_image.set_from_pixbuf, pixbuf)
        except Exception as e:
            print(f"Image conversion error: {e}")

    def stop_subscription(self):
        if self.image_subscriber:
            self.ros_node.destroy_subscription(self.image_subscriber)
            self.image_subscriber = None

    def on_back_clicked(self, button):
        self.stop_subscription()
        # Notify home_screen or main controller to stop camera node subprocess
        if hasattr(self, 'home_screen'):
            self.home_screen.stop_camera_and_node()
        self.stack.set_visible_child_name("start")
