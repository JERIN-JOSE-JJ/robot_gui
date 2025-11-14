import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GdkPixbuf, GLib
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


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

        

        self.bridge = CvBridge()

        # Subscribe to the image topic with proper callback
        self.image_subscriber = self.ros_node.create_subscription(
            Image,
            '/person_follower/image',
            self.image_callback,
            10
        )


        self.detection_subscriber = ros_node.create_subscription(
            String,
            '/person_detection/image',
            self.detections_callback,
            10
        )

    def detections_callback(self, msg):
        def update_list():
            self.detected_list.foreach(lambda row: self.detected_list.remove(row))
            for name in msg.data:  # Assuming msg.data is list of names
                row = Gtk.ListBoxRow()
                label = Gtk.Label(label=name)
                row.add(label)
                self.detected_list.add(row)
            self.detected_list.show_all()
        GLib.idle_add(update_list)
            

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, c = frame.shape
            pixbuf = GdkPixbuf.Pixbuf.new_from_data(
                frame.tobytes(),
                GdkPixbuf.Colorspace.RGB,
                False,
                8,
                w,
                h,
                w * c
            )
            GLib.idle_add(self.camera_image.set_from_pixbuf, pixbuf)
        except Exception as e:
            print(f"Image conversion error: {e}")

    def stop_subscription(self):
        if self.image_subscriber:
            self.ros_node.destroy_subscription(self.image_subscriber)
            self.image_subscriber = None

    
