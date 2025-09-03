import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GdkPixbuf, GLib
import cv2
import numpy as np

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

        self.cap = None
        self._update_id = None

    def start_camera(self):
        if self.cap is None:
            self.cap = cv2.VideoCapture(2)
            if not self.cap.isOpened():
                print("Failed to open camera!")
                self.cap = None
                return
            self._update_id = GLib.timeout_add(30, self.update_camera_frame)

    def stop_camera(self):
        if self.cap:
            if self._update_id is not None:
                GLib.source_remove(self._update_id)
                self._update_id = None
            self.cap.release()
            self.cap = None
            self.camera_image.clear()  # Optional clear image

    def update_camera_frame(self):
        if self.cap is None:
            return False
        ret, frame = self.cap.read()
        if not ret:
            return True
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
        self.camera_image.set_from_pixbuf(pixbuf)
        return True

    def on_back_clicked(self, button):
        self.stop_camera()
        self.stack.set_visible_child_name("start")
