import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GObject

import threading, queue, json
import sounddevice as sd
from vosk import Model, KaldiRecognizer

from geometry_msgs.msg import Twist   # ROS 2 message type


class VoiceWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        """
        stack     : Gtk.Stack for navigation
        ros_node  : the shared ROS2 node from main.py
        """
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        self.stack = stack
        self.ros_node = ros_node

        # --- ROS publisher for robot movement ---
        self.cmd_pub = self.ros_node.create_publisher(Twist, "/cmd_vel", 10)

        # --- GUI Layout ---
        self.set_border_width(20)
        title = Gtk.Label(label="🎤 Voice Control")
        title.set_margin_bottom(10)
        self.pack_start(title, False, False, 0)

        self.status_label = Gtk.Label(label="Press the mic to start listening.")
        self.pack_start(self.status_label, False, False, 0)

        self.mic_btn = Gtk.Button(label="🎙️ Start")
        self.mic_btn.connect("clicked", self.toggle_listen)
        self.pack_start(self.mic_btn, False, False, 0)

        back_btn = Gtk.Button(label="← Back")
        back_btn.set_size_request(150, 60)
        back_btn.set_hexpand(False)
        back_btn.set_halign(Gtk.Align.CENTER)
        back_btn.connect("clicked", self.on_back_clicked)
        self.pack_start(back_btn, False, False, 10)

        # --- Vosk / sounddevice setup ---
        self.model = Model("/home/user/vosk_models/vosk-model-small-en-us-0.15")
        self.rec = KaldiRecognizer(self.model, 16000)
        self.q = queue.Queue()

        self.stream = None
        self.listen_thread = None
        self.listening = False

    # ------------------------------------------------------------------
    # Button callbacks
    # ------------------------------------------------------------------
    def toggle_listen(self, button):
        if not self.listening:
            self.start_listening()
        else:
            self.stop_listening(send_stop=True)

    def on_back_clicked(self, button):
        # Always stop listening and send a stop Twist when leaving window
        self.stop_listening(send_stop=True)
        self.stack.set_visible_child_name("start")

    # ------------------------------------------------------------------
    # Voice control logic
    # ------------------------------------------------------------------
    def start_listening(self):
        self.status_label.set_text("Listening… say forward, back, left, right, stop")
        self.mic_btn.set_label("🛑 Stop")
        self.listening = True

        # Microphone stream
        self.stream = sd.RawInputStream(
            samplerate=16000, blocksize=8000,
            dtype='int16', channels=1,
            callback=lambda indata, frames, t, status: self.q.put(bytes(indata))
        )
        self.stream.start()

        # Background thread for processing
        self.listen_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.listen_thread.start()

    def stop_listening(self, send_stop=False):
        if not self.listening:
            return
        self.listening = False
        self.mic_btn.set_label("🎙️ Start")
        self.status_label.set_text("Stopped listening.")

        # Stop mic stream
        if self.stream:
            try:
                self.stream.stop()
                self.stream.close()
            except Exception:
                pass
            self.stream = None

        # Optionally send a zero Twist to stop the robot
        if send_stop:
            t = Twist()
            self.cmd_pub.publish(t)

    def process_audio(self):
        while self.listening:
            if not self.q.empty():
                if self.rec.AcceptWaveform(self.q.get()):
                    text = json.loads(self.rec.Result()).get("text", "").lower()
                    if text:
                        GObject.idle_add(self.status_label.set_text, f"Heard: {text}")
                        self.handle_command(text)

    def handle_command(self, text):
        t = Twist()
        if "forward" in text:
            t.linear.x = 0.2
        elif "back" in text:
            t.linear.x = -0.2
        elif "left" in text:
            t.angular.z = 0.5
        elif "right" in text:
            t.angular.z = -0.5
        elif "stop" in text:
            pass  # Zero Twist stops
        else:
            return
        self.cmd_pub.publish(t)
