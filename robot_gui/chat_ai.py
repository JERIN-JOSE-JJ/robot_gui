import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GObject
import threading
import requests
import os

class ChatWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.stack = stack
        self.ros_node = ros_node
        self.last_bot_reply = ""

        # Output: Bot reply TextView (big, non-editable)
        self.output_view = Gtk.TextView()
        self.output_view.set_editable(False)
        self.output_view.set_wrap_mode(Gtk.WrapMode.WORD)
        self.output_view.set_vexpand(True)
        scroll = Gtk.ScrolledWindow()
        scroll.set_hexpand(True)
        scroll.set_vexpand(True)
        scroll.add(self.output_view)
        self.pack_start(scroll, True, True, 0)

        # Bottom area: Input, buttons in a horizontal box
        input_area = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)

        # User input entry
        self.entry = Gtk.Entry()
        self.entry.set_placeholder_text("Type your message here...")
        self.entry.set_hexpand(True)
        self.entry.connect("activate", self.on_send)
        input_area.pack_start(self.entry, True, True, 0)

        # Voice input button
        voice_btn = Gtk.Button(label="🎤")
        voice_btn.set_tooltip_text("Speak your input")
        voice_btn.connect("clicked", self.on_voice_clicked)
        input_area.pack_start(voice_btn, False, False, 0)



        # Send button
        send_btn = Gtk.Button(label="Send")
        send_btn.connect("clicked", self.on_send)
        input_area.pack_start(send_btn, False, False, 0)

        # Button: Play output as speech
        tts_btn = Gtk.Button(label="🔊 Play Output")
        tts_btn.set_tooltip_text("Play last bot reply")
        tts_btn.connect("clicked", self.on_tts_clicked)
        input_area.pack_start(tts_btn, False, False, 0)

        self.pack_start(input_area, False, False, 10)

        
    

    # -- Core Methods --

    def set_output(self, message):
        buffer = self.output_view.get_buffer()
        buffer.set_text(message)
        self.last_bot_reply = message

    def on_send(self, widget):
        message = self.entry.get_text().strip()
        if not message:
            return
        self.set_output("You: " + message)
        self.entry.set_text("")
        threading.Thread(target=self.call_chatbot_api, args=(message,), daemon=True).start()

    def call_chatbot_api(self, text):
        try:
            url = "http://192.168.29.87:8000/api/respond"
            payload = {"prompt": text}
            response = requests.post(url, data=payload, timeout=8)
            if response.status_code == 200:
                reply = response.json().get("response", "No reply provided.")
            else:
                reply = f"Error: {response.status_code}, {response.text}"
        except Exception as e:
            reply = f"API error: {e}"
        GObject.idle_add(self.set_output, "Bot: " + reply)

    def on_voice_clicked(self, button):
        # Integrate Python SpeechRecognition or your preferred voice system here
        self.set_output("System: Voice input feature not implemented yet.")

    def on_tts_clicked(self, button):
        threading.Thread(target=self.call_tts_api, args=(self.last_bot_reply,), daemon=True).start()

    def call_tts_api(self, text):
        if not text:
            GObject.idle_add(self.set_output, "System: Nothing to speak.")
            return
        try:
            url = "http://192.168.29.87:8000/api/tts"
            payload = {"text": text, "as_base64": "false"}
            response = requests.post(url, data=payload, timeout=20)
            if response.status_code == 200:
                with open("bot_reply.mp3", "wb") as f:
                    f.write(response.content)
                GObject.idle_add(self.set_output, self.last_bot_reply + "\n[Audio saved as bot_reply.mp3]")
                os.system("mpg123 bot_reply.mp3")  # Play using system player (needs mpg123 installed)
            else:
                GObject.idle_add(self.set_output, f"TTS Error: {response.status_code}, {response.text}")
        except Exception as e:
            GObject.idle_add(self.set_output, f"TTS API error: {e}")
