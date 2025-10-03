import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk, GObject
import threading
import requests  # for your chatbot API calls


class ChatWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.stack = stack
        self.ros_node = ros_node

        # Chat display area inside ScrolledWindow
        self.chat_view = Gtk.TextView()
        self.chat_view.set_editable(False)
        self.chat_view.set_wrap_mode(Gtk.WrapMode.WORD)
        scroll = Gtk.ScrolledWindow()
        scroll.set_hexpand(True)
        scroll.set_vexpand(True)
        scroll.add(self.chat_view)
        self.pack_start(scroll, True, True, 0)

        # User input horizontal box (entry + voice + send buttons)
        input_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)

        self.entry = Gtk.Entry()
        self.entry.set_placeholder_text("Type your message here...")
        self.entry.connect("activate", self.on_send)
        input_box.pack_start(self.entry, True, True, 0)

        # Voice input button, add your voice trigger callback here
        voice_btn = Gtk.Button(label="🎤")
        voice_btn.set_tooltip_text("Ask via voice (not implemented)")
        voice_btn.connect("clicked", self.on_voice_clicked)
        input_box.pack_start(voice_btn, False, False, 0)

        # Send button
        send_btn = Gtk.Button(label="Send")
        send_btn.connect("clicked", self.on_send)
        input_box.pack_start(send_btn, False, False, 0)

        self.pack_start(input_box, False, False, 0)

        # Back button at bottom
        back_btn = Gtk.Button(label="← Back")
        back_btn.set_size_request(150, 60)
        back_btn.set_hexpand(False)
        back_btn.set_halign(Gtk.Align.CENTER)
        back_btn.connect("clicked", self.on_back_clicked)
        self.pack_start(back_btn, False, False, 10)

    def append_message(self, sender, message):
        buffer = self.chat_view.get_buffer()
        end_iter = buffer.get_end_iter()
        buffer.insert(end_iter, f"{sender}: {message}\n")
        # Scroll to end
        mark = buffer.create_mark(None, buffer.get_end_iter(), False)
        self.chat_view.scroll_to_mark(mark, 0.0, True, 0.0, 1.0)

    def on_send(self, widget):
        message = self.entry.get_text().strip()
        if not message:
            return
        self.append_message("You", message)
        self.entry.set_text("")
        threading.Thread(target=self.call_chatbot_api, args=(message,), daemon=True).start()

    def call_chatbot_api(self, text):
        # Placeholder example: replace with your real API call
        try:
            # Simulate API call delay
            # url = "https://your-api.url/chat"
            # payload = {"message": text}
            # response = requests.post(url, json=payload)
            # reply = response.json().get("reply", "Sorry, no reply.")
            reply = "Echo: " + text  # Mock reply
        except Exception as e:
            reply = f"API error: {e}"
        GObject.idle_add(self.append_message, "Bot", reply)

    def on_voice_clicked(self, button):
        # TODO: Implement voice input trigger
        self.append_message("System", "Voice input feature not implemented yet.")

    def on_back_clicked(self, button):
        self.stack.set_visible_child_name("start")
