import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk


class ChatWindow(Gtk.Box):
    def __init__(self, stack, ros_node):
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        self.stack = stack
        self.ros_node = ros_node

        self.chat_view = Gtk.TextView()
        self.chat_view.set_editable(False)
        self.chat_view.set_wrap_mode(Gtk.WrapMode.WORD)
        self.pack_start(self.chat_view, True, True, 0)

        self.entry = Gtk.Entry()
        self.entry.set_placeholder_text("Type your message here...")
        self.entry.connect("activate", self.on_send)
        self.pack_start(self.entry, False, False, 0)

        # Back button packed last to show at bottom
        back_btn = Gtk.Button(label="← Back")
        back_btn.set_size_request(150, 60)
        back_btn.set_hexpand(False)
        back_btn.set_halign(Gtk.Align.CENTER)
        back_btn.connect("clicked", self.on_back_clicked)
        self.pack_start(back_btn, False, False, 10)

    def on_back_clicked(self, button):
        self.stack.set_visible_child_name("start")

    def on_send(self, widget):
        buffer = self.chat_view.get_buffer()
        start, end = buffer.get_bounds()
        text = buffer.get_text(start, end, True)
        message = self.entry.get_text().strip()
        if message != "":
            buffer.set_text(text + "\nYou: " + message)
            self.entry.set_text("")
            buffer.set_text(buffer.get_text(buffer.get_start_iter(), buffer.get_end_iter(), True) + "\nAI: " + "Echo: " + message)
