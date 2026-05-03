# main GND station
from tkinter import *
from tkinter import ttk, messagebox
import sys
import time
import threading
import os
import numpy as np
from PIL import Image
from serial_handler import SerialHandler
from telemetry_parser import TelemetryParser
from command_builder import CommandBuilder
from gui import GroundStationGUI


class GroundStation:
    def __init__(self):
        self.serial = SerialHandler(callback=self.on_serial_data)
        self.parser = TelemetryParser()
        self.photo_chunks = {}  # drone_id -> {chunk_index: bytes}

        # Setup GUI
        self.root = Tk()
        self.gui = GroundStationGUI(self.root, self.on_command,
                                    change_port_callback=self.reconnect_serial)

        # Connect to serial
        self.connect_serial()

        # Start drone timeout checker
        self.start_drone_timeout_checker()

    def connect_serial(self):
        """Prompt user to select serial port via dropdown"""
        ports = self.serial.list_ports()
        if not ports:
            messagebox.showerror("Error", "No serial ports found!")
            sys.exit(1)

        dialog = Toplevel(self.root)
        dialog.title("Select Serial Port")
        dialog.geometry("300x130")
        dialog.transient(self.root)
        dialog.grab_set()
        dialog.resizable(False, False)

        Label(dialog, text="Select GCS ESP32 port:", pady=10).pack()

        port_var = StringVar(value=ports[0])
        ttk.Combobox(dialog, textvariable=port_var, values=ports,
                     state="readonly", width=25).pack(padx=20, pady=5)

        def connect():
            port = port_var.get()
            if self.serial.connect(port):
                self.serial.start_reading()
                self.gui.status_label.config(text=f"Connected to {port}")
                dialog.destroy()
            else:
                messagebox.showerror("Error", f"Failed to connect to {port}")

        Button(dialog, text="Connect", command=connect).pack(pady=10)
        dialog.wait_window()

    def reconnect_serial(self):
        """Close current connection and prompt for a new port"""
        self.serial.close()
        self.connect_serial()

    def on_serial_data(self, data):
        try:
            parsed = self.parser.parse(data)
            if not parsed:
                return

            if parsed['type'] == 'photo_chunk':
                self._accumulate_chunk(parsed)
            elif parsed['type'] == 'photo_done':
                self._reconstruct_image(parsed['id'])
                self.root.after(0, self.gui.update_drone, parsed)
            else:
                self.root.after(0, self.gui.update_drone, parsed)
        except Exception as e:
            print(f"Parse error: {e}")

    def _accumulate_chunk(self, parsed):
        drone_id = parsed['id']
        idx      = parsed['chunk_index']
        total    = parsed['total_chunks']

        if drone_id not in self.photo_chunks:
            self.photo_chunks[drone_id] = {'total': total, 'chunks': {}}
            self.root.after(0, self.gui.update_drone, {
                'type': 'photo_start', 'id': drone_id
            })

        self.photo_chunks[drone_id]['chunks'][idx] = parsed['data']
        received = len(self.photo_chunks[drone_id]['chunks'])
        print(f"[camera] chunk {idx+1}/{total} drone:{drone_id} ({received} received)")

    def _reconstruct_image(self, drone_id):
        entry = self.photo_chunks.pop(drone_id, None)
        if not entry:
            print(f"[camera] photo_done for drone {drone_id} but no chunks buffered")
            return

        chunks = entry['chunks']
        total  = entry['total']
        missing = [i for i in range(total) if i not in chunks]
        if missing:
            print(f"[camera] missing chunks {missing}, dropping image")
        
        raw = bytearray()
        for i in range(total):
            if i in chunks:
                raw.extend(chunks[i])
            else:
                raw.extend(b'\x00' * 230)  # fill missing chunk

        raw = bytearray()
        for i in range(total):
            raw.extend(chunks[i])

        try:
            data = np.frombuffer(raw, dtype='>u2').reshape((240, 240))
            r = ((data >> 11) & 0x1F) << 3
            g = ((data >> 5)  & 0x3F) << 2
            b = ( data        & 0x1F) << 3
            img = np.stack([r, g, b], axis=-1).astype(np.uint8)

            save_dir = os.path.join(os.path.dirname(__file__), 'received_images')
            os.makedirs(save_dir, exist_ok=True)
            filename = os.path.join(save_dir, f"img_{time.strftime('%Y%m%d_%H%M%S')}.png")
            Image.fromarray(img).save(filename)
            print(f"[camera] Image saved: {filename}")

            self.root.after(0, self.gui.display_image, img)
        except Exception as e:
            print(f"Image decode error: {e}")

    def on_command(self, cmd_type, drone_id, *args):
        """Called when user clicks a command button"""
        if cmd_type == 'arm':
            packet = CommandBuilder.arm(drone_id)
        elif cmd_type == 'disarm':
            packet = CommandBuilder.disarm(drone_id)
        elif cmd_type == 'request_image':
            packet = CommandBuilder.flag_ack(drone_id)
            # packet = CommandBuilder.request_image(drone_id)
        elif cmd_type == 'waypoint':
            lat, lon, alt = args
            packet = CommandBuilder.waypoint(drone_id, lat, lon, alt)
        else:
            return
        self.serial.send(packet)
        print(f"Sent {cmd_type} to Drone {drone_id}")

    def start_drone_timeout_checker(self):
        """Background thread that turns indicators red when a drone stops sending"""
        def check():
            while True:
                now = time.time()
                for did, data in list(self.gui.drones.items()):
                    connected = (now - data.get('last_seen', 0)) < 10
                    self.root.after(0, self.gui.update_drone_indicator, did, connected)
                time.sleep(3)

        t = threading.Thread(target=check, daemon=True)
        t.start()

    def run(self):
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.on_closing()

    def on_closing(self):
        self.serial.close()
        self.root.destroy()


if __name__ == "__main__":
    app = GroundStation()
    app.run()
