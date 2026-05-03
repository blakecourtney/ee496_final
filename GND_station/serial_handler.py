#handle input from GND esp32
import serial
import serial.tools.list_ports
import threading
import time

PACKET_START_BYTE  = 0xFE
PACKET_SIZE        = 68   # sizeof(packet_t)
PHOTO_PACKET_SIZE  = 411  # sizeof(photo_packet_t): 3 bytes + 1 pad + 3×uint16 + 230 + 1 + 1 pad
PKT_TYPE_PHOTO_CHUNK = 9

class SerialHandler:
    def __init__(self, callback=None):
        self.ser = None
        self.callback = callback
        self.running = False
        self.thread = None

    def list_ports(self):
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self, port, baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)
            print(f"Connected to {port}")
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False

    def start_reading(self):
        if self.ser and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._read_loop, daemon=True)
            self.thread.start()

    def stop_reading(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)

    def _read_exact(self, n):
        """Read exactly n bytes, retrying on partial reads until timeout"""
        buf = bytearray()
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                return None
            buf.extend(chunk)
        return bytes(buf)

    def _read_loop(self):
        while self.running:
            try:
                b = self.ser.read(1)
                if not b or b[0] != PACKET_START_BYTE:
                    continue

                # Read drone_id and type
                header = self._read_exact(2)
                if not header:
                    continue

                pkt_type = header[1]
                remainder = (PHOTO_PACKET_SIZE if pkt_type == PKT_TYPE_PHOTO_CHUNK
                             else PACKET_SIZE) - 3

                rest = self._read_exact(remainder)
                if rest and self.callback:
                    self.callback(bytes([PACKET_START_BYTE]) + header + rest)
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.1)

    def send(self, data):
        if self.ser:
            try:
                self.ser.write(data + b'\n')
                return True
            except Exception as e:
                print(f"Send error: {e}")
                return False
        return False

    def close(self):
        self.stop_reading()
        if self.ser:
            self.ser.close()
