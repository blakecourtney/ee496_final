import threading
import serial
import numpy as np
import cv2

class CameraHandler:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.running = False
        self.frame = None
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def get_frame(self):
        with self.lock:
            return self.frame

    def _run(self):
        WIDTH, HEIGHT = 240, 240
        IMG_SIZE = WIDTH * HEIGHT * 2

        try:
            ser = serial.Serial(self.port, self.baud, timeout=5)
            ser.reset_input_buffer()
            print("Camera connected")
        except Exception as e:
            print("[WARNING] Camera connection failed:", e)
            self.running = False
            return

        while self.running:
            try:
                line = ser.readline()

                if b"START_IMAGE" in line:
                    raw_data = bytearray()

                    while len(raw_data) < IMG_SIZE:
                        chunk = ser.read(min(4096, IMG_SIZE - len(raw_data)))
                        if not chunk:
                            break
                        raw_data.extend(chunk)

                    if len(raw_data) == IMG_SIZE:
                        data = np.frombuffer(raw_data, dtype='>u2').reshape((HEIGHT, WIDTH))

                        r = ((data >> 11) & 0x1F) << 3
                        g = ((data >> 5) & 0x3F) << 2
                        b = (data & 0x1F) << 3

                        img = np.stack([r, g, b], axis=-1).astype(np.uint8)

                        # store safely
                        with self.lock:
                            self.frame = img

            except Exception as e:
                print("Camera stream error:", e)
                break

        ser.close()