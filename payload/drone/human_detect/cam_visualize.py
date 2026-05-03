import serial
import numpy as np
import cv2
import sys

# --- CONFIGURATION ---
PORT = '/dev/cu.usbmodem5B414825001'
BAUD = 115200
WIDTH = 240
HEIGHT = 240
IMG_SIZE = WIDTH * HEIGHT * 2  # 32768 bytes for RGB565

# --- SERIAL SETUP ---
try:
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate = BAUD
    ser.timeout = 5
    ser.dtr = True
    ser.rts = True
    
    ser.open()
    ser.reset_input_buffer()
    print(f"Connected to {PORT}. Waiting for drone feed...")
    print("Press 'q' in the video window to quit.\n")
except Exception as e:
    print(f"Failed to connect to camera: {e}")
    sys.exit(1)

# --- MAIN STREAM LOOP ---
while True:
    try:
        line = ser.readline()
        # print(line)
        
        if b"START_IMAGE" in line:
            raw_data = bytearray()
            
            while len(raw_data) < IMG_SIZE:
                # Read safely in chunks to prevent macOS buffer overflow
                to_read = min(4096, IMG_SIZE - len(raw_data))
                chunk = ser.read(to_read)
                if not chunk:
                    break
                raw_data.extend(chunk)

                # print((raw_data))
            
            if len(raw_data) == IMG_SIZE:
                # Interpret as 16-bit Big-Endian
                data = np.frombuffer(raw_data, dtype='>u2').reshape((HEIGHT, WIDTH))
                
                # Unpack the 5-6-5 bits into 8-bit R, G, and B
                r = ((data >> 11) & 0x1F) << 3
                g = ((data >> 5) & 0x3F) << 2
                b = (data & 0x1F) << 3
                
                # Stack into an image and convert to OpenCV's BGR format
                img = np.stack([r, g, b], axis=-1).astype(np.uint8)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                cv2.imshow("Drone Feed", cv2.resize(img, (512, 512)))
                
                # Press 'q' to exit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print(f"Dropped frame: Received {len(raw_data)}/{IMG_SIZE} bytes.")
                ser.reset_input_buffer()
                
        elif line.strip():
            try:
                text_log = line.decode('utf-8').strip()
                # Ignore the END_IMAGE marker so the terminal stays clean
                if "PROBABILITY" in text_log:
                    print(f"{text_log}")
            except UnicodeDecodeError:
                pass # Ignore random bits of binary noise that fail to decode

    except KeyboardInterrupt:
        print("\nExiting stream...")
        break
    except Exception as e:
        print(f"Stream error: {e}")
        break

# --- CLEANUP ---
ser.close()
cv2.destroyAllWindows()