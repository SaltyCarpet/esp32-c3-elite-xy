import serial
import struct
import threading
import queue
import time
from pynput import keyboard, mouse

# --- Serial setup ---
PORT = 'COM3'   # Change to your ESP32 port
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# --- Protocol ---
HEADER = 0xAA

send_q = queue.Queue()

def send_packet(key_code, value):
    """
    Build packet: header (1), key (1), float (4), checksum (1) = 7 bytes
    """
    payload = struct.pack('<BBf', HEADER, key_code, float(value))
    checksum = 0
    for b in payload:
        checksum ^= b
    full = payload + bytes([checksum])
    send_q.put(full)
    print("TX:", list(full), "len=", len(full))

def writer():
    while ser.is_open:
        try:
            pkt = send_q.get(timeout=0.1)
            ser.write(pkt)
        except queue.Empty:
            pass

def reader():
    while ser.is_open:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print("ESP:", line)
        except Exception as e:
            print("Read error:", e)
            break

# --- Keyboard handling ---
pressed_keys = set()

def on_press(key):
    try:
        c = key.char
        if c in ['w','s','a','d','q','e','i','k','j','l','r']:
            if c not in pressed_keys:
                pressed_keys.add(c)
                send_packet(ord(c), 1.0)   # pressed = 1.0
    except AttributeError:
        pass

def on_release(key):
    try:
        c = key.char
        if c in pressed_keys:
            pressed_keys.remove(c)
            send_packet(ord(c), 0.0)   # released = 0.0
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        return False

# --- Mouse handling ---
last_x, last_y = None, None
last_send = 0

def on_move(x, y):
    global last_x, last_y, last_send
    now = time.time()
    if now - last_send < 0.02:  # throttle to 50 Hz
        return
    if last_x is None:
        last_x, last_y = x, y
        return
    dx, dy = x - last_x, y - last_y
    last_x, last_y = x, y
    if dx or dy:
        # send axis codes: 100 for X, 101 for Y
        send_packet(150, float(dx))
        send_packet(151, float(dy))
        last_send = now

def on_click(x, y, button, pressed):
    if pressed:
        if button == mouse.Button.left:
            send_packet(ord('q'), 1.0)
        elif button == mouse.Button.right:
            send_packet(ord('e'), 1.0)
    else:
        if button == mouse.Button.left:
            send_packet(ord('q'), 0.0)
        elif button == mouse.Button.right:
            send_packet(ord('e'), 0.0)

# --- Run listeners ---
def start_listeners():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as kb_listener, \
         mouse.Listener(on_move=on_move, on_click=on_click) as ms_listener:
        kb_listener.join()
        ms_listener.stop()

if __name__ == "__main__":
    print("Controls: WASD/QE/IJKL/R + mouse, ESC to quit")
    threading.Thread(target=writer, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()
    start_listeners()
    ser.close()
