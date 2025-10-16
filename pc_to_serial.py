import serial
import struct
import threading
from pynput import keyboard, mouse

# --- Serial setup ---
PORT = 'COM4'   # Change to your ESP32 port
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# --- Protocol ---
HEADER = 0xAA
CMD_MOVE = 0x01
CMD_KEY  = 0x02

def send_packet(cmd, dx=0, dy=0):
    # Build 7‑byte payload: header (1), cmd (1), dx (2), dy (2), pad (1)
    # The pad ensures we always have 7 bytes before checksum
    payload = struct.pack('<BBhhB', HEADER, cmd, dx, dy, 0)

    # Compute checksum over the first 7 bytes
    checksum = 0
    for b in payload:
        checksum ^= b

    # Append checksum to make 8 bytes total
    full = payload + bytes([checksum])

    ser.write(full)
    print("TX:", list(full), "len=", len(full))


# --- Reader thread to show ESP output ---
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
                send_packet(CMD_KEY, ord(c), 1)
    except AttributeError:
        pass

def on_release(key):
    try:
        c = key.char
        if c in pressed_keys:
            pressed_keys.remove(c)
            send_packet(CMD_KEY, ord(c), 0)
    except AttributeError:
        pass
    if key == keyboard.Key.esc:
        return False

# --- Mouse handling ---
last_x, last_y = None, None

def on_move(x, y):
    global last_x, last_y
    if last_x is None:
        last_x, last_y = x, y
        return
    dx = x - last_x
    dy = y - last_y
    last_x, last_y = x, y
    if dx or dy:
        send_packet(CMD_MOVE, int(dx), int(dy))

def on_click(x, y, button, pressed):
    if pressed:
        if button == mouse.Button.left:
            send_packet(CMD_KEY, ord('q'), 1)
        elif button == mouse.Button.right:
            send_packet(CMD_KEY, ord('e'), 1)
    else:
        if button == mouse.Button.left:
            send_packet(CMD_KEY, ord('q'), 0)
        elif button == mouse.Button.right:
            send_packet(CMD_KEY, ord('e'), 0)

# --- Run listeners ---
def start_listeners():
    with keyboard.Listener(on_press=on_press, on_release=on_release) as kb_listener, \
         mouse.Listener(on_move=on_move, on_click=on_click) as ms_listener:
        kb_listener.join()
        ms_listener.stop()

if __name__ == "__main__":
    print("Controls: WASD/QE/IJKL/R + mouse, ESC to quit")
    # start reader thread
    threading.Thread(target=reader, daemon=True).start()
    start_listeners()
    ser.close()
