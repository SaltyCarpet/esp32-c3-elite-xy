import serial
import struct
import threading
import queue
import pygame
from pygame.locals import *

# --- Serial setup ---
PORT = 'COM3'   # Change to your ESP32 port
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=0.1)

# --- Protocol ---
HEADER = 0xAA

send_q = queue.Queue()

def send_packet(key_code, value):
    # Build payload: header (1), key (1), float (4)
    payload = struct.pack('<BBf', HEADER, key_code, value)

    # Compute checksum over first 6 bytes
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

# --- Controller setup ---
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found")
    exit()

joy = pygame.joystick.Joystick(0)
joy.init()
print("Using controller:", joy.get_name())

# --- Event handlers ---
def handle_button(event):
    if event.type == JOYBUTTONDOWN:
        print("Button", event.button, "pressed")
        send_packet(event.button, 1.0)   # pressed = 1.0
    elif event.type == JOYBUTTONUP:
        print("Button", event.button, "released")
        send_packet(event.button, 0.0)   # released = 0.0

def handle_axis(event):
    axis = event.axis
    val = float(event.value)  # -1.0 .. 1.0
    print("Axis", axis, "=", val)
    send_packet(axis + 150, val)  # offset axis codes to avoid clash with buttons

# --- Main loop ---
if __name__ == "__main__":
    threading.Thread(target=writer, daemon=True).start()
    threading.Thread(target=reader, daemon=True).start()

    print("Controller active. ESC to quit.")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type in [JOYBUTTONDOWN, JOYBUTTONUP]:
                handle_button(event)
            elif event.type == JOYAXISMOTION:
                handle_axis(event)
            elif event.type == KEYDOWN and event.key == K_ESCAPE:
                running = False

    ser.close()
    pygame.quit()
