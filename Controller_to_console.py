import pygame
from pygame.locals import *

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found")
    exit()

joy = pygame.joystick.Joystick(0)
joy.init()
print("Using controller:", joy.get_name())

while True:
    for event in pygame.event.get():
        if event.type == JOYBUTTONDOWN:
            print("Button", event.button, "pressed")
            # map to send_packet(CMD_KEY, ...)
        elif event.type == JOYBUTTONUP:
            print("Button", event.button, "released")
        elif event.type == JOYAXISMOTION:
            axis = event.axis
            val = event.value  # -1.0 .. 1.0
            print("Axis", axis, "=", val)
            # map joystick movement to CMD_MOVE
