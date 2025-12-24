import pygame
pygame.init()
pygame.joystick.init()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Name: {joystick.get_name()}")
print(f"Axes: {joystick.get_numaxes()}")
print(f"Buttons: {joystick.get_numbuttons()}")
# print(f"Hats: {joystick.get_numhats()}")

while True:
    pygame.event.pump()
    # for i in range(joystick.get_numaxes()):
    #     print(f"Axis {i}: {joystick.get_axis(i):.3f}", end="  ")
    for i in range(joystick.get_numbuttons()):
        print(f"Button {i}: {joystick.get_button(i)}", end="  ")
    # for i in range(joystick.get_numhats()):
    #     print(f"Hat {i}: {joystick.get_hat(i)}", end="  ")
    print("\r", end="")