import pygame, sys
from time import sleep

class controller:

    def __init__(self):
        # setup the pygame window
        pygame.init()
        window = pygame.display.set_mode((200, 200), 0, 32)

        # how many joysticks connected to computer?
        joystick_count = pygame.joystick.get_count()
        print(joystick_count)
        print("There is " + str(joystick_count) + " joystick/s")

        if joystick_count == 0:
            # if no joysticks, quit program safely
            print("Error, I did not find any joysticks")
            pygame.quit()
            sys.exit()
        else:
            # initialise joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

    def getBottomAxis(self,number):
        adjustedValue = (self.joystick.get_axis(number) - 1)/-2
        print("Axis value is %s" %(adjustedValue))
        return adjustedValue


controller = controller()
while True:
    pygame.event.pump()
    print(controller.joystick.get_axis(2))