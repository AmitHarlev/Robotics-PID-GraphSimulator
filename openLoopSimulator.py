import matplotlib.pyplot as plt
import keyboard
import pylab as plt
import numpy as np
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




class simulator:
    def __init__(self, motor, motorCount, gearRatio, mass, sprocketRadius, timestep, Kp, Kd):
        self.motorCount = motorCount
        self.gearRatio = gearRatio
        self.mass = mass
        self.sprocketRadius = sprocketRadius
        self.timestep = timestep
        self.Kp = Kp
        self.Kd = Kd

        self.position = 0
        self.velocity = 0
        self.acceleration = 0
        self.voltage = 0

        self.positionGraph = []
        self.velocityGraph = []
        self.accelerationGraph = []
        self.voltageGraph = []
        self.time = []

        self.setMotor(motor)

    def setMotor(self,motor):
        stallCurrent = "stallCurrent"
        stallTorque = "stallTorque"
        freeSpeed = "freeSpeed"
        ratedVoltage = "ratedVoltage"    
        freeCurrent = "freeCurrent"

        motors={
            "775Pro":{
                stallCurrent: 134.0,
                stallTorque: 0.71,
                freeSpeed: 18730.0,
                ratedVoltage: 12.0,
                freeCurrent: 0.7
            },
            "CIM":{
                stallCurrent: 131.0,
                stallTorque: 2.41,
                freeSpeed: 5330.0,
                ratedVoltage: 12.0,
                freeCurrent: 2.7
            },
            "Mini CIM":{
                stallCurrent: 89.0,
                stallTorque: 1.4,
                freeSpeed: 5840.0,
                ratedVoltage: 12.0,
                freeCurrent: 3.0
            }
        }

        freeSpeed = 2*3.141*motors[motor][freeSpeed]/60
        self.stallCurrent = motors[motor][stallCurrent]
        self.stallTorque = motors[motor][stallTorque]
        self.ratedVoltage = motors[motor][ratedVoltage]
        self.freeCurrent = motors[motor][freeCurrent]
        self.motorResistance = self.ratedVoltage/self.stallCurrent
        self.kt = self.stallTorque*self.motorCount/self.stallCurrent
        self.kv = freeSpeed/(self.ratedVoltage-self.motorResistance*self.freeCurrent)

    def getVoltageWithPID(self,Kp,Kd,goal):
        newVoltage = Kp*(goal-self.position)+Kd*(-self.velocity)
        if(newVoltage>self.ratedVoltage):
            newVoltage=self.ratedVoltage
        if(newVoltage < -self.ratedVoltage):
            newVoltage = -self.ratedVoltage
        return newVoltage

    def calculateAcceleration(self,voltage):
        acceleration = -self.kt * self.gearRatio * self.gearRatio * self.velocity/ (self.kv * self.motorResistance * self.sprocketRadius * self.sprocketRadius * self.mass) + self.gearRatio * self.kt * voltage/(self.motorResistance * self.sprocketRadius * self.mass) - 9.8
        return acceleration

    def calculateVelocityChange(self,acceleration, timestep):
        return acceleration*timestep

    def calculatePositionChange(self,velocity, timestep):
        return velocity*timestep

    def simulate(self,cycles,goal):

        accelerationTickUpdate = 0.001
        accelerationTickUpdateCount = int(self.timestep/accelerationTickUpdate)

        for cycle in range(cycles):
            for _ in range(accelerationTickUpdateCount):
                self.updatePhysics(accelerationTickUpdate)
            self.voltage = self.getVoltageWithPID(self.Kp,self.Kd,goal)
            self.pushToGraphs(cycle)

    def startSimulate(self):

        accelerationTickUpdate = 0.001
        accelerationTickUpdateCount = int(self.timestep/accelerationTickUpdate)
        x = 0
        goal = 0

        while (x>-1):

            for _ in range(accelerationTickUpdateCount):
                self.updatePhysics(accelerationTickUpdate)
            self.voltage = self.getVoltageWithPID(self.Kp,self.Kd,goal)
            self.pushToGraphs(x)

            goal = self.updateGoal(goal)
            x += 1

            if(keyboard.is_pressed('enter')):
                x = -1


    def updateGoal(self,goal):

        # pygame.event.pump()
        # return controller.getBottomAxis(2)


        scale = 1.1/9.0
        try: 
            if keyboard.is_pressed('1'):
                return scale*1
            elif keyboard.is_pressed('2'):
                return scale*2
            elif keyboard.is_pressed('3'):

                return scale*3
            elif keyboard.is_pressed('4'):
                return scale*4
            elif keyboard.is_pressed('5'):
                return scale*5
            elif keyboard.is_pressed('6'):
                return scale*6
            elif keyboard.is_pressed('7'):
                return scale*7
            elif keyboard.is_pressed('8'):
                return scale*8
            elif keyboard.is_pressed('9'):
                return scale*9
            else:
                return goal
        except:
            return goal

    def updatePhysics(self,accelerationTickUpdate):
        self.position += self.calculatePositionChange(self.velocity,accelerationTickUpdate)
        self.velocity += self.calculateVelocityChange(self.acceleration,accelerationTickUpdate)
        self.acceleration = self.calculateAcceleration(self.voltage)
    
    def pushToGraphs(self,cycle):
        # self.positionGraph.append(self.position)
        # self.velocityGraph.append(self.velocity)
        # self.accelerationGraph.append(self.acceleration)
        # self.voltageGraph.append(self.voltage)
        # self.time.append(cycle*self.timestep)

        for i in range(250):
            if(i != 249):
                self.Y[i] = self.Y[i+1]
            else:
                self.Y[249] = self.position
            

        self.graph.set_ydata(self.Y)
        plt.draw()
        plt.pause(0.01)

    def startGraph(self):

        self.Y = np.zeros(250)
        switch = np.full(250,0.25)
        scale = np.full(250,0.7)

        self.Y[248] = 1.0
        self.Y[249] = 0.5

        plt.ion()
        self.graph = plt.plot(self.Y)[0]
        plt.plot(switch,'r--')
        plt.plot(scale,'r--')

        # plt.subplot(411)
        # plt.plot(self.time,self.positionGraph)
        # plt.ylabel("Position [m]")
        # plt.subplot(412)
        # plt.plot(self.time,self.velocityGraph)
        # plt.ylabel("Velocity [m/s]")
        # plt.subplot(413)
        # plt.plot(self.time,self.accelerationGraph)
        # plt.ylabel("Acceleration [m/s/s]")
        # plt.subplot(414)
        # plt.plot(self.time,self.voltageGraph)
        # plt.ylabel("Voltage [V]")
        # plt.xlabel("Time[s]")
        # plt.show()

# controller = controller()
simulator = simulator("775Pro", 2.0, 20, 25, 0.0162, 0.020, 135.0, 1)
#simulator.simulate(2000,1.1)
simulator.startGraph()
simulator.startSimulate()
