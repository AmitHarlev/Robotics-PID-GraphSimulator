import matplotlib.pyplot as plt
from tkinter import Tk, Label, Button, Entry, OptionMenu, StringVar

_acceleration = 0.0
_velocity = 0.0
_position = 0.0

carriageMass = 27/2.2
elevatorMass = 8/2.2
effectiveMass = 2*carriageMass+elevatorMass
gearRatio = 30.0
sprocketRadius = 0.0162
voltage = 0.0
timeStep = 0.020
gravity = 0.0
time = []
positionArray=[]
velocityArray=[]
accelerationArray=[]
voltageArray=[]
currentArray=[]


def PID(position,velocity,goal):
    kp = 165.0 #165
    kd = 0.0
    newVoltage = kp*(goal-position)+kd*(-velocity)
    if(newVoltage>ratedVoltage):
        newVoltage=ratedVoltage
    if(newVoltage < -ratedVoltage):
        newVoltage = -ratedVoltage
    return newVoltage

def getAcceleration(newVoltage):
    acceleration = -kt * gearRatio * gearRatio * _velocity/ (kv * motorResistance * sprocketRadius * sprocketRadius * effectiveMass) + gearRatio * kt * voltage/(motorResistance * sprocketRadius * effectiveMass) - 9.8
    #acceleration = kt/(sprocketRadius*effectiveMass*motorResistance*gearRatio)*(newVoltage+(sprocketRadius*effectiveMass*gravity*motorResistance*gearRatio)/(kt)*-_velocity/(sprocketRadius*kv))
    return acceleration

def calculateVelocityChange(acceleration,timeStep):
    return acceleration*timeStep

def calculatePosition(velocity,timeStep):
    return _position + velocity*timeStep

first = True

def simulate(cycles,timeStep,goal):
    global voltage,_position,_velocity,_acceleration, first
    goal = goal/2
    for i in range(cycles):
        accelerationTickUpdate = 0.001
        accelerationTickUpdateCount = int(timeStep/accelerationTickUpdate)
        for j in range(accelerationTickUpdateCount):
            _position = calculatePosition(_velocity,accelerationTickUpdate)
            _velocity += calculateVelocityChange(getAcceleration(voltage),accelerationTickUpdate)
            _acceleration = getAcceleration(voltage)
        voltage = PID(_position,_velocity,goal)
        positionArray.append(_position)
        velocityArray.append(_velocity)
        accelerationArray.append(_acceleration)
        voltageArray.append(voltage)
        currentArray.append(calculateCurrent(voltage,_velocity))
        time.append(i*timeStep)
        if(abs(goal-_position)<=0.02 and _velocity <=0.03 and first):
            print(_position,time[-1])
            first = False

def clearPlots():
    global positionArray, velocityArray, accelerationArray, currentArray, voltageArray
    positionArray=[]
    velocityArray=[]
    accelerationArray=[]
    voltageArray=[]
    currentArray=[]
    print("WORKING")
    print(positionArray)
    

def calculateCurrent(voltage,velocity):
    #global sprocketRadius, kv, motorResistance
    current = (voltage-velocity/(sprocketRadius*kv))/motorResistance
    return current

def simulateTime(time,timeStep,goal):
    simulate(int(time/timeStep),timeStep,goal)

class mainGUI:
    def __init__(self, master):
        self.master = master
        master.title("Set Constants to Test")

        self.label = Label(master, text="This is our first GUI!")
        self.label.pack()

        self.motor = StringVar(master)
        self.motor.set("CIM") # default value

        MotorSelector = OptionMenu(master, self.motor, "CIM", "775Pro", "Mini CIM")
        MotorSelector.pack()

        self.timeInput = Entry(master)
        self.timeInput.pack()

        self.simulate_button = Button(master, text="Simulate", command=self.runSimulate)
        self.simulate_button.pack()

        self.close_button = Button(master, text="Exit", command=master.quit)
        self.close_button.pack()

    def runSimulate(self):
        global motorName, freeSpeed
        plt.gcf().clear()
        clearPlots()
        setMotorValues(self.motor.get())
        simulateTime(float(self.timeInput.get()),timeStep,2.2)
        print(freeSpeed)
        print(positionArray)
        plt.subplot(511)
        plt.plot(time,positionArray)
        plt.ylabel("Position [m]")
        #,time,velocityArray,time,accelerationArray,linewidth=2.0)
        plt.subplot(512)
        plt.plot(time,velocityArray)
        plt.ylabel("Velocity [m/s]")
        plt.subplot(513)
        plt.plot(time,accelerationArray)
        plt.ylabel("Acceleration [m/s/s]")
        plt.subplot(514)
        plt.plot(time,voltageArray)
        plt.ylabel("Voltage [V]")
        plt.subplot(515)
        plt.plot(time,currentArray)
        plt.ylabel("Current [Ohms]")
        plt.xlabel("Time[s]")
        #plt.xlabel("Time[s]")
        plt.show()


root = Tk()
my_gui = mainGUI(root)
root.mainloop()