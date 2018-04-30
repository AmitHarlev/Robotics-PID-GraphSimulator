import matplotlib.pyplot as plt

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
    
    def updatePhysics(self,accelerationTickUpdate):
        self.position += self.calculatePositionChange(self.velocity,accelerationTickUpdate)
        self.velocity += self.calculateVelocityChange(self.acceleration,accelerationTickUpdate)
        self.acceleration = self.calculateAcceleration(self.voltage)
    
    def pushToGraphs(self,cycle):
        self.positionGraph.append(self.position)
        self.velocityGraph.append(self.velocity)
        self.accelerationGraph.append(self.acceleration)
        self.voltageGraph.append(self.voltage)
        self.time.append(cycle*self.timestep)

    def graph(self):
        plt.subplot(411)
        plt.plot(self.time,self.positionGraph)
        plt.ylabel("Position [m]")
        plt.subplot(412)
        plt.plot(self.time,self.velocityGraph)
        plt.ylabel("Velocity [m/s]")
        plt.subplot(413)
        plt.plot(self.time,self.accelerationGraph)
        plt.ylabel("Acceleration [m/s/s]")
        plt.subplot(414)
        plt.plot(self.time,self.voltageGraph)
        plt.ylabel("Voltage [V]")
        plt.xlabel("Time[s]")
        plt.show()

simulator = simulator("Mini CIM", 1.0, 20, 25, 0.0162, 0.020, 165.0, 0)
simulator.simulate(2000,1.1)
simulator.graph()
