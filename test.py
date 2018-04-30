import simulator

simulator = simulator("775Pro", 2.0, 30, 25, 0.0162, 0.020, 165.0, 0)
simulator2 = simulator("Mini CIM", 1.0, 30, 25, 0.0162, 0.020, 50.0, 0)
simulator.simulate(200,1)
simulator2.simulateTime(200,1)
simulator.graph()