import math

#air data
gasouesConstant = 287
adiabaticIndex = 1.4
altitude = 16000
T0 = 288.15
temperature = T0 - 6.5/1000*altitude
speedOfSound = math.sqrt(gasouesConstant*adiabaticIndex*temperature)

#initial data of the intruder plane
intruderMachNumber = 1.0
intruderVelocity = intruderMachNumber*speedOfSound

#intruder path points
intruderStartingPoint = [500000, 500000]
intruderMiddlePoint = [295000, 341000]
intruderFinalPoint = [0, 0]

#initial data of own plane
ownMachNumber = 0.9
ownVelocity = ownMachNumber * speedOfSound

#own path points
ownStartingPoint = [0, 400000]
ownFinalPoint = [500000, 300000]

#seperation
safetyCoefficient = 3
LoSM = 5