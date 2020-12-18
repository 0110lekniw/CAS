import numpy as np
from initial_data import intruderStartingPoint, ownStartingPoint
from seperation_calculator import DistanceCalculator

timeElapsed = 0
intruderPosition = intruderStartingPoint
ownPosition = ownStartingPoint
distance = DistanceCalculator(ownPosition, intruderPosition)
dangerData = np.array([60, 3])
timeStep = 5
dangerZoneTime = 0