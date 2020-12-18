import math

import geometrical
from initial_data import ownVelocity, ownStartingPoint, intruderVelocity, intruderStartingPoint
from track_generator import linear_coordinates, circle_coordinates


def Own(time):
    alpha = math.atan(linear_coordinates[0])
    eastVelocity = ownVelocity * math.cos(alpha)
    eastPositon = ownStartingPoint[0] + eastVelocity * time
    currentPostion = [eastPositon, eastPositon * linear_coordinates[0] + linear_coordinates[1]]
    return currentPostion


def Intruder(time):
    intruderAngularPosition = (intruderVelocity / circle_coordinates[2]) * time
    currentPosition = geometrical.turnAroundPoint(intruderStartingPoint, intruderAngularPosition,
                                                  circle_coordinates[:2])
    return currentPosition