import math

def DistanceCalculator(knownPosition, alienPosition):
    xDifference = float(knownPosition[0]) - float(alienPosition[0])
    yDifference = float(knownPosition[1]) - float(alienPosition[1])
    distance = math.sqrt(math.pow(xDifference, 2) + math.pow(yDifference, 2))
    return distance


def CollisionTimeCalc(formerDistance, currentDistance, formerTime, currentTime):
    changeOfDistance = formerDistance - currentDistance
    approachingVelocity = changeOfDistance/(currentTime-formerTime)
    timeLeft = currentDistance/approachingVelocity/60
    return timeLeft