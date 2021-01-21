import matplotlib.pyplot as plt
import numpy as np
import get_position
import geometrical
import math
from scipy import optimize
from geometrical import LinearQuadraticCommonPoint
from initial_data import ownStartingPoint, intruderStartingPoint, safetyCoefficient, LoSM, ownFinalPoint, ownVelocity
from track_generator import linear_coordinates, lineTrack, arcTrack
from seperation_calculator import DistanceCalculator, CollisionTimeCalc


def UpdateInputData(input, timeStep):
    updateData = []
    updateData.append(input[0]+timeStep)
    updateData.append(get_position.Own(updateData[0])[0])
    updateData.append(get_position.Own(updateData[0])[1])
    updateData.append(get_position.Intruder(updateData[0])[0])
    updateData.append(get_position.Intruder(updateData[0])[1])
    updateData.append(DistanceCalculator(updateData[1:3], updateData[3:5]))
    return updateData

def VelocityCalculator(pointOne, pointTwo, time):
    traveldDistance = geometrical.DistanceBetweenPoints(pointOne, pointTwo)
    velocity = traveldDistance/time
    return velocity

def main(timeStep = 5, timeElapsed = 0, dangerZoneTime = 0,  intruderPosition = intruderStartingPoint,
         ownPosition=ownStartingPoint):

# creating the plot area
    fig = plt.figure(figsize=(15, 5))
    ax = plt.subplot(1, 3, 1)
    az = plt.subplot(1, 3, 2)
    ay = plt.subplot(1, 3, 3)
    ax.set(xlim=(0, 500000), ylim=(0, 500000))

# initiation - inital data defined in file initial_data
    distance = DistanceCalculator(ownPosition, intruderPosition)
    inputData = np.array([timeElapsed, ownPosition[0], ownPosition[1], intruderPosition[0], intruderPosition[1],
                          distance], dtype=float)
    outputData = []
# defining an array for possible danger
    dangerData = []
    collisionPoint = ownStartingPoint
    maneouverPoints = np.zeros([4, 2])
    previousDistance = 1000000
    previousTimeToCollision = 0
    newManeouverPoints = np.zeros((150, 2))

    while(inputData[1]<ownFinalPoint[0]):
        # updating the data after time = timeStep
        formerInput = inputData
        inputData = UpdateInputData(inputData, timeStep)
        # calculating LoSM
        timeToCollision = CollisionTimeCalc(formerInput[5], inputData[5], formerInput[0], inputData[0])
        print("Danger of Collision in :" + str(abs(timeToCollision)))
        condition = timeToCollision - previousTimeToCollision > 0
        # check if the aircrafts if the seperation is sufficient
        if condition:
            if timeToCollision < LoSM * safetyCoefficient:
                if round(timeToCollision, 2) == 0:
                    print("BUM")
                    break
                # increasing the frequency of signal receiving
                timeStep = 1
                # record the position of the intruder
                dangerData.append([inputData[3], inputData[4], inputData[5]])
                dangerZoneTime += 1
                if (dangerZoneTime/30).is_integer():
                    dangerDataArray = np.array(dangerData)
                    # use of recorded data to predict the trajectory with the quadratic extrapolation
                    predictedTrajectory = np.polyfit(dangerDataArray[:, 0], dangerDataArray[:, 1], 2)
                    startingX = int(round(dangerDataArray[0, 0]))
                    # calculating collision point as the intersection of known own extrapolated trajectory
                    linearQuadraticCoefficients = np.poly1d([predictedTrajectory[0], predictedTrajectory[1] -
                                                             linear_coordinates[0], predictedTrajectory[2]-
                                                             linear_coordinates[1]])
                    newCollisionPointECoordinate = np.min(optimize.fsolve(linearQuadraticCoefficients,
                                                                          np.array([startingX+100000])))
                    newCollisionPoint = [newCollisionPointECoordinate,
                                         linear_coordinates[0]*(newCollisionPointECoordinate)+linear_coordinates[1]]
                    #upadate the trajectory, if it move toward own plane
                    if round(geometrical.DistanceBetweenPoints(collisionPoint, ownStartingPoint), 2) == 0:
                        collisionPoint = newCollisionPoint
                    elif round(geometrical.DistanceBetweenPoints(collisionPoint, ownStartingPoint), 2) > \
                            round(geometrical.DistanceBetweenPoints(newCollisionPoint, ownStartingPoint), 2):
                        collisionPoint = newCollisionPoint
                    del newCollisionPoint
                    #calculateIntruderTheTimeToCollisionPoint
                    intruderVelocity = VelocityCalculator(dangerDataArray[-1, :2], dangerDataArray[-2, :2], timeStep)
                    intruderDistanceToCP = abs(geometrical.LengthOfTheParaboleBetweenPoints([inputData[4],
                                                                                         collisionPoint[0]],
                                                                                        predictedTrajectory))

                    intruderTimeToCP = intruderDistanceToCP/intruderVelocity
    #                   intruderDistanceToCP2 = geometrical.LengthOfAPolynolam(inputData[4], collisionPoint[0], 10,
    #                                                                           predictedTrajectory)
    #                   intruderTimeToCP2 = intruderDistanceToCP2 / intruderVelocity

                    #calculate own time to Collison Point
                    ownDistanceToCP = abs(geometrical.DistanceBetweenPoints(collisionPoint, inputData[1:3]))
                    ownTime = ownDistanceToCP/ownVelocity
                    if  ownDistanceToCP - previousDistance > 0:
                        break
                    #calculate the difference between times:
                    differenceBetweenTime = intruderTimeToCP - ownTime
                    print("Time difference: " + str(round(differenceBetweenTime, 3)) + "s")
                    if differenceBetweenTime > 0:
                        print("descend")
                    else:
                        R = ownVelocity*LoSM*60
                        ownd = 2*R
                        distanceToCP = abs(differenceBetweenTime*ownVelocity)
                        d = ownd - distanceToCP
                        cosAlpha = (R**2-(d**2+ownd**2))/(-2*d*ownd)
                        Alpha = np.arccos(cosAlpha)
                        print(Alpha)
                        coefficient = linear_coordinates[0]
                        cosBeta = 1
                        if coefficient < 0:
                            cosBeta = math.cos( - math.atan(coefficient))
                        elif coefficient > 0:
                            cosBeta = math.cos(math.atan(coefficient))
                        sign = 1
                        if inputData[1] < formerInput[1]:
                            sign = -1
                        deltaX1 = distanceToCP*cosBeta
                        deltaX2 = 2*R
                        maneouverPoints[1, :] = ([collisionPoint[0]-deltaX1*sign, (collisionPoint[0]-deltaX1*sign)*
                                                          linear_coordinates[0]+linear_coordinates[1]])
                        #divergent point
                        maneouverPoints[0, :] = ([collisionPoint[0]-deltaX2*sign, (collisionPoint[0]-deltaX2*sign)*
                                           linear_coordinates[0]+linear_coordinates[1]])
                        #safetyzone new point
                        rotated = geometrical.turnAroundPoint(maneouverPoints[1, :], Alpha, maneouverPoints[0, :])
                        maneouverPoints[1, :] = rotated
                        #maneouver wayback points
                        axisOfSymetry = geometrical.PerpendicularLineFromPoint(linear_coordinates[0], collisionPoint)
                        maneouverPoints[2:4, :] = geometrical.MirrorByAxis(np.vstack((maneouverPoints[0],
                                                                                     maneouverPoints[1])),
                                                                          axisOfSymetry)
                        
                        #calculating the coefficients of the first function
                        x1 = maneouverPoints[0][0]
                        x2 = maneouverPoints[1][0]
                        y1 = maneouverPoints[0][1]
                        y2 = maneouverPoints[1][1]
                        A = np.array([[1/2*x1, 1, 0], [x1**2, x1, 1], [x2**2, x2, 1]])
                        B = np.array([[linear_coordinates[0]], [y1], [y2]])
                        del x1, x2, y1, y2
                        X = np.linalg.solve(A, B).tolist()
                        C = [X[0][0], X[1][0], X[2][0]]
                        del A, B
                        function_one = np.poly1d(C)
                        del X, C

                        # calculating the coefficients of the second function
                        x1 = maneouverPoints[2][0]
                        x2 = maneouverPoints[3][0]
                        y1 = maneouverPoints[2][1]
                        y2 = maneouverPoints[3][1]
                        A = np.array([[1 / 2 * x2, 1, 0], [x1 ** 2, x1, 1], [x2 ** 2, x2, 1]])
                        B = np.array([[linear_coordinates[0]], [y1], [y2]])
                        del x1, x2, y1, y2
                        X = np.linalg.solve(A, B).tolist()
                        C = [X[0][0], X[1][0], X[2][0]]
                        del A, B
                        function_two = np.poly1d(C)
                        del X, C

                        #calculating the coefficients of the third function
                        x1 = maneouverPoints[2][0]
                        x2 = maneouverPoints[1][0]
                        y1 = maneouverPoints[2][1]
                        A = np.array([[1/2*x1, 1, 0], [1/2*x2, 1, 0], [x1**2, x1, 1]])
                        B = np.array([[1/2*function_two[0]*x1+function_two[1]], [1/2*function_one[0]*x2+function_one[1]], [y1]])
                        del x1, x2, y1
                        X = np.linalg.solve(A, B).tolist()
                        C = [X[0][0], X[1][0], X[2][0]]
                        del A, B
                        function_three = np.poly1d(C)
                        del X, C
                        points_one = np.linspace(maneouverPoints[0][0], maneouverPoints[1][0], 50)
                        for i in range(points_one.shape[0]):
                            newManeouverPoints[i, :] = [points_one[i], function_one(points_one[i])]
                        points_two = np.linspace(maneouverPoints[2][0], maneouverPoints[3][0], 50)
                        for i in range(points_two.shape[0]):
                            newManeouverPoints[50+i, :] = [points_two[i], function_two(points_two[i])]
                        points_three = np.linspace(maneouverPoints[1][0], maneouverPoints[2][0], 50)
                        for i in range(points_three.shape[0]):
                            newManeouverPoints[100+i, :] = [points_three[i], function_three(points_three[i])]
                        print (newManeouverPoints)

        previousTimeToCollision = timeToCollision
        outputData.append([inputData[0], inputData[1], inputData[2], inputData[3], inputData[4], inputData[5],
                           timeToCollision, collisionPoint[0], collisionPoint[1]])

        if timeToCollision > LoSM*safetyCoefficient:
            timeStep = 5

    outputArray = np.array(outputData)
    ax.scatter(outputArray[-1, -2], outputArray[-1, -1], c="r")
    ax.scatter(maneouverPoints[:, 0], maneouverPoints[:, 1], c="blue")
    ax.plot(newManeouverPoints[:50, 0], newManeouverPoints[:50, 1], c="red")
    ax.plot(newManeouverPoints[50:100, 0], newManeouverPoints[50:100, 1], c="green")
    ax.plot(newManeouverPoints[100:151, 0], newManeouverPoints[100:151, 1], c="pink")
    draw_circle = plt.Circle((outputArray[-1, -2], outputArray[-1, -1]), ownVelocity*LoSM*60, fill=False)
    ax.add_artist(draw_circle)
    ax.plot(outputArray[:, 1], outputArray[:, 2], c="green")
    ax.plot(lineTrack[:, 0], lineTrack[:, 1], '--', c="green")
    ax.plot(arcTrack[:, 0], arcTrack[:, 1], '--', c="royalblue")
    ax.plot(arcTrack[:, 0], np.poly1d(predictedTrajectory)(arcTrack[:, 0]), '--', c="darkorange")
    ax.plot(outputArray[:, 3], outputArray[:, 4], c="k")
    az.plot(outputArray[:, 0], outputArray[:, 5], c="k")
    ay.plot(outputArray[:, 0], outputArray[:, 6], c="k")
    plt.show()





main()












