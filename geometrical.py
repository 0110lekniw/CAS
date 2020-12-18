import math
import numpy as np

#Vectors

def vectorCalculation(terminal_point, initial_point):
    delta_x = terminal_point[0] - initial_point[0]
    delta_y = terminal_point[1] - initial_point[1]
    modulo = math.sqrt(delta_x**2+delta_y**2)
    return np.array([delta_x, delta_y, modulo])

def degreeBetweenVectors(vector_1, vector_2):
    scalar_product = scalarProductVector(vector_1, vector_2)
    cosinus_degree = scalar_product/(vector_1[2]*vector_2[2])
    degree = math.acos(cosinus_degree)
    return degree

def scalarProductVector(vector_1, vector_2):
    product = vector_1[0]*vector_2[0]+vector_1[1]*vector_2[1]
    return product

# Rotation of the point
def turnAroundPoint(point, angle, origin):
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return [qx, qy]

#Arc/Circle calculations
def CricleFromThreePoints(points):
    [x1, y1] = points[0]
    [x2, y2] = points[1]
    [x3, y3] = points[2]
    ## caclulating the coefficients
    c = (x1 - x2) ** 2 + (y1 - y2) ** 2
    a = (x2 - x3) ** 2 + (y2 - y3) ** 2
    b = (x3 - x1) ** 2 + (y3 - y1) ** 2
    s = 2 * (a * b + b * c + c * a) - (a * a + b * b + c * c)
    x0 = (a * (b + c - a) * x1 + b * (c + a - b) * x2 + c * (a + b - c) * x3) / s
    y0 = (a * (b + c - a) * y1 + b * (c + a - b) * y2 + c * (a + b - c) * y3) / s
    ar = a ** 0.5
    br = b ** 0.5
    cr = c ** 0.5
    R = ar * br * cr / ((ar + br + cr) * (-ar + br + cr) * (ar - br + cr) * (ar + br - cr)) ** 0.5
    circleCoordinates = [x0, y0, R]
    return circleCoordinates

def ArcLengthCalc(R, angle):
    length = angle/2*math.pi*2*math.pi*R
    return length

def TrackArcPoints(circleCenter, numberOfPoints, startPoint, finalPoint):
    tracksCoordinates = np.zeros([numberOfPoints, 2])
    dAngle = degreeBetweenVectors(vectorCalculation(startPoint, circleCenter),
                                  vectorCalculation(finalPoint, circleCenter)) / numberOfPoints
    for i in range(numberOfPoints):
        tracksCoordinates[i, :] = turnAroundPoint(startPoint, dAngle*i, circleCenter)
    return tracksCoordinates

#Line Calculations
def LineFromTwoPoints(points):
    A = points[0]
    B = points[1]
    a = (A[1] - B[1]) / (A[0] - B[0])
    b = A[1] - A[0] * a
    lineCoefficients = [a, b]
    return lineCoefficients

def TrackLinear(linearCoeeficients, numberOfPoints, startPoint, finalPoint):
    tracksCoordinates = np.zeros([numberOfPoints, 2])
    dValue = (finalPoint[0] - startPoint[0])/numberOfPoints
    for i in range(numberOfPoints):
        tracksCoordinates[i, 0] = startPoint[0] + dValue * i
        tracksCoordinates[i, 1] = linearCoeeficients[0]*(startPoint[0]+dValue*i)+linearCoeeficients[1]
    return tracksCoordinates

def DistanceBetweenPoints(pointA, pointB):
    distance = math.sqrt((pointA[0]-pointB[0])**2+(pointA[1]-pointB[1])**2)
    return distance


def LinearQuadraticCommonPoint(linear, quadratic):
    aQuadratic = quadratic[0]
    bQuadratic = (quadratic[1]-linear[0])
    cQuadratic = (quadratic[2] - linear[1])
    differentiator = math.sqrt(bQuadratic**2 - 4*cQuadratic*aQuadratic)
    if differentiator == 0:
        roots = -bQuadratic / (2 * aQuadratic)
        return roots
    elif differentiator > 0:
        root_1 = (-bQuadratic - math.sqrt(differentiator)) / (2 * aQuadratic)
        if root_1 > 500000 or root_1 < 0:
            root_1 = (-bQuadratic + math.sqrt(differentiator)) / (2 * aQuadratic)
        return root_1
    elif differentiator < 0:
        print("No roots in the set of real numbers ")

def LengthOfTheParaboleBetweenPoints(points, parabole):
    a = parabole[0]
    b = parabole[1]
    xFrom = points[0]
    xTo = points[1]
    def DerevativesValueForPoint(point):
        value = 2*a*point+b
        return value
    lengthValue = 1/(4*a)*(math.log((math.sqrt(DerevativesValueForPoint(xTo)**2+1)+DerevativesValueForPoint(xTo)) /
                              (math.sqrt(DerevativesValueForPoint(xFrom)**2+1)+DerevativesValueForPoint(xFrom))) +
                     DerevativesValueForPoint(xTo)*math.sqrt(DerevativesValueForPoint(xTo)**2+1) -
                     DerevativesValueForPoint(xFrom)*math.sqrt(DerevativesValueForPoint(xFrom)**2+1))
    return lengthValue
