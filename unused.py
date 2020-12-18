### for the purpose of this program all the coordinates position in array were assumed as follow [x, y, z, t]


def UpdateInput(positions, newData):
    newPositions = positions
    newPositions[0, :] = positions[1, :]
    newPositions[1, :] = newData[:]
    return newPositions

def VelocityCalculator(positions):
    xChange = positions[1, 0] - positions[0, 0]
    yChange = positions[1, 1] - positions[0, 1]
    zChange = positions[1, 2] - positions[0, 2]
    tChange = positions[1, 3] - positions[0, 3]
    velocityArray = np.array([xChange/tChange, yChange/tChange, zChange/tChange])
    return velocityArray

def HorizontalSeperation(ownPosition, alienPosition):
    horizontalDistance = (math.sqrt(math.pow(ownPosition[0]-alienPosition[0], 2) +
                                    math.pow(ownPosition[1]-alienPosition[1], 2)))
    return horizontalDistance

def ImportData(txt_file, delimiter):
    data = [i.strip('\n').split(delimiter) for i in open(txt_file)]
    arrayOfData = np.array(data)
    return arrayOfData


# Generate3Dtrack([0, 5000], [500000, 480000], "parabolic",)
#
# simulationData = np.array(ImportData('', '\t'))
# safetyCoefficient = 2
#
# timeValue = 0
# distances = []
# collisionTimes = []
#
# for row in simulationData:
#     timeValue += 1
#     distance = round(DistanceCalculator(row[:3], row[3:]), 3)
#     distances.append([distance, timeValue])
#     if len(distances)>1:
#         collisionTime = CollisionTimeCalc(distances[timeValue - 2][0], distances[timeValue - 1][0],
#                                           distances[timeValue-2][1], distances[timeValue-1][1])
#         if collisionTime < 15*safetyCoefficient:
#
#     else: collisionTime = 20
#     collisionTimes.append([collisionTime, timeValue])
#     #print(timeValue, distance, collisionTime)
#     #time.sleep(0.05)
#
# distancesArray = np.array(distances)
# collisionTimesArray = np.array(collisionTimes)

fig = plt.figure()
ax = plt.subplot(1, 2, 1)
ax.plot(arcTrack[:, 0], arcTrack[:, 1])
ax.plot(lineTrack[:, 0], lineTrack[:, 1])

plt.show()