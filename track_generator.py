import geometrical
from initial_data import intruderStartingPoint, intruderMiddlePoint, intruderFinalPoint, ownStartingPoint, ownFinalPoint

#generating the arc track of the intruder plane
circle_coordinates = geometrical.CricleFromThreePoints([intruderStartingPoint, intruderMiddlePoint, intruderFinalPoint])
arcTrack = geometrical.TrackArcPoints(circle_coordinates[:2], 1000, intruderStartingPoint, intruderFinalPoint)

#generating the linear track of the own plane

linear_coordinates = geometrical.LineFromTwoPoints([ownStartingPoint, ownFinalPoint])
lineTrack = geometrical.TrackLinear(linear_coordinates, 500, ownStartingPoint, ownFinalPoint)