# pylint: disable=invalid-name
# pylint: disable=c0116
# pylint: disable=c0114


from math import log10, pi
import numpy as np
from shapely.geometry import Polygon, Point


class OBSTACLE:
    """for define the FAPs"""

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


file = open('obstacle', 'r')
lines2 = file.readlines()
file.close()
ObstacleCoordinates = []
BuildingHights = []
POLY = []

for line in lines2:
    l = line.split(',')
    if len(l) == 3:
        ObstacleCoordinates.append(
            OBSTACLE(float(l[0]), float(l[1]), float(l[2])))
        BuildingHights.append(float(l[2]))
        POLY.append((float(l[0]), float(l[1])))
poly = Polygon(POLY)
MaxHight = max(BuildingHights)

# print(POLY[0])
# print(BuildingHights[0], BuildingHights[1], BuildingHights[2])
# for A in ObstacleCoordinates:
#     POLY.append((A.x, A.y))


def calculate_distace(obs_poly, point):
    dist = obs_poly.distance(point)
    return dist


def calculate_angle(Hight, Distance):
    calculated_angle = np.arctan(Hight/Distance)
    # calculated_angle = np.rad2deg(calculated_angle)
    return calculated_angle


# , m.atan(CheckAngle) > MaxFirstAngel
