# pylint: disable=invalid-name
# pylint: disable=c0116
# pylint: disable=c0114

import numpy as np
from shapely.geometry import Polygon, Point
from obstacle import *
from math import log10, pi, sqrt
from gekko import GEKKO


class UE:
    """for define the UEs"""

    def __init__(self, UE_id, x, y, z, snr):  # , traffic
        self.id = UE_id
        self.x = x
        self.y = y
        self.z = z
        # self.traffic = traffic
        self.snr = snr


K = -20 * log10((4 * pi) / (3 * 10 ** 8)) - 20 * log10(5250 * 10 ** 6) - (-85)
# print('K=', K)


def calculate_radius(PT, snr):
    return 10 ** ((K + PT - snr) / 20)


file = open('UEs', 'r')
lines = file.readlines()
file.close()
# print(lines)
UEs = []
FirstAngle = []
print(MaxHight)

for line in lines:
    print(line)
    l = line.split(',')
    if len(l) == 5:
        UEs.append(UE(int(l[0]), float(l[1]), float(l[2]),
                      float(l[3]), float(l[4])))
        point = Point(float(l[1]), float(l[2]))
        dist = calculate_distace(poly, point)
        # print(dist)
        angle = calculate_angle(MaxHight, dist)
        FirstAngle.append(angle)

print(FirstAngle)
# print(poly)
# for f in FAPS:
#     point = Point(f.x, f.y)
#     dist = calculate_distace(poly, point)
#     angle = calculate_angle(MaxHight, dist)
#     FirstAngle.append(angle)

MaxFirstAngel = max(FirstAngle)
# print(MaxFirstAngel)


def solve_equation(PT, users):
    m = GEKKO(remote=False)

    x = m.Var()
    y = m.Var()
    z = m.Var()

    for user in users:
        x_term = (x - user.x) ** 2
        y_term = (y - user.y) ** 2
        z_term = (z - user.z) ** 2
        radius_term = calculate_radius(PT, user.snr) ** 2
        # print(radius_term)
        CheckAngle = m.atan(z/m.sqrt(x_term + y_term))
        # print(m.atan(CheckAngle))
        # z > 5 to ensure that the UAV is at a reasonable height
        m.Equations([x_term + y_term + z_term <= radius_term,
                    z > MaxHight, CheckAngle >= FirstAngle[user.id]])
        # print(FirstAngle[user.id])

    try:
        m.solve(disp=False)
        solution = [x.value[0], y.value[0], z.value[0]]

    except:
        solution = None

    return solution


def UAVP(users):
    #  Transmission Power (dBm)
    PT = 0

    while True:
        solution = solve_equation(PT, users)
        #   print(solution, PT)

        if solution != None:
            # for ue in UEs:
            # CheckAngle = (
            #     solution[2])/(((solution[0]-ue.x)**2+(solution[1]-ue.y)**2)**0.5)
            # UavAngle = np.arctan(CheckAngle)
            # if UavAngle > MaxFirstAngel:
            return PT, solution

        else:
            PT += 1


print(UAVP(UEs))
