# pylint: disable=invalid-name
# pylint: disable=c0116
# pylint: disable=c0114


from math import log10, pi
from gekko import GEKKO
from CalculateAngle import calculate_angle


class FAP:
    """for define the FAPs"""

    def __init__(self, fap_id, x, y, z, traffic, snr):
        self.id = fap_id
        self.x = x
        self.y = y
        self.z = z
        self.traffic = traffic
        self.snr = snr


ObstacleHight = float(input("the Hight of obstacle:"))
K = -20 * log10((4 * pi) / (3 * 10 ** 8)) - 20 * log10(5250 * 10 ** 6) - (-85)
print('K=', K)


def calculate_radius(PT, snr):
    return 10 ** ((K + PT - snr) / 20)


file = open('FAP', 'r')
lines = file.readlines()
file.close()
# print(lines)
FAPS = []
for line in lines:
    # print(line)
    l = line.split(',')
    FAPS.append(FAP(l[0], int(l[1]), int(l[2]), int(l[3]), int(l[4]), int(l[5])))

FirstAngle = []
for f in FAPS:
    FirstAngleCalculation = calculate_angle(ObstacleHight, f.y)
    FirstAngle.append(float(FirstAngleCalculation))

print(FirstAngle)


def solve_equation(PT, faps):
    m = GEKKO(remote=False)

    x = m.Var()
    y = m.Var()
    z = m.Var()

    for fap in faps:
        x_term = (x - fap.x) ** 2
        y_term = (y - fap.y) ** 2
        z_term = (z - fap.z) ** 2
        radius_term = calculate_radius(PT, fap.snr) ** 2

        # z > 5 to ensure that the UAV is at a reasonable height
        m.Equations([x_term + y_term + z_term <= radius_term, z > 5.0])

    try:
        m.solve(disp=False)
        solution = (x.value[0], y.value[0], z.value[0])

    except:
        solution = None

    return solution
