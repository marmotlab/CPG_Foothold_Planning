import numpy as np
import math

pi = math.pi


def calculate_a(cpg,cx_new):

    dist = cpg['foothold_offsetY'] - cpg['s1OffsetY']
    leg_offset = np.array([pi / 3, pi / 3, 0, 0, -pi / 3, -pi / 3]) + 0.173 * np.ones([6])
    p = np.tan(leg_offset + cx_new)
    a_cal = np.arctan(((-abs(dist) / cpg['s']) * (1 + p ** 2) * 2 + np.sqrt(
        (4 * abs(dist) ** 2) / (cpg['s'] ** 2) * (1 + p ** 2) ** 2 + 4 * p ** 2)) / (2 * p ** 2))
    return a_cal


def calculate_cx(cpg, x_correction, cx, a):

    dist = cpg['foothold_offsetY'] - cpg['s1OffsetY']
    leg_offset = np.array([pi / 3, pi / 3, 0, 0, -pi / 3, -pi / 3]) + 0.173 * np.ones([6])
    land_point = abs(dist) * np.tan(cx + leg_offset + a)
    cx_f = (np.arctan((land_point + x_correction) / abs(dist)) + np.arctan(
        (land_point + x_correction - cpg['s']) / abs(dist))) / 2
    cx_cal = cx_f - leg_offset
    cx_cal = cx_cal.reshape(6)

    return cx_cal


if __name__ == "__main__":
    cpg = {'s1OffsetY': np.array([0.2375*math.sin(pi/6), -0.2375*math.sin(pi/6), 0.1875, -0.1875, 0.2375*math.sin(pi/6), -0.2375*math.sin(pi/6)])}


    cpg['foothold_offsetY'] = np.array([0.38145, - 0.38145,   0.5125, - 0.5125,   0.33105, - 0.33105])
    cpg['s'] = 0.12
    cx_new = np.array([0, 0, 0, 0, 0, 0]) * math.pi
    a = calculate_a(cpg,cx_new)
    print(a)
    print(0.0267918/pi*180)





















