import numpy as np
import math

def groundIK_cf(cpg,dist):
    Leglength = 0.325
    z = np.zeros(6)
    z[0] = -(math.pi - (math.asin((abs(0.35) / math.cos(math.pi / 3 - cpg['legs'][0, 0]) - Leglength * math.cos(
        cpg['legs'][0, 1])) / Leglength) + math.pi / 2 - cpg['legs'][0, 1]))
    z[1] = math.pi - (math.asin((abs(0.35) / math.cos(math.pi / 3 + cpg['legs'][0, 3]) - Leglength * math.cos(
        cpg['legs'][0, 4])) / Leglength) + math.pi / 2 - cpg['legs'][0, 4])
    z[2] = -(math.pi - (math.asin((abs(0.35) / math.cos(cpg['legs'][0, 6]) - Leglength * math.cos(
        cpg['legs'][0, 7])) / Leglength) + math.pi / 2 - cpg['legs'][0, 7]))
    z[3] = math.pi - (math.asin((abs(0.35) / math.cos(cpg['legs'][0, 9]) - Leglength * math.cos(
        cpg['legs'][0, 10])) / Leglength) + math.pi / 2 - cpg['legs'][0, 10])
    z[4] = -(math.pi - (
                math.asin((abs(dist[4]) / math.cos(math.pi / 3 - 0.28 + cpg['legs'][0, 12]) - Leglength * math.cos(
                    cpg['legs'][0, 13])) / Leglength) + math.pi / 2 - cpg['legs'][0, 13]))
    z[5] = math.pi - (
                math.asin((abs(dist[5]) / math.cos(math.pi / 3 - 0.28 - cpg['legs'][0, 15]) - Leglength * math.cos(
                    cpg['legs'][0, 16])) / Leglength) + math.pi / 2 - cpg['legs'][0, 16])

    return z