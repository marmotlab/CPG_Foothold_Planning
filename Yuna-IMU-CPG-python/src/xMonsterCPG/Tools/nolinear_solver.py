import numpy as np
from scipy.optimize import minimize
import sys
sys.path.append('/home/marmot/elevation_ws/src/Yuna-IMU-CPG-python/src/xMonsterCPG/')

from setup.xMonsterKinematics import *
xmk = HexapodKinematics()

pi = np.pi
cpg = {}
cpg['legs'] = np.array([1,18])
cpg['foothold_offsetY'] = np.array([0.40145, - 0.40145, 0.5125, - 0.5125, 0.32105, - 0.32105])


def GIK_error(cpg,x):
    legs = np.copy(cpg['legs'])
    legs[0][17] = x
    leg_5_y = cpg['xmk'].getLegPositions_leg(5, legs)[1]
    leg_5_dy = cpg['foothold_offsetY'][5]
    error = leg_5_y - leg_5_dy
    print('error',error)
    return error

def fun(cpg):
    v = lambda x: GIK_error(cpg, x)
    return v

def con(args):
    xmin, xmax = args
    cons = ({'type': 'ineq', 'fun': lambda x: x - xmin}, \
            {'type': 'ineq', 'fun': lambda x: x + xmax})
    return cons
#  using slover package

x0 = pi /3
cons = con((0, pi))
res = minimize(fun(cpg), x0, method='L-BFGS-B', bounds=[(0, pi)])
print("solver result : ")
print(res.fun)
print(res.success)
print(res.x)