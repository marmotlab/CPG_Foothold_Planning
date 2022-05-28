from Tools.constrainSE3 import constrainSE3
import numpy as np


def computeOffsets(cpg, t, dt):
    [er, cpg] = constrainSE3(cpg, t, dt)  # [er, cpg] =

    # Use Jacobian to approximate joint offsets to achieve corrections

    J = cpg['xmk'].getLegJacobians(cpg['legs'])

    dynOffsetInc = np.zeros([3, 6]);

    for leg in range(6):
        Jl = J[0:3, :, leg]
        if (abs(np.linalg.det(Jl)) > 0.0001):
            dynOffsetInc[:, leg] = np.dot(np.linalg.pinv(Jl), er[:, leg])  # this might be wrong
        else:
            # 0/0
            dynOffsetInc[:, leg] = np.zeros([3])

    dynOffsetInc[0, :] = dynOffsetInc[0, :] * cpg['shouldersCorr']
    dynOffsetInc[1, :] = dynOffsetInc[1, :] * cpg['shouldersCorr']

    return [dynOffsetInc, cpg]
