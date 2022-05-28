import numpy as np
def updateCPGStance(cpg, t):
# Provides a logical vector of which legs are on the ground
    oldStance = cpg['CPGStance']
    cpg['CPGStance'] = cpg['y'][t-1,:] <= cpg['dynOffset'][1,:]
    #cpg['CPGStance'] = [1,1,1,1,1,1]

    
    cpg['CPGStanceDelta'] = np.logical_xor(oldStance,cpg['CPGStance'])

    # print(cpg['CPGStanceDelta'])
    # if cpg['CPGStanceDelta'][3] == True:
    #     print("...............................................................")
    # print(cpg['CPGStance'])
    return cpg