from CPG.torqueToForce import torqueToForce
import numpy as np
def updateForceStance(cpg):
#Determines which legs are currently supporting the robot's weight by
#examining the z-component of the force at each end-effector in the world
#frame
	cpg['forces'] = np.dot(cpg['pose'].T, torqueToForce(cpg, cpg['torques']))  #SOMETHING WRONG HERE
	forceThreshold = -4.0
	cpg['forceStance'] = cpg['forces'][2, :] < forceThreshold
	return cpg