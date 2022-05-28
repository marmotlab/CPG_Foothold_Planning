import numpy as np

def limitValue(inputVal, limit):
#function out = limitValue(in, limit)
#Bounds "in" to [-limit, limit]
	return np.maximum(-limit, np.minimum(limit, inputVal))


