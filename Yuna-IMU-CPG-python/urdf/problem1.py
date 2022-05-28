import numpy as np



def interpolate(F,X,n,xEval):
	answer = F[0]
	memodict =	{}

	for i in range(1,n):
		f = divDiff(F,0,i,X,memodict)
		k = i
		subAnswer = f
		while k > 0:
			subAnswer *= (xEval-X[k-1])
			k -= 1
		answer += subAnswer
	return answer


def divDiff(F,start,stop,X,memodict):
	if (start,stop) in memodict:
		return memodict[(start,stop)]
	elif start + 1 == stop:
		a = (F[start]-F[stop])/(X[start]-X[stop])
	else:
		a = (divDiff(F,start,stop-1,X,memodict) - divDiff(F,start+1,stop,X,memodict))/(X[start]-X[stop])
	memodict[(start,stop)] = a
	return a


Xpoints = [0,1,-1,6]
Fpoints = [1,0, 4,25]
print(interpolate(Fpoints,Xpoints,3,6))


Xpoints = [0.0,1.0/8.0,.25,.5,.75,1.0]
Fpoints = [1,0.8824969,0.77880078,0.60653066,0.47236655,0.36787944]
print(interpolate(Fpoints,Xpoints,6,1/3))





def generatePoints(n):
	Xpoints = []
	Fpoints = []
	for i in range(n):
		x = i*2/n-1
		f = 1/(1+16*x*x)
		Xpoints.append(x)
		Fpoints.append(f)
	return Xpoints, Fpoints

n = 2
xVal = .05
X,F = generatePoints(n)
print(interpolate(F,X,n,xVal))

n = 4
xVal = .05
X,F = generatePoints(n)
print(interpolate(F,X,n,xVal))

n = 40
xVal = .05
X,F = generatePoints(n)
print(interpolate(F,X,n,xVal))





