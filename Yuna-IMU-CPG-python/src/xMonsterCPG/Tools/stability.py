import numpy as np
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import matplotlib.path as mplPath
from shapely.geometry import Polygon
    
# weights for body translation
def calc_weight(time, a, b):
    return (a * np.e) ** (-time * b)

# grounded = 1
def get_grounded_binary(ys):
    return np.where(ys > 0, 0, 1)

# grounded = 1
def get_grounded_binary_thetas(thetas):
    t = thetas % (2*np.pi)
    out = np.ones(len(t))
    for i in range(len(t)):
        if t[i] < np.pi and t[i] > 0:
            out[i] = 0
    return out

def grounded_binary_from_groups(active_groups):
    gb = []
    for group in active_groups:
        gr = [1,1,1,1,1,1]
        for i in group:
            gr[i-1] = 0
        gb.append(gr)
    for i in range(1, 7):
        if not any(i in group for group in active_groups):
            for group in gb:
                group[i-1] = 0
    return gb

# returns x,y positions of all grounded feet
def get_grounded_pos(E, grounded):
    feet_xy = E[0:2,].T
    num_rows, num_cols = feet_xy.shape
    grounded_feet = []
    for i in range(num_rows):
        if grounded[i] == 1: grounded_feet.append(feet_xy[i])
    return np.array(grounded_feet)

def binary_stability_score(E, grounded, pino_com):
    grounded_feet = get_grounded_pos(E, grounded)
    if len(grounded_feet) < 3: return 0
    gfl = convex_hull(E, grounded)
    return 1 if mplPath.Path(gfl).contains_point(np.array([pino_com[0], pino_com[1]])) else 0

# def stability_score(feet, grounded, thresh=0):
#     grounded_feet = get_grounded_pos(feet, grounded)
#     if len(grounded_feet) < 3: return 0.0
#     num_rows = grounded_feet.shape[0]
        
#     # Get convex hull of polygon points
#     hull = ConvexHull(grounded_feet)
#     grounded_feet = grounded_feet[hull.vertices]
#     gfl = grounded_feet.tolist()
#     gfl.append(grounded_feet[0])
#     grounded_feet = np.array(gfl)

#     c = np.array([0, 0])

#     # check if body is within polygon
#     return 1.0 if mplPath.Path(grounded_feet).contains_point(c) else 0.0
def stability_score(feet, grounded, com, thresh):
    feet_xy = feet[0:2,].T
    num_rows, num_cols = feet_xy.shape

    grounded_feet = []

    for i in range(num_rows):
        if grounded[i] == 1:
            grounded_feet.append(feet_xy[i])
    grounded_feet = np.array(grounded_feet)

    if len(grounded_feet) == 0:
        return 0.0

    num_rows = grounded_feet.shape[0]
        
    # Get convex hull of polygon points
    if len(grounded_feet) > 2:
        hull = ConvexHull(grounded_feet)
        grounded_feet = grounded_feet[hull.vertices]
        gfl = grounded_feet.tolist()
        gfl.append(grounded_feet[0])
        grounded_feet = np.array(gfl)

    c = np.array([com[0], com[1]])

    projs = []
    orths = []
    for i in range(1, num_rows + 1):
        a = c - grounded_feet[i-1]
        if i == num_rows: # wrap around to 0
            b = grounded_feet[0] - grounded_feet[i-1]
        else:
            b = grounded_feet[i] - grounded_feet[i-1]
        proj = (np.dot(a,b)/np.dot(b,b)) * b
        orth = a - proj
        projs.append(proj)
        orths.append(orth)

    norms = np.linalg.norm(orths, axis=1).tolist()

    min_norms = min(norms)
    index_min = norms.index(min_norms)
    kf1, kf2 = index_min, index_min + 1
    if kf2 == num_rows:
        kf2 = 0

    closest_pt = projs[index_min] + grounded_feet[kf1]

    # check if body is within polygon
    poly_path = mplPath.Path(grounded_feet)
    if not poly_path.contains_point(c):
        return 0.0

    stable = 1.0
    if min_norms < thresh:
        stable = min_norms/thresh
    if len(grounded_feet) < 3:
        stable = 0.0
    return stable


def convex_hull(E, grounded):
    grounded_feet = get_grounded_pos(E, grounded)
    if len(grounded_feet) < 3: return
    num_rows = grounded_feet.shape[0]
        
    #if len(grounded_feet) > 2:
    hull = ConvexHull(grounded_feet)
    grounded_feet = grounded_feet[hull.vertices]
    gfl = grounded_feet.tolist()
    gfl.append(grounded_feet[0])
    return np.array(gfl)

def get_polygon_intersection(ce, cgrounded, se, sgrounded):
    l1 = convex_hull(ce, cgrounded)
    l2 = convex_hull(se, sgrounded)

    intersect = Polygon(l1).intersection(Polygon(l2))

    if l1 is None and l2 is not None:
        return l2, l1, l2
    elif l2 is None:
        return l1, l1, l2
    if intersect.is_empty:
        return l2, l1, l2

    intersect = list(zip(*intersect.exterior.coords.xy))
    intersect = np.array(intersect)

    return intersect, l1, l2

# must have 3 legs grounded and one leg on each side grounded
# Want 1 leg in each row???
def binary_group_stability_score(active_groups, E, pino_com, left = [1,3,5], right = [2,4,6]):
    stab = []
    grounded_binary = grounded_binary_from_groups(active_groups)
    if any(sum(group) < 3 for group in grounded_binary):
        return False
    for ag, gb in zip(active_groups, grounded_binary):
        grounded_feet = get_grounded_pos(E, gb)
        hull = ConvexHull(np.array(grounded_feet))
        grounded_feet = grounded_feet[hull.vertices]
        gfl = grounded_feet.tolist()
        gfl.append(grounded_feet[0])
        if len(ag) <= 3 and any(l in ag for l in left) and any(r in ag for r in right) and mplPath.Path(np.array(gfl)).contains_point(np.array([pino_com[0], pino_com[1]])):
            stab.append(True)
    return True if all(stab) else False

def group_stability_score(feet, grounded, com, thresh):
    feet_xy = feet[0:2,].T
    num_rows, num_cols = feet_xy.shape

    grounded_feet = []
    for i in range(num_rows):
        if grounded[i] == 1:
            grounded_feet.append(feet_xy[i])
    grounded_feet = np.array(grounded_feet)

    if len(grounded_feet) == 0:
        return 0.0

    num_rows = grounded_feet.shape[0]
        
    # Get convex hull of polygon points
    if len(grounded_feet) > 2:
        hull = ConvexHull(grounded_feet)
        grounded_feet = grounded_feet[hull.vertices]
        gfl = grounded_feet.tolist()
        gfl.append(grounded_feet[0])
        grounded_feet = np.array(gfl)

    c = np.array([com[0], com[1]])

    projs = []
    orths = []
    for i in range(1, num_rows + 1):
        a = c - grounded_feet[i-1]
        if i == num_rows: # wrap around to 0
            b = grounded_feet[0] - grounded_feet[i-1]
        else:
            b = grounded_feet[i] - grounded_feet[i-1]
        proj = (np.dot(a,b)/np.dot(b,b)) * b
        orth = a - proj
        projs.append(proj)
        orths.append(orth)

    norms = np.linalg.norm(orths, axis=1).tolist()

    min_norms = min(norms)
    index_min = norms.index(min_norms)
    kf1, kf2 = index_min, index_min + 1
    if kf2 == num_rows:
        kf2 = 0

    closest_pt = projs[index_min] + grounded_feet[kf1]

    # check if body is within polygon
    poly_path = mplPath.Path(grounded_feet)
    if not poly_path.contains_point(c):
        return 0.0

    stable = 1.0
    if min_norms < thresh:
        stable = min_norms/thresh
    if len(grounded_feet) < 3:
        stable = 0.0
    return stable




























def dist_to_centroid(feet, grounded):
    feet_xy = feet[0:2,].T
    num_rows, num_cols = feet_xy.shape
    grounded_feet = []

    for i in range(num_rows):
        if grounded[i] == 1:
            grounded_feet.append(feet_xy[i])
    grounded_feet = np.array(grounded_feet)

    if len(grounded_feet) < 3:
        return 1

    x_ = np.sum(grounded_feet[:,0])/len(grounded_feet)
    y_ = np.sum(grounded_feet[:,1])/len(grounded_feet)
    displacement_v = np.array([x_, y_])
    dist = np.linalg.norm(displacement_v)
    return dist

# return dist from edge if unstable
def stab_table(feet, grounded):
    feet_xy = feet[0:2,].T
    num_rows, num_cols = feet_xy.shape
    grounded_feet = []

    for i in range(num_rows):
        if grounded[i] == 1:
            grounded_feet.append(feet_xy[i])
    grounded_feet = np.array(grounded_feet)

    if len(grounded_feet) == 0:
        return 0, 0

    num_rows = grounded_feet.shape[0]
        
    # Get convex hull of polygon points
    if len(grounded_feet) > 2:
        hull = ConvexHull(grounded_feet)
        grounded_feet = grounded_feet[hull.vertices]
        gfl = grounded_feet.tolist()
        gfl.append(grounded_feet[0])
        grounded_feet = np.array(gfl)

    c = np.array([0, 0])

    projs = []
    orths = []
    for i in range(1, len(grounded_feet) + 1):
        a = c - grounded_feet[i-1]
        if i == len(grounded_feet): # wrap around to 0
            b = grounded_feet[0] - grounded_feet[i-1]
        else:
            b = grounded_feet[i] - grounded_feet[i-1]
        proj = (np.dot(a,b)/np.dot(b,b)) * b
        orth = a - proj
        projs.append(proj)
        orths.append(orth)

    norms = np.linalg.norm(orths, axis=1).tolist()

    min_norms = min(norms)
    index_min = norms.index(min_norms)
    kf1, kf2 = index_min, index_min + 1
    if kf2 == num_rows:
        kf2 = 0

    closest_pt = projs[index_min] + grounded_feet[kf1]

    # check if body is within polygon
    poly_path = mplPath.Path(grounded_feet)
    if not poly_path.contains_point(c):
        return np.linalg.norm(closest_pt), 0

    stable = 1.0
    # if min_norms < thresh:
    #     stable = min_norms/thresh
    if len(grounded_feet) < 3:
        stable = 0.0
    return 0, 1

legs_moving = [
            # zero
            [1,1,1,1,1,1],
            # one
            [1,1,1,1,1,0],
            [1,1,1,1,0,1],
            [1,1,1,0,1,1], 
            [1,1,0,1,1,1], 
            [1,0,1,1,1,1], 
            [0,1,1,1,1,1], 
            # two
            [1,1,1,1,0,0], 
            [1,1,1,0,1,0],
            [1,1,1,0,0,1], 
            [1,1,0,1,1,0], 
            [1,1,0,1,0,1], 
            [1,1,0,0,1,1],
            [1,0,1,1,1,0], 
            [1,0,1,1,0,1], 
            [1,0,1,0,1,1],
            [1,0,0,1,1,1],
            [0,1,1,1,1,0], 
            [0,1,1,1,0,1], 
            [0,1,1,0,1,1],
            [0,1,0,1,1,1],
            [0,0,1,1,1,1],    
            # three
            [1,1,1,0,0,0],
            [1,1,0,1,0,0], 
            [1,1,0,0,1,0], 
            [1,1,0,0,0,1], 
            [1,0,1,1,0,0], 
            [1,0,1,0,1,0], 
            [1,0,1,0,0,1],  
            [1,0,0,1,1,0], 
            [1,0,0,1,0,1], 
            [1,0,0,0,1,1], 
            [0,1,1,1,0,0],
            [0,1,1,0,1,0], 
            [0,1,1,0,0,1],  
            [0,1,0,1,1,0], 
            [0,1,0,1,0,1], 
            [0,1,0,0,1,1], 
            [0,0,1,1,1,0], 
            [0,0,1,1,0,1],
            [0,0,1,0,1,1],
            [0,0,0,1,1,1],
            # four
            [1,1,0,0,0,0], 
            [1,0,1,0,0,0],
            [1,0,0,1,0,0],
            [1,0,0,0,1,0],
            [1,0,0,0,0,1],
            [0,1,1,0,0,0],
            [0,1,0,1,0,0],
            [0,1,0,0,1,0],
            [0,1,0,0,0,1],
            [0,0,1,1,0,0],
            [0,0,1,0,1,0],
            [0,0,1,0,0,1],
            [0,0,0,1,1,0],
            [0,0,0,1,0,1],
            [0,0,0,0,1,1],
            # five
            [1,0,0,0,0,0],
            [0,1,0,0,0,0], 
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0], 
            [0,0,0,0,0,1],   
            # six
            [0,0,0,0,0,0]]

def cross_point(s_p,CoM):

    # consider the forward transition x-axis only  TODO: both x and y axis transition
    p1, p2 = s_p[0, 0], s_p[0, 1]
    p3, p4 = s_p[1, 0], s_p[1, 1]
    p5, p6 = s_p[2, 0], s_p[2, 1]

    k1 = (p4 - p2) / (p3 - p1)
    k2 = (p6 - p4) / (p5 - p3)
    k3 = (p6 - p2) / (p5 - p1)

    b1 = p2 - k1 * p1
    b2 = p4 - k2 * p3
    b3 = p6 - k3 * p5

    c1 = - b1 / k1
    c2 = - b2 / k2
    c3 = - b3 / k3

    R = []
    if c1 > CoM[0]:
        R.append(c1 - CoM[0])
    if c2 > CoM[0]:
        R.append(c2 - CoM[0])
    if c3 > CoM[0]:
        R.append(c3 - CoM[0])

    return min(R)


if __name__ == "__main__":
    s_p = np.array([[2.0,2.0], [1.0,1.0], [3.0,-1.0], [3.7,-1.8]])
    CoM = np.array([0.1,0])
    dis = cross_point(s_p,CoM)
    print(dis)

