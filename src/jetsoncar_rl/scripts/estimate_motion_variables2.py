from __future__ import division
import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial import distance

inf = float("inf")
THETA = 0.6562437987498679  # 37.6 degrees
OP2_distance = 320
A0_distance = 70            #13.5 cm
D_distance = 17.5             #17.5 cm
#frame_size = [155, 640]


#This function retrieves a list of interest points of the track
def get_lane_points(img,  thresh = 190, stride = 5):
    #Region of interest on the image
    road_region = img
    #road_region = img[170:325, :]
    #road_region = img[45:179, :]
    #Converting image to gray scale and binary
    gray = cv2.cvtColor(road_region, cv2.COLOR_BGR2GRAY)
    ret,bin_image = cv2.threshold(gray, thresh ,255,cv2.THRESH_BINARY)

    #size of the region of interest
    size_road_image = np.shape(road_region)
    #Variable to keep track of the position of last large positive slope on the image brightness
    last_brightness = 0

    #Flag to know if a lane line has been detected,
    on_line = False
    start_pos = 0
    final_pos = 0

    points = []
    #A lane most have a width_thresh number of continous pixels to be considered a lane line
    width_thresh = 3

    for ren in range(size_road_image[0]-1, -1, -stride):
        last_brightness = int(bin_image[ren, 0])

        points_in_line = []
        for col in range(1, size_road_image[1]):
            curr_brightness = int(bin_image[ren, col])

            if not on_line and curr_brightness > last_brightness:
                start_pos = int(col)
                on_line = True
            elif on_line and curr_brightness < last_brightness and (int(col)-start_pos) >= width_thresh:
                final_pos = int(col)
                on_line = False
                points_in_line.append(((final_pos+start_pos)/2, ren))

            elif on_line and curr_brightness < last_brightness and (int(col)-start_pos) < width_thresh:
                on_line = False
            last_brightness = curr_brightness
        points = points + [points_in_line]
        start_pos = 0
        final_pos = 0
        on_line = False
    return points, bin_image


def get_groups(road_points, max_dist):
    groups = []
    current_group = []

    db = DBSCAN(eps = max_dist, min_samples = 3).fit(road_points)

    labels = set(db.labels_)

    if -1 in labels:
        labels.remove(-1)
    for group in labels:
        positions = [i for i, label in enumerate(db.labels_) if label == group]
        for pos in positions:
            current_group.append(road_points[pos])
        groups.append(current_group)
        current_group = []

    return groups


def nearest_group(dashed_group, groups, ):

    if groups:
        ref_point = dashed_group[-1]

        first_points = [i[0] for i in groups]

        dist_list = [distance.euclidean(point, ref_point) for point in first_points if
                     distance.euclidean(point, ref_point) < 40]
        if dist_list:

            return groups[np.argsort(dist_list)[0]]
        else:
            return dashed_group
    else:
        return dashed_group


def get_first_dashed_group(f_group, groups):
    if groups:
        f_point_list = [i[0] for i in groups]
        d_list = []
        for point in f_point_list:
            d_list.append(distance.euclidean(f_group[0], point))
        return groups[np.argsort(d_list)[0]]
    else:
        return []


def search_left_group(d1_group, groups):
    left_group = []
    left_point = []
    ref_point = d1_group[-1]
    f_points_list = [point[0] for point in groups]
    left_cand_list = [i for i, point in enumerate(f_points_list) if point[0] < ref_point[0] and point[1] >= ref_point[1]]
    if left_cand_list:
        num_points = [len(groups[i]) for i in left_cand_list]
        left_group = groups[np.argsort(num_points)[0]]
        left_point = left_group[0]

        return left_group, left_point
    else:
        return left_group, left_point


def search_right_group(d1_group, groups):
    left_group = []
    left_point = []
    ref_point = d1_group[-1]
    f_points_list = [point[0] for point in groups]
    right_cand_list = [i for i, point in enumerate(f_points_list) if point[0] > ref_point[0] and point[1] >= ref_point[1]]
    if right_cand_list:
        num_points = [len(groups[i]) for i in right_cand_list]
        left_group = groups[np.argsort(num_points)[0]]
        left_point = left_group[0]

        return left_group, left_point
    else:
        return left_group, left_point


def classify_groups(groups):

    left_group = []
    dashed_group = []
    right_group = []

    g1 = groups[0]
    g2 = groups[1]
    g3 = groups[2]

    if g1 and g2 and g3:

        f_points = [g1[0][0], g2[0][0], g3[0][0]]
        sorted_x = np.argsort(f_points)
        left_group = groups[sorted_x[0]]
        dashed_group = groups[sorted_x[1]]
        right_group = groups[sorted_x[2]]

        return left_group, dashed_group, right_group

    else:
        return left_group, dashed_group, right_group


def polynomial_fitting(group):
    """x = []
    y = []
    for point in group:
        x.append(point[0])
        y.append(point[1])

    p_group = np.polyfit(x, y, 1)
    return p_group"""
    

    if len(group) < 4:
	return []
    p1 = group[0]
    p2 = group[-3]

    if p2[0] - p1[0] == 0:
        return []
    else:
        m = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = -m * p1[0] + p1[1]

        return [m, b]

def get_heading_angle_lateral_deviation(dashed_eq, roi):
    frame_size = [roi[1] - roi[0], roi[-1]]
    vanishing_line = -24 # 0 - roi[0]
    O_point = (frame_size[1] / 2, vanishing_line)
    OP_distance = None
    eta = None
    delta_x = None
    if dashed_eq[0] != 0:
        OP_distance = ((vanishing_line - dashed_eq[1]) / dashed_eq[0]) - O_point[0]
        eta = np.arctan((OP_distance * np.tan(THETA)) / OP2_distance)
        p1 = ((frame_size[0] - dashed_eq[1]) / dashed_eq[0], frame_size[1] / 2)
        a_pixel_distance = p1[0] - (frame_size[1] / 2)
        a_real = (A0_distance / OP2_distance) * a_pixel_distance
        delta_x = a_real * np.cos(eta) - D_distance * np.sin(eta)

    return eta, delta_x

"""
Clustering points: this function classifies the points detected in get_lane_points function in three classes according
with a set of rules and parameters
"""

def clustering_points(points):
    # Groups: number of possible founded lines on the road: left line, central line and right line
    g1 = []
    g2 = []
    g3 = []
    last_g1 = None
    last_g2 = None
    last_g3 = None

    # Maximum difference of x coordinate to consider a point to be a member of a group accordingly to the x position of
    # the last point founded of every group
    deltax_max = 30

    for row_points in points:
        for point in row_points:

            if point[1] < 10:
                deltax_max = 15
            else:
                deltax_max = 35

            if not last_g1:
                last_g1 = point[0]
                g1.append(point)

            elif last_g1 and not last_g2:

                if abs(last_g1 - point[0]) >= deltax_max:
                    last_g2 = point[0]
                    g2.append(point)
                else:
                    last_g1 = point[0]
                    g1.append(point)

            elif last_g1 and last_g2 and not last_g3:

                if abs(last_g1 - point[0]) >= deltax_max:
                    if abs(last_g2 - point[0]) >= deltax_max:
                        last_g3 = point[0]
                        g3.append(point)
                    else:
                        last_g2 = point[0]
                        g2.append(point)
                else:
                    last_g1 = point[0]
                    g1.append(point)

            else:

                if abs(last_g1 - point[0]) >= deltax_max:
                    if abs(last_g2 - point[0]) >= deltax_max:
                        if abs(last_g3 - point[0]) < deltax_max:
                            last_g3 = point[0]
                            g3.append(point)

                    else:
                        last_g2 = point[0]
                        g2.append(point)
                else:
                    last_g1 = point[0]
                    g1.append(point)

        #print "last_g1: ", last_g1, "last_g2: ", last_g2, "last_g3: ", last_g3

    return g1, g2, g3

def closest_node(node, nodes, dashed_calculation):
    #nodes = np.asarray(nodes)
    #print nodes

    if dashed_calculation:
        max_delta = 7
        max_eta = 7
    else:
        max_delta = 12
        max_eta = 15

    deltas = nodes[:, 1]
    etas = nodes[:, 0]

    delta_dist = abs(deltas - node[1])
    eta_dist = abs(etas - node[0])
    sorted_dist = np.argsort(delta_dist)

    for i in sorted_dist:
        if delta_dist[i] <= max_delta and eta_dist[i] <= max_eta:
            return etas[i], deltas[i]

    return None, None


def get_nearest_state_variables(groups, last_eta, last_delta_x, dashed_calculation, roi):
    ref_point = np.array([last_eta, last_delta_x])
    new_groups = [group for group in groups if len(group) > 4]
    #print new_groups, "New groups pap"
    eq_mat = []
    #print "new groups", new_groups
    for group in new_groups:
        dashed_eq = list(polynomial_fitting(group))
        if dashed_eq:
            eta, delta_x = get_heading_angle_lateral_deviation(dashed_eq, roi)
	    if eta and delta_x:
            	dashed_estimation = [eta * 180 /np.pi, delta_x]
            	eq_mat.append(dashed_estimation)

    eq_mat = np.array(eq_mat)
    if list(eq_mat):
    	eta, delta_x = closest_node(ref_point, eq_mat, dashed_calculation)

    else: 
	return None, None
    if eta and delta_x:
        #dashed_calculation = True
        return eta, delta_x
    else:
        #dashed_calculation = True
        eq_mat = []
        for group in new_groups:
            dashed_eq = list(polynomial_fitting(group))
            if dashed_eq:
                eta, delta_x = get_heading_angle_lateral_deviation(dashed_eq, roi)
		if eta and delta_x:
                    left_estimation = [eta * 180 / np.pi, delta_x + 36]
                    right_estimation = [eta * 180 / np.pi, delta_x - 37]
                    eq_mat.append(left_estimation)
                    eq_mat.append(right_estimation)
        eq_mat = np.array(eq_mat)
	if list(eq_mat):
            eta, delta_x = closest_node(ref_point, eq_mat, dashed_calculation)
	else:
	    return None, None

        if eta and delta_x:
        #   dashed_calculation = False
            return eta, delta_x

        else:
            return None, None
