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
def get_lane_points(img, thresh = 190, stride = 5, roi =(0, 0, 0, 0)):
    #Region of interest on the image
    road_region = img[roi[0]:roi[1], roi[2]: roi[3]]
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
        #points_in_line = []
        for col in range(1,size_road_image[1]):
            curr_brightness = int(bin_image[ren, col])

            if not on_line and curr_brightness > last_brightness:
                start_pos = int(col)
                on_line = True
            elif on_line and curr_brightness < last_brightness and (int(col)-start_pos) >= width_thresh:
                final_pos = int(col)
                on_line = False
                points.append(((final_pos+start_pos)/2, ren))


            elif on_line and curr_brightness < last_brightness and (int(col)-start_pos) < width_thresh:
                on_line = False
            last_brightness = curr_brightness
        #points = points + [points_in_line]
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


def nearest_group(dashed_group, groups ):

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
    if groups and f_group:
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


def classify_groups(first_frame, groups, last_left_point, last_right_point):

    max_radio = 25
    left_group = []
    right_group = []
    dashed_group = []
    current_left_point = []
    current_right_point = []
    d1_group = []

    if first_frame:
        if len(groups) >= 2:

            x_pos_list = [i[0][0] for i in groups]

            sorted_x = np.argsort(x_pos_list)

            if len(groups[sorted_x[0]]) > 10:
                left_group = groups[sorted_x[0]]
		
                current_left_point = left_group[0]

            if len(groups[sorted_x[-1]]) > 10:
                right_group = groups[sorted_x[-1]]
                current_right_point = right_group[0]

            if left_group and right_group:
                groups.remove(left_group)
                groups.remove(right_group)
                if groups:
                    x_pos_list = [i[0][1] for i in groups]
                    sorted_y = np.argsort(x_pos_list)
                    d1_group = groups[sorted_y[-1]]
                    groups.remove(d1_group)

            elif left_group:
                groups.remove(left_group)
                d1_group = get_first_dashed_group(left_group, groups)
                if d1_group:
                    groups.remove(d1_group)

            elif right_group:
                groups.remove(right_group)
                d1_group = get_first_dashed_group(right_group, groups)
                if d1_group:
                    groups.remove(d1_group)

            else:
                return left_group, dashed_group, right_group, current_left_point, current_right_point

            if d1_group:

                d2_group = nearest_group(d1_group, groups)

                dashed_group = list(set(d1_group) | set(d2_group))

                return left_group, dashed_group, right_group, current_left_point, current_right_point

        else:
            return left_group, dashed_group, right_group, current_left_point, current_right_point
    else:
   
	f_points = []

	if groups:
	    f_points = [group[0] for group in groups]

        if last_left_point and f_points:
	    dist_left_list = [distance.euclidean(last_left_point, i) for i in f_points]
	    sorted_left_dist = np.argsort(dist_left_list)
	    
	    if dist_left_list[sorted_left_dist[0]] <= max_radio:
	        left_group = groups[sorted_left_dist[0]]
                groups.remove(left_group)
                f_points.pop(sorted_left_dist[0])
                current_left_point = left_group[0]

        if last_right_point and f_points:
	    dist_right_list = [distance.euclidean(last_right_point, i) for i in f_points]
	    sorted_right_dist = np.argsort(dist_right_list)
	
	    if dist_right_list[sorted_right_dist[0]] <= max_radio:
	        right_group = groups[sorted_right_dist[0]]
                groups.remove(right_group)
                f_points.pop(sorted_right_dist[0])
                current_right_point = right_group[0]
     


        if left_group and right_group:

            y_pos_list = [i[0][1] for i in groups]
            if y_pos_list:
                sorted_y = np.argsort(y_pos_list)
                d1_group = groups[sorted_y[-1]]

            if d1_group:
                groups.remove(d1_group)

        elif left_group:
            #groups.remove(left_group)
            d1_group = get_first_dashed_group(left_group, groups)
            if d1_group:
                groups.remove(d1_group)
                right_group, current_right_point = search_right_group(d1_group, groups)

        elif right_group:
            #groups.remove(left_group)
            d1_group = get_first_dashed_group(left_group, groups)
            if d1_group:
                groups.remove(d1_group)
                left_group, current_left_point = search_left_group(d1_group, groups)

        else:
            return left_group, dashed_group, right_group, current_left_point, current_right_point

        if d1_group:
            d2_group = nearest_group(d1_group, groups)
            dashed_group = list(set(d1_group) | set(d2_group))

    return left_group, dashed_group, right_group, current_left_point, current_right_point


def polynomial_fitting(group):
    x = []
    y = []
    for point in group:
        x.append(point[0])
        y.append(point[1])

    p_group = np.polyfit(x, y, 1)

    return p_group


def get_heading_angle_lateral_deviation(dashed_eq, roi):
    frame_size = [roi[1] - roi[0], roi[-1]]
    vanishing_line = 0 - roi[0]
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

