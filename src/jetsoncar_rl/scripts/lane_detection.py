from __future__ import division
import cv2
import numpy as np


THETA = 0.6562437987498679  # 37.6 degrees
OP2_distance = 159
A0_distance = 13.5            #13.5 cm
D_distance = 17.5             #17.5 cm
inf = float("inf")
state_variance = 15

#This function retrieves a list of interest points of the track
def get_lane_points(img, thresh = 190):
    #Region of interest on the image
    road_region = img[45:179, :]

    #Converting image to gray scale and binary
    gray = cv2.cvtColor(road_region, cv2.COLOR_BGR2GRAY)
    ret,bin_image = cv2.threshold(gray, thresh , 255, cv2.THRESH_BINARY)

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

    for ren in range(size_road_image[0]-1, -1, -5):
        last_brightness = int(bin_image[ren, 0])
        points_in_line = []
        for col in range(1,size_road_image[1]):
            curr_brightness = int(bin_image[ren, col])

            if not(on_line) and curr_brightness > last_brightness:
                start_pos = int(col)
                on_line = True
            elif on_line and curr_brightness < last_brightness and (int(col)-start_pos) >= width_thresh:
                final_pos = int(col)
                on_line = False
                points_in_line = points_in_line + [((final_pos+start_pos)/2, ren)]


            elif on_line and curr_brightness < last_brightness and (int(col)-start_pos) < width_thresh:
                on_line = False
            last_brightness = curr_brightness
        points = points + [points_in_line]
        start_pos = 0
        final_pos = 0
        on_line = False
    return points


"""
Clustering points: this function classifies the points detected in get_lane_points function in three classes according
with a set of rules and parameters
"""
def clustering_points(points):
    #Groups: number of posible founded lines on the road: left line, central line and right line
    g1 = []
    g2 = []
    g3 = []
    last_g1 = None
    last_g2 = None
    last_g3 = None

    #Maximum diference of x coordinate to consider a point to be a member of a group accordingly to the x position of
    #the last point founded of every group
    deltax_max = 70

    very_croked = False
    for curr_points in points:

        if curr_points:
            if (len(curr_points) > 1 and curr_points[0][1] > 67) or curr_points[0][0] < 20 or curr_points[0][0] > 300:
                very_croked = True

            if very_croked:
                if curr_points[0][1] < 40 and curr_points[0][1] > 30:
                    deltax_max = 92
                elif curr_points[0][1] < 30 and curr_points[0][1] > 20:
                    deltax_max = 70
                elif curr_points[0][1] < 20:
                    deltax_max = 50

            else:

                if curr_points[0][1] < 40 and curr_points[0][1] > 30:
                    deltax_max = 60
                elif curr_points[0][1] < 30 and curr_points[0][1] > 20:
                    deltax_max = 45
                elif curr_points[0][1] < 20:
                    deltax_max = 40

            if len(curr_points) == 1:
                if last_g1 == None:
                    last_g1 = curr_points[0][0]  # Saving the last x coordenade of the point saved on group 1
                    g1 = g1 + [curr_points[0]]
                elif last_g1 != None and last_g2 == None:
                    if abs(last_g1 - curr_points[0][0]) >= deltax_max:
                        last_g2 = curr_points[0][0]
                        g2 = g2 + [curr_points[0]]
                    else:
                        last_g1 = curr_points[0][0]
                        g1 = g1 + [curr_points[0]]
                elif last_g1 != None and last_g2 != None and last_g3 == None:
                    if abs(last_g1 - curr_points[0][0]) >= deltax_max:
                        if abs(last_g2 - curr_points[0][0]) >= deltax_max:
                            last_g3 = curr_points[0][0]
                            g3 = g3 + [curr_points[0]]
                        else:
                            last_g2 = curr_points[0][0]
                            g2 = g2 + [curr_points[0]]
                    else:
                        last_g1 = curr_points[0][0]
                        g1 = g1 + [curr_points[0]]
                else:
                    nearest_group = np.argmin([abs(last_g1 - curr_points[0][0]), abs(last_g2 - curr_points[0][0]), \
                                               abs(last_g3 - curr_points[0][0])])
                    if nearest_group == 0:
                        last_g1 = curr_points[0][0]
                        g1 = g1 + [curr_points[0]]
                    elif nearest_group == 1:
                        last_g2 = curr_points[0][0]
                        g2 = g2 + [curr_points[0]]
                    else:
                        last_g3 = curr_points[0][0]
                        g3 = g3 + [curr_points[0]]

            elif len(curr_points) == 2:
                if last_g1 == None:
                    last_g1 = curr_points[0][0]
                    g1 = g1 + [curr_points[0]]
                    last_g2 = curr_points[1][0]
                    g2 = g2 + [curr_points[1]]

                elif last_g1 != None and last_g2 == None:
                    for point in curr_points:
                        if abs(last_g1 - point[0]) >= deltax_max:
                            if last_g2 != None and last_g3 == None:
                                last_g3 = point[0]
                            else:
                                last_g2 = point[0]
                                g2 = g2 + [point]
                        else:
                            last_g1 = point[0]
                            g1 = g1 + [point]


                elif last_g1 != None and last_g2 != None and last_g3 == None:
                    for point in curr_points:
                        if abs(last_g1 - point[0]) >= deltax_max:
                            if abs(last_g2 - point[0]) >= deltax_max:
                                last_g3 = point[0]
                                g3 = g3 + [point]
                            else:
                                last_g2 = point[0]
                                g2 = g2 + [point]
                        else:
                            last_g1 = point[0]
                            g1 = g1 + [point]

                else:
                    for point in curr_points:
                        if abs(last_g1 - point[0]) >= deltax_max:
                            if abs(last_g2 - point[0]) >= deltax_max:
                                # AQui se puede agregar las regla completa
                                last_g3 = point[0]
                                g3 = g3 + [point]
                            else:
                                last_g2 = point[0]
                                g2 = g2 + [point]
                        else:
                            last_g1 = point[0]
                            g1 = g1 + [point]

            else:
                if last_g1 == None:
                    last_g1 = curr_points[0][0]
                    g1 = g1 + [curr_points[0]]
                    last_g2 = curr_points[1][0]
                    g2 = g2 + [curr_points[1]]
                    last_g3 = curr_points[2][0]
                    g3 = g3 + [curr_points[2]]

                elif last_g1 != None and last_g2 == None:
                    for point in curr_points:
                        if abs(last_g1 - point[0]) >= deltax_max:
                            if last_g2 != None and last_g3 == None:
                                last_g3 = point[0]
                            else:
                                last_g2 = point[0]
                                g2 = g2 + [point]
                        else:
                            last_g1 = point[0]
                            g1 = g1 + [point]
                else:
                    for point in curr_points:
                        if abs(last_g1 - point[0]) >= deltax_max:
                            if abs(last_g2 - point[0]) >= deltax_max:
                                # AQui se puede agregar las regla completa
                                last_g3 = point[0]
                                g3 = g3 + [point]
                            else:
                                last_g2 = point[0]
                                g2 = g2 + [point]
                        else:
                            last_g1 = point[0]
                            g1 = g1 + [point]
    return g1, g2, g3

def get_lines_labels(g1, g2, g3):
    if g1 and g2 and g3:
        g1_av = 0
        g2_av = 0
        g3_av= 0

        for i in g1:
            g1_av = g1_av + i[0]
        g1_av = g1_av / len(g1)

        for i in g2:
            g2_av = g2_av + i[0]
        g2_av = g2_av / len(g2)

        for i in g3:
            g3_av = g3_av + i[0]
        g3_av = g3_av / len(g3)

        sorted_groups = np.argsort([g1_av, g2_av, g3_av])
        
        if sorted_groups[0] == 0 and sorted_groups[1] == 1:
            return [g1, g2, g3]
        elif sorted_groups[0] == 0 and sorted_groups[1] == 2:
            return [g1, g3, g2]
        elif sorted_groups[0] == 1 and sorted_groups[1] == 0:
            return [g2, g1, g3]
	elif sorted_groups[0] == 1 and sorted_groups[1] == 2:
	    return [g2, g3, g1]
	elif sorted_groups[0] == 2 and sorted_groups[1] == 0:
	    return [g3, g1, g2]
	else:
	    return [g3, g2, g1]	
	
    elif g1 and g2 and not g3:
        g1_delta = 0
        g2_delta = 0

	g1_av = 0
        g2_av = 0

        for i in g1:
            g1_av = g1_av + i[0]
        g1_av = g1_av / len(g1)

        for i in g2:
            g2_av = g2_av + i[0]
        g2_av = g2_av / len(g2)
	
	#Check for the group that contains the pair of contiguos furthest points (central line)
        for i in range(len(g1)-1):
            if abs(g1[i][1] - g1[i + 1][1]) > g1_delta:
                g1_delta = abs(g1[i][1] - g1[i+1][1])

        for i in range(len(g2)-1):
            if abs(g2[i][1] - g2[i + 1][1]) > g2_delta:
                g2_delta = abs(g2[i][1] - g2[i+1][1])

        if g1_delta > g2_delta: 	#g1 is the central line
	    if g1_av < g2_av:
            	return [g3, g1, g2]
	    else:
		return [g2, g1, g3]
        else:				#g2 is the central line
            if g1_av < g2_av:
            	return [g1, g2, g3]
            else:
	    	return [g3, g2, g1]

    else:
    	return [[], [], []]
        


def get_heading_angle_lateral_deviation(central_points):
    if len(central_points) < 4:
	return None, None
    vanishing_line = 0              #Vanishing line
    bottom_line = 134
    p1 = central_points[0]
    p2 = central_points[-3]
    if p2[0] - p1[0] == 0:
	m = inf
    else:
	m = (p2[1] - p1[1]) / (p2[0] - p1[0])


    if m == 0:
	return None, None

    #OP_distance = ((vanishing_line - y1) / m + x1) - OP2_distance
    OP_distance =  (((vanishing_line - p1[1]) / m ) + p1[0]) - OP2_distance
    #a_pixel_distance = (((bottom_line - y1) / m) + x1) - OP2_distance
    a_pixel_distance = (((bottom_line-p1[1])/ m) + p1[0]) - OP2_distance
    a_real = (A0_distance / OP2_distance) * a_pixel_distance
    eta = np.arctan((OP_distance * np.tan(THETA)) / OP2_distance)
    delta_x = a_real * np.cos(eta) - D_distance * np.sin(eta)

    return eta, delta_x


def get_values_for_one_visible_line(last_value, g1):
    eta, delta_x = get_heading_angle_lateral_deviation(g1)
    if delta_x != None:
	result = np.array([delta_x + 43, delta_x, delta_x - 43])
	min_position = int(np.argmin(abs(result - last_value)))
	if result[min_position] <= state_variance:
	    
	    return eta * (180/np.pi), result[min_position]
	else:
	    return None, None
    else:
        return None, None

def get_values_for_two_visible_lines(last_value, g1, g2):
    
    eta1, delta_x1 = get_heading_angle_lateral_deviation(g1)
    eta2, delta_x2 = get_heading_angle_lateral_deviation(g2)
    if delta_x1 != None and delta_x2 != None:
	result = np.array([delta_x1 + 43, delta_x1, delta_x1 - 43, delta_x2 + 43, delta_x2, delta_x2 - 43])
	min_position = int(np.argmin(abs(result - last_value)))
	if result[min_position] <= state_variance:
	    
	    if min_position <= 2:
		return eta1 * (180/np.pi), result[min_position]
	    else:
		return eta2 * (180/np.pi), result[min_position]
	else:
	    return None, None
    elif delta_x1 != None:
	return get_values_for_one_visible_line(last_value, g1)
    elif delta_x2 != None:
	return get_values_for_one_visible_line(last_value, g2)
    else:
	return None, None
	 

def get_values_for_three_visible_lines(last_value, g1, g2, g3):
    eta1, delta_x1 = get_heading_angle_lateral_deviation(g1)
    eta2, delta_x2 = get_heading_angle_lateral_deviation(g2)
    eta3, delta_x3 = get_heading_angle_lateral_deviation(g3)
	
    if delta_x1 != None and delta_x2 != None and delta_x3 != None:
	result = np.array([delta_x1 + 43, delta_x1, delta_x1 - 43, delta_x2 + 43, delta_x2, delta_x2 - 43, delta_x3 + 43, delta_x3, delta_x3 - 43])
        min_position = int(np.argmin(abs(result - last_value)))
	if result[min_position] <= state_variance:
	    if min_position <= 2:
		return eta1 * (180/np.pi), result[min_position]
	    elif min_position > 2 and  min_position <= 5:
		return eta2 * (180/np.pi), result[min_position]
	    else:
		return eta3 * (180/np.pi), result[min_position]
	else:
	    return None, None
    elif delta_x1 != None and delta_x2 != None:
	return get_values_for_two_visible_lines(last_value, g1, g2)
    elif delta_x1 != None and delta_x3 != None:
	return get_values_for_two_visible_lines(last_value, g1, g3)
    elif delta_x2 != None and delta_x3 != None:
	return get_values_for_two_visible_lines(last_value, g2, g3)
    elif delta_x1 != None:
	return get_values_for_one_visible_line(last_value, g1)
    elif delta_x2 != None:
	return get_values_for_one_visible_line(last_value, g2)
    elif delta_x3 != None:
	return get_values_for_two_visible_line(last_value, g3)
    else:
	return None, None







