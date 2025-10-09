import pyrealsense2 as rs
from lane_detection import *
#import time
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 320, 180, rs.format.bgr8, 10)

# Start streaming
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert images to numpy arrays
    	
	#start_time = time.time()
        color_image = np.asanyarray(color_frame.get_data())
	points = get_lane_points(color_image, thresh = 200)
	points = list(filter(lambda x : x, points))
	g1, g2, g3 = clustering_points(points)
	gi, gc, gd = get_lines_labels(g1, g2, g3)
	#finish_time = time.time()
	if gi:
	    for point in gi:
	        cv2.circle(color_image, (int(point[0]), int(point[1]) + 45), 2, (255, 0, 0), -1)
	if gc:
	    for point in gc:
	        cv2.circle(color_image, (int(point[0]), int(point[1]) + 45), 2, (0, 255, 0), -1)
	if gd:
	    for point in gd:
	        cv2.circle(color_image, (int(point[0]), int(point[1]) + 45), 2, (0, 0, 255), -1)
	#print finish_time - start_time
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        #depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, None, 0.1, 0), cv2.COLORMAP_JET)

        # Stack both images horizontally
        #images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.imshow('Color image', color_image)
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        if cv2.waitKey(1) & 0xFF == ord('q'):
	    break

finally:

    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
