#!/usr/bin/env python3
 
"""
First, open a new terminal and run the command:
  roslaunch realsense2_camera demo_pointcloud.launch
 
Then, open a new terminal and run this script:
  python3 demoVersion.py
 
Note that depending on lighting, the values of colorRange1_hsv
and colorRange2_hsv may need to be adjusted. In particular, the value at
index 0 in colorRange2_hsv (the upper hue value for the mask) and index 1
in colorRange1_hsv (the lower saturation value for the mask) may need
to be adjusted by +- 20. A flashlight as an addition
to this system is suggested, as this would ensure constant lighting at the
time of the camera capturing the image of the buns.
 
"""
 
import math
import json
import cv2 as cv
import numpy as np
import statistics
import cv_bridge
import rospy
import time
import sensor_msgs.msg
import geometry_msgs.msg
from commander_api.motion_control_client import MotionControlClient, Waypoint, Motion
import mlrose_hiive
 
 
 
img = np.zeros([540,960,3],dtype=np.uint8)
gray = img.copy()
calibrationData = {}
 
# initial coordinates for TCP in quaternion format
initial_quat_x = 0.6647241595678892
initial_quat_y = 0.7467135563785953
initial_quat_z = -0.022904520873364663
initial_quat_w = 0.006003276668262179
 
# initial coordinates for TCP in cartesian format
initial_car_x = 0.678
initial_car_y = -0.358
initial_car_z = 0.479
 
# z coordinate for extruder head (constant value)
extrudeCoord_z = 0.07960647367160992
 
 
# color range for the buns being decorated - used to create a mask to isolate the buns in the image
colorRange1_hsv = (0, 30, 0)
colorRange2_hsv = (40, 255, 255)
 
# constant which determines size of crosses drawn on the buns - can be changed depending on application
bunDiameter = 0.09
 
 
"Return pose of TCP, from which x, y and z coordinates can be found"
def get_tcp_pose():
    return rospy.wait_for_message("/default_move_group/tcp_pose", geometry_msgs.msg.PoseStamped)
 
 
"Capture image using camera attached to the TCP"
def get_image():
    msg = rospy.wait_for_message(
        "/camera/color/image_raw", sensor_msgs.msg.Image)
    bridge = cv_bridge.CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    cv_image_flipped = cv.flip(cv_image, 0)
    cv_image_flipped = cv.flip(cv_image_flipped, 1) # rotate image so orientation and axes are aligned with TCP motion
    return cv_image_flipped
 
"Get data required to calibrate camera and carry out processing to find scale factor to map between pixels and metres and the angular offset of the camera's grid and the grid of the Inovo robot arm"
def calibrateCamera():
    print("\nFinding circles...")
    calibrationCoords_pixel, calibrationCoords_tcp, lineLengths_pixel, lineLengths_tcp, lineAngles_pixel, lineAngles_tcp, lengthFactor, angleOffset = ([] for i in range(8))
    rospy.init_node("buns")
    origin = True
    circlesImage = get_image()
    circlesImage_gray = cv.cvtColor(circlesImage,cv.COLOR_BGR2GRAY)
    circlesImage_blurred = cv.medianBlur(circlesImage_gray,5)
    circles = cv.HoughCircles(circlesImage_blurred, cv.HOUGH_GRADIENT, 1, 20,param1=50, param2=20, minRadius=0, maxRadius=0) # locate circles (circular stickers) in camera field of view
    circles = np.uint16(np.around(circles))
    circles = sorted(circles.tolist()[0], key=lambda a: a[0])
    for circle in circles:
        # draw the circle outline onto the captured image
        circlesImage_display = circlesImage.copy()
        cv.circle(circlesImage_display, (circle[0], circle[1]), circle[2],(0,0,255), 2)
        cv.imshow("Calibration circles", circlesImage_display)
        print("\nPress enter key on the image viewer window to continue...")
        cv.waitKey()
        if origin == True:
            if (input("\nIs this circle the origin of the image frame? (Y/N) ")).capitalize() == "Y":
                print("\nMove TCP so that the extruder head is directly above the centre of this circle ")
                if (input("\nPress 'R' when the TCP is in the correct position ")).capitalize() == "R":
                    tcp = get_tcp_pose() # returns the TCP pose when the TCP is directly over the centre of the circle
                    calibrationCoords_tcp.append((tcp.pose.position.x, tcp.pose.position.y))
                    origin = False
            else:
                continue
        else:
            if (input("\nUse this circle to calibrate the robot? (Y/N) ")).capitalize() == "Y":
                print("\nMove TCP so that the extruder head is directly above the centre of this circle ")
                if (input("\nPress 'R' when the TCP is in the correct position ")).capitalize() == "R":
                    calibrationCoords_pixel.append((circle[0], circle[1]))
                    tcp = get_tcp_pose()
                    calibrationCoords_tcp.append((tcp.pose.position.x, tcp.pose.position.y))
            if len(calibrationCoords_tcp) == 3:
                break
            else:
                continue
    origin = calibrationCoords_tcp.pop(0)
    for pixelCoord in calibrationCoords_pixel:
        length = math.sqrt((pixelCoord[0]**2)+(pixelCoord[1]**2))
        lineLengths_pixel.append(length) # lengths in pixels
        angle = math.atan(pixelCoord[1]/pixelCoord[0]) # angle between calibration circle, origin and x axis of camera grid
        lineAngles_pixel.append(angle)
    for tcpCoord in calibrationCoords_tcp:
        length = math.sqrt(((tcpCoord[0]-origin[0])**2)+((tcpCoord[1]-origin[1])**2))
        lineLengths_tcp.append(length) # length in metres
        angle = math.atan(abs(tcpCoord[1]-origin[1])/(tcpCoord[0]-origin[0])) # angle between calibration circle, origin and x axis of robot grid
        lineAngles_tcp.append(angle)
    for index, lineLength_pixel in enumerate(lineLengths_pixel):
        lengthFactor.append(lineLengths_tcp[index]/lineLength_pixel)
    lengthFactor = statistics.mean(lengthFactor)
    for index, lineAngle_pixel in enumerate(lineAngles_pixel):
        angleOffset.append(abs(lineAngles_tcp[index])-abs(lineAngle_pixel)) # angular difference between x axis of camera grid and robot grid
    angleOffset = statistics.mean(angleOffset)
 
    data = {"origin": origin,"length factor": lengthFactor,"angle offset": angleOffset}
    with open("calibrationData.json", "w") as write_file:
        json.dump(data, write_file) # offset data cached in a json file
    return
 
"Move robot arm to initial position so camera can capture image of buns"
def initialPos(mc):
    # move robot to "photo" pose using quaternion coordinates
    w = Waypoint(initial_car_x, initial_car_y, initial_car_z, math.pi, 0, math.pi / 2)
    w.constrain_joint_velocity(0.3)
    w.constrain_joint_acceleration(0.3)
    w._point.pose.orientation.x = initial_quat_x
    w._point.pose.orientation.y = initial_quat_y
    w._point.pose.orientation.z = initial_quat_z
    w._point.pose.orientation.w = initial_quat_w
    mc.run(Motion(w))
 
"Prepare raw image captured by the camera to optimize accuracy of computer vision algorithm to find centre coordinates of the buns"
def prepImage():
    global img
    global gray
    # capture an image with the realsense camera
    img = get_image()
 
    dim = (960, 540)
    img = cv.resize(img, dim, interpolation=cv.INTER_AREA)
 
    # converts image to grayscale to improve computer vision
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
 
    # apply brown mask to isolate the buns in the image and remove the background
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) # convert image to hsv
    mask = cv.inRange(hsv, colorRange1_hsv, colorRange2_hsv)  # range of hue (color), saturation (percentage of white component in color) and value (brightness) which buns fit into
    buns_only = cv.bitwise_and(img, img, mask=mask)  # apply brown mask to the image
 
    # apply light blurring to image to remove light visual noise
    blurred = cv.medianBlur(buns_only, 5)
 
    # erosion reduces the size of each bun so that their edges are significantly further from each other, which removes risk of inteference between edges of different buns and ensures each bun is identified as a single entity
    for i in range(0, 29):
        eroded = cv.erode(blurred.copy(), None, iterations=i + 1)
 
    # apply further blurring to smoothen the edges of the eroded buns
    blurred = cv.medianBlur(eroded, 31)
 
    buns_white = blurred.copy()
    # convert the eroded and blurred buns to a standard white color
    buns_white[np.all(buns_white != (0, 0, 0), axis=-1)] = (255, 255, 255)
 
    # canny edge detection used to detect continuous edges in image
    edges = cv.Canny(buns_white, 60, 100)  # last two parameters in cv2.Canny() method set threshold values for pixel intensity when performing edge detection (pixels with intensity below lower threshold are automatically discarded as not being an egde and pixels and pixels with intensity above upper threshold automatically set as being an edge. Pixels with intensity between upper and lower thresholds only set as edges if connected to pixels with intensity above upper threshold)
 
    return edges
 
"Create many individual straight lines from the continuous canny edges"
def HoughLines(edges):
    lineImage = np.zeros((gray.shape), dtype=np.uint8)
    lineCoordinates = []
    rho = 2  # distance resolution in pixels of the Hough grid
    theta = np.pi / 180  # angular resolution in radians of the Hough grid
    threshold = 7  # minimum number of votes (intersections in Hough grid cell)
    min_line_length = 0 # minimum number of pixels making up a line
    max_line_gap = 20  # maximum gap in pixels between connectable line segments
    # run Hough line detection on edge detected image
    lines = cv.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap) # "lines" is an array containing endpoints of detected line segments
    lines = sorted(lines, key=lambda a: a[0][0])
    for line in lines: # store eahc individual line detect by the hough line transform
        for x1,y1,x2,y2 in line:
            newCoord = [(x1,y1),(x2,y2)]
            lineCoordinates.append(newCoord)
            cv.line(lineImage, (x1, y1), (x2, y2), (255, 0, 0), 1)
    return lineImage, lineCoordinates
 
 
"Adjusts the the Hough lines so that they are either vertical or horizontal"
def straightenLines(lineCoordinates):
    lineImage_straightened = np.zeros((gray.shape), dtype=np.uint8)
    lineCoordinates_straightened = []
    for lineCoord in lineCoordinates:
        x1 = int(lineCoord[0][0])
        x2 = int(lineCoord[1][0])
        y1 = int(min([lineCoord[0][1], lineCoord[1][1]]))
        y2 = int(max([lineCoord[0][1], lineCoord[1][1]]))
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        if dx < dy and dx < 40: # if dx < 40, line is almost vertical
            x2 = x1 = round((x2 + x1) / 2)
            length = abs(y2 - y1)
            lengthDifference = length - 2
            if lengthDifference > 0: # makes vertical line a single pixel in height
                y1 += math.ceil(lengthDifference / 2)-1
                y2 -= math.ceil(lengthDifference / 2)+1
        elif dy < dx and dy < 40: # if dy < 40, line is almost horizontal
            y2 = y1 = round((y2 + y1) / 2)
        if ((not(y2-y1 !=0 and x2-x1 != 0)) and (abs(x2-x1) > 3 or (abs(y2-y1) > 0 ))): # if line has not been made vertical or horizontal (i.e. if line is closer to be diagonal), then it is discarded.
            newCoord = [(x1, y1),(x2, y2)]
            lineCoordinates_straightened.append(newCoord)
            cv.line(lineImage_straightened, (x1, y1), (x2, y2), (255, 0, 0), 1)
    return lineImage_straightened, lineCoordinates_straightened
 
"Finds clusters of lines and averages them to create one single line"
def lineClusters(lineCoordinates_straightened, x_or_y):
    lineCoordinates_clustered = []
    clusteredImage = np.zeros((gray.shape), dtype=np.uint8)
    index1 = 0
    added = []
    if x_or_y == "x": # if function has been called to average horizontal lines
        lineCoordinates_straightened = list(filter(lambda a: a[0][1] == a[1][1], lineCoordinates_straightened))
        lineCoordinates_straightened = sorted(lineCoordinates_straightened, key=lambda a: a[0][1])
    elif x_or_y == "y": # if function has been called to average vertical lines
        lineCoordinates_straightened = list(filter(lambda a: a[0][0] == a[1][0], lineCoordinates_straightened))
        lineCoordinates_straightened = sorted(lineCoordinates_straightened,key = lambda a:a[0][0])
    while index1 < len(lineCoordinates_straightened):
        index = 0
        clustered = False
        line = lineCoordinates_straightened[index1]
        x1 = line[0][0]
        y1 = line[0][1]
        x2 = line[1][0]
        y2 = line[1][1]
        if x_or_y == "y":
            length = abs(y2-y1)
            midpoint = (y2 - (0.5*length))
        if x_or_y == "x":
            length = abs(x2-x1)
            midpoint = (x2 - (0.5*length))
        clustered_lines_y_coords = []
        clustered_lines_x_coords = []
        # averages the horizontal lines:
        if y1 == y2 and line not in added:
            while index < len(lineCoordinates_straightened):
                line_x = lineCoordinates_straightened[index]
                length_iter = abs(line_x[1][0] - line_x[0][0])
                midpoint_iter = round(line_x[1][0] - (0.5 * length_iter))
                dist_x = abs(midpoint_iter - midpoint)
                dist_y = line_x[0][1]-y1
                if ((abs(dist_y) < 20) and (abs(dist_x) < 40)):
                    clustered = True
                    clustered_lines_y_coords.append(line_x[0][1])
                    clustered_lines_x_coords.extend((line_x[0][0],line_x[1][0]))
                    added.append(line_x)
                index += 1
            if clustered == True:
                y1 = y2 = round(statistics.mean(clustered_lines_y_coords))
                x1 = min(clustered_lines_x_coords)
                x2 = max(clustered_lines_x_coords)
            x1 -= 10
            x2+= 10
            newCoord = [(x1, y1), (x2, y2)]
            lineCoordinates_clustered.append(newCoord)
            cv.line(clusteredImage,(x1, y1), (x2, y2), (255, 0, 0), 1)
            index1 += 1
        # averages the vertical lines:
        elif x1 == x2 and line not in added:
            while index < len(lineCoordinates_straightened):
                line_y = lineCoordinates_straightened[index]
                length_iter = abs(line_y[1][1] - line_y[0][1])
                midpoint_iter = round(line_y[1][1]-(0.5*length_iter))
                dist_x = line_y[0][0]-x1
                dist_y = abs(midpoint_iter-midpoint)
                if ((abs(dist_x) < 30) and (abs(dist_y) < 20)):
                    clustered = True
                    clustered_lines_y_coords.extend((line_y[0][1], line_y[1][1]))
                    clustered_lines_x_coords.append(line_y[0][0])
                    added.append(line_y)
                index += 1
            if clustered == True:
                x1 = x2 = round(statistics.mean(clustered_lines_x_coords)) - 5
                y1 = min(clustered_lines_y_coords)
                y2 = max(clustered_lines_y_coords)
                newCoord = [(x1+6, y1), (x2+6, y2)]
                lineCoordinates_clustered.append(newCoord)
                cv.line(clusteredImage, (x1+6, y1), (x2+6, y2), (255, 0, 0), 1)
            newCoord = [(x1, y1), (x2, y2)]
            lineCoordinates_clustered.append(newCoord)
            cv.line(clusteredImage,(x1, y1), (x2, y2), (255, 0, 0), 1)
            index1 +=1
        else:
            index1+=1
    return clusteredImage, lineCoordinates_clustered
 
"Adjusts the height of the vertical lines so that they are in between the horizontal line closest above them and closest below them"
def adjustHeight(lineCoordinates_x, lineCoordinates_y):
    lineCoordinates_adjusted_y, lineCoordinates_cross_sorted_y, heights_y, lineCoordinates_cross_y = ([] for i in range(4))
    lineImage_adjustedHeight = np.zeros(gray.shape, dtype=np.uint8)
 
    for line_x in lineCoordinates_x:
        y1 = y2 = line_x[0][1]
        x1 = line_x[0][0]
    for line_y in lineCoordinates_y:
        y1 = min(line_y[0][1], line_y[1][1])
        y2 = max(line_y[0][1], line_y[1][1])
        x1 = x2 = line_y[0][0]
        lineCoordinates_x_y1 = list(filter(lambda a: (a[0][1] < y1) and (min([abs(a[0][0] - x1), abs(a[1][0] - x1)]) < 55), lineCoordinates_x)) # 'lineCoordinates_x_y1' stores all horizontal lines which are closest to and above the y1 coordinate of each vertical line
        if len(lineCoordinates_x_y1) != 0:
            line_x_closest_y1 = min(lineCoordinates_x_y1, key=lambda a: abs(a[0][1] - y1))
            y1 = line_x_closest_y1[0][1] # assigns the y1 coordinate of the vertical lines to the y coordinates of the closest horizontal line above it
        lineCoordinates_x_y2 = list(filter(lambda a: (a[1][1] > y2) and (min([abs(a[0][0] - x1), abs(a[1][0] - x1)]) < 55), lineCoordinates_x))  # 'lineCoordinates_x_y2' stores all horizontal lines which are closest to and below the y2 coordinate of each vertical line
        if len(lineCoordinates_x_y2) != 0:
            line_x_closest_y2 = min(lineCoordinates_x_y2, key=lambda a: abs(a[1][1] - y2))
            y2 = line_x_closest_y2[0][1] # assigns the y2 coordinate of the vertical lines to the y coordinates of the closest horizontal line below it
        newCoord_y = [(x1, y1), (x2, y2)]
        lineCoordinates_adjusted_y.append(newCoord_y)
        heights_y.append(abs(y2-y1))
    median_height_y = round(statistics.median(heights_y)) # median height of all the vertical lines
 
    for index,line_y in enumerate(lineCoordinates_adjusted_y):
        y1 = min(line_y[0][1], line_y[1][1])
        y2 = max(line_y[0][1], line_y[1][1])
        x1 = x2 = line_y[0][0]
        newCoord_y = [(x1, y1), (x2, y2)]
        length = abs(y2-y1)
        if abs(median_height_y-length) < 30: # ensures any lines which are abnormally long are not stored, as this indicates an error has occured
            lineCoordinates_cross_y.append(newCoord_y)
            cv.line(lineImage_adjustedHeight, (x1, y1), (x2, y2), (255, 0, 0), 1)
    return lineCoordinates_cross_y, median_height_y, lineImage_adjustedHeight
 
"Determines the start and end coordinates of the vertical and horizontal lines for each cross on each bun"
def createCross(lineCoordinates_cross_y, median_height_y):
    global calibrationData
    lengthFactor = calibrationData["length factor"]
    lineImage_cross = np.zeros(gray.shape, dtype=np.uint8)
    lineCoordinates_cross_x = []
    index = 0
    middle_target = 0
    state = ""
    lineCoordinates_cross_y = sorted(lineCoordinates_cross_y, key= lambda a: a[0][0])
    while not all(a == "Null" for a in lineCoordinates_cross_y):
        if index < len(lineCoordinates_cross_y):
            if lineCoordinates_cross_y[index] != "Null":
                line_cross_y = lineCoordinates_cross_y[index]
                middle = round(line_cross_y[0][1]+(0.5*median_height_y))
                x1 = line_cross_y[0][0]
                if state != "Start": # if the vertical line being iterated over is the starting point for the horizontal line of a cross
                    x1 = x1_start = line_cross_y[0][0]
                    state = "Start"
                    coord_index = len(lineCoordinates_cross_x)
                    middle_target = middle
                    lineCoordinates_cross_x.append([(x1, middle_target)])
                    lineCoordinates_cross_y[index] = "Null"
                elif abs(middle_target - middle) <= 15 and lineCoordinates_cross_y[index] != "Null": # if the vertical line being iterated over has a midpoint which is very close to the midpoint of the midpoint of the first vertical line for that bun, then it is assumed the vertical line being iterated over belongs to the same buns
                    lineCoordinates_cross_y[index] = "Null"
                    for index, end_line in enumerate(lineCoordinates_cross_y):
                        if end_line != "Null" :
                            x1_end = end_line[0][0]
                            middle_end = round(end_line[0][1]+(0.5*median_height_y))
                            if abs(middle_end - middle_target) <= 15:
                                lineCoordinates_cross_y[index] = "Null"
                                if (abs(x1_end - x1_start) > 100) or (abs(x1_end-x1) == 6 and abs(x1_end - x1_start) > 30): # if '(abs(x1_end-x1) == 6' this indicates the final vertical line for a bun has been reached, as all vertical lines are created in pairs 6 pixels apart from each other - this technique serves as a validation check
                                    lineCoordinates_cross_x[coord_index].append((x1, middle_target)) # stores the endpoint coordinate of the horizontal line of the cross
                                    state = "End"
                                    lineCoordinates_cross_y[index] = "Null"
                                    middle_target = 0
                                    index = 0
                                    break
                                else:
                                    x1 = x1_end
                            elif abs(middle_target - middle_end) <= 35 and lineCoordinates_cross_y[index] != "Null":
                                lineCoordinates_cross_y[index] = "Null"
                    else:
                        lineCoordinates_cross_x[coord_index].append((x1, middle_target))
                        state = "End"
                        middle_target = 0
                        index = 0
                elif abs(middle_target - middle) <= 30 and lineCoordinates_cross_y[index] != "Null":
                    lineCoordinates_cross_y[index] = "Null"
                    index+=1
                    if index == len(lineCoordinates_cross_y):
                        lineCoordinates_cross_x.pop(coord_index)
                        state = "Failed"
                        middle_target = 0
                        index = 0
                else:
                    index+=1
            else:
                index += 1
        else:
            index = 0
    else:
        index +=1
    if len(lineCoordinates_cross_x[-1]) < 2:
        lineCoordinates_cross_x.pop(-1)
    centreCoordinates, lineCoordinates_cross = [], []
 
    # finds the midpoint of each horizontal line of each cross - this midpoint is the centre coordinate of the bun
    for index, horizontal_cross in enumerate(lineCoordinates_cross_x):
        y1_horizontal = horizontal_cross[0][1]
        x1_horizontal = horizontal_cross[0][0]
        x2_horizontal = horizontal_cross[1][0]
        length = abs(x2_horizontal - x1_horizontal)
        if length > 10:
            midpoint = round(x2_horizontal - (0.5 * length))
            centreCoordinates.append((midpoint,y1_horizontal))
    # using each centre coordinate for each bun, the final start and endpoints of each vertical and horizontal lines for each cross can be stored in pixels, as the diameter of each cross is a constant which is inputted by the user
    for centreCoord in centreCoordinates:
        y1 = round(centreCoord[1] - (bunDiameter / 2) / lengthFactor)
        x1 = x2 = round(centreCoord[0])
        y2 = round(centreCoord[1] + (bunDiameter / 2) / lengthFactor)
        cv.line(lineImage_cross, (x1, y1), (x2, y2), (255, 0, 0), 1)
        y1 = y2 = round(centreCoord[1])
        x1 = round(centreCoord[0] - (bunDiameter / 2) / lengthFactor)
        x2 = round(centreCoord[0] + (bunDiameter / 2) / lengthFactor)
        cv.line(lineImage_cross, (x1, y1), (x2, y2), (255, 0, 0), 1)
    return lineImage_cross, centreCoordinates
 
"Path optimisation algorithm which finds the shortest path for the TCP to extrude the icing crosses over the buns to ensure the least amount of wasted icing"
def createPath(centreCoordinates):
    global calibrationData
    lengthFactor = calibrationData["length factor"]
    pathCoordinates = []
    bestState_viable = []
    for index, centreCoord in enumerate(centreCoordinates):
        startCoord_y = centreCoord[1]-(bunDiameter/2)/lengthFactor
        finishCoord_y = centreCoord[1]+(bunDiameter/2)/lengthFactor
        startCoord_x = centreCoord[0]-(bunDiameter/2)/lengthFactor
        finishCoord_x = centreCoord[0] + (bunDiameter / 2) / lengthFactor
        pathCoordinates.extend([(centreCoord[0], startCoord_y),(centreCoord[0], finishCoord_y),(startCoord_x, centreCoord[1]),(finishCoord_x, centreCoord[1])])
    problem = mlrose_hiive.TSPOpt(length=len(pathCoordinates), coords=pathCoordinates,maximize=False) # mlrose machine learning and path optimisation library used to find the optimal path between each coordinate in 'pathCoordinates'
    bestState, bestFitness, obsolete = mlrose_hiive.genetic_alg(problem, random_state=2) # 'bestState' stores the order with which each value in 'pathCoordinates' should be visited to ensure the most optimal path
    bestState = list(bestState)
    completed = False
    index = 0
    # as the extruder must not simply visit each coordinate in 'pathCoordinates', but also travel between the start and endpoints of each vertical and horizontal line, the viable optimal path is found by
    # ensuring the appropriate endpoint is the next coordinate after a startpoint is traversed by the extruder, and then the next most optimal coordinate is travelled to (as determined using the mlrose path optimisation algorithm) and the process is repeated
    while len(bestState) > 0:
        coord = bestState[index]
        if index != len(bestState)-1:
            if (bestState[index+1] == coord+1) and (min([coord, bestState[index+1]])%2 == 0):
                try:
                    bestState_viable.extend([coord, coord+1])
                    bestState.pop(index)
                    bestState.pop(index)
                    index = index%len(bestState)
                    completed = True
                except:
                    break
            elif (bestState[index + 1] == coord - 1) and (min([coord, bestState[index + 1]]) % 2 == 0):
                try:
                    bestState_viable.extend([coord, coord-1])
                    bestState.pop(index)
                    bestState.pop(index)
                    index = index%len(bestState)
                    completed = True
                except:
                    break
        if completed == False and index != 0:
            if (bestState[index-1] == coord+1) and (min([coord, bestState[index-1]])%2 == 0):
                try:
                    bestState_viable.extend([coord, coord+1])
                    bestState.pop(index)
                    bestState.pop(index-1)
                    index = (index-1) % len(bestState)
                    completed = True
                except:
                    break
            elif (bestState[index-1] == coord-1) and (min([coord, bestState[index-1]])%2 == 0):
                try:
                    bestState_viable.extend([coord, coord-1])
                    bestState.pop(index)
                    bestState.pop(index-1)
                    index = (index-1) % len(bestState)
                    completed = True
                except:
                    break
        if completed == False and coord%2 == 0:
            try:
                coordNext = coord + 1
                bestState.pop(index)
                completed = True
                if bestState.index(coordNext) == len(bestState) -1:
                    bestState.insert(-1, coord)
                    index = index % len(bestState)
                else:
                    bestState_viable.extend([coord, coordNext])
                    coordNext_index = bestState.index(coordNext)
                    bestState.pop(coordNext_index)
                    index = coordNext_index%len(bestState)
            except:
                break
        if completed == False and coord%2 == 1:
            try:
                coordNext = coord - 1
                bestState.pop(index)
                if bestState.index(coordNext) == len(bestState) -1:
                    bestState.insert(-1, coord)
                    index = index % len(bestState)
                else:
                    bestState_viable.extend([coord, coordNext])
                    coordNext_index = bestState.index(coordNext)
                    bestState.pop(coordNext_index)
                    index = coordNext_index%len(bestState)
            except:
                break
        completed = False
    return bestState_viable, pathCoordinates
 
"The calibration data is used to map the pixel coordinates into metre coordinates for the TCP"
def mapCoords(pixelCoords):
    global calibrationData
    origin = calibrationData["origin"]
    lengthFactor = calibrationData["length factor"]
    angleOffset = calibrationData["angle offset"]
    tcpCoords = []
    for pixelCoord in pixelCoords:
        angle_pixel = math.atan(pixelCoord[1] / pixelCoord[0]) # angle between the pixel, the origin and the x axis of the camera grid
        length_pixel = math.sqrt((pixelCoord[0] ** 2) + (pixelCoord[1] ** 2)) # magnitude of the distance from the origin to the pixel coordinate
        length_tcp = length_pixel*lengthFactor
        # TCP x and y coordinates found using the scale factor conversion between pixels and metres and the angular offset between the camera's grid and the robot's grid
        tcpCoord_x = origin[0] + ((math.cos(angle_pixel+angleOffset))*length_tcp)
        tcpCoord_y = origin[1] - ((math.sin(angle_pixel + angleOffset))*length_tcp)
        tcpCoords.append((tcpCoord_x, tcpCoord_y))
    return tcpCoords
 
"Path for the TCP is finalised to avoid the extruder passing over buns inappropriately as it moves around the tray and sent to the robot using ROS"
"(This aspect of the path optimisation algorithm could stil be improved, as at times the extruder head still passes over a bun incorrectly as it moves from one side of the tray to another)"
def extrudePath(mc, tcpCoords, bestState):
    for index, coordIndex in enumerate(bestState):
        tcpCoord = tcpCoords[coordIndex]
        if index == 0: # if coordinate is the initial motion of the TCP towards the buns
            mc.run(
                Motion(
                    Waypoint(tcpCoord[0], tcpCoord[1], extrudeCoord_z, math.pi, 0, math.pi / 2)
                        .constrain_joint_velocity(0.3)
                        .constrain_joint_acceleration(0.3)))
        else:
            m = Motion()
            if not ((coordIndex == coordIndex_previous-1) or (coordIndex == coordIndex_previous+1)): # if next coordinate is not on the same bun as the previous coordinate
                if ((bunDiameter/2)-0.01) <= abs(tcpCoord[1] - tcpCoord_previous[1]) <= ((bunDiameter/2)+0.01) and ((bunDiameter/2)-0.01) <= abs(tcpCoord[0] - tcpCoord_previous[0]) <= ((bunDiameter/2)+0.01):
                    m.add(Waypoint(tcpCoord_previous[0], tcpCoord[1], extrudeCoord_z, math.pi,
                                 0, math.pi / 2)
                        .constrain_joint_acceleration(0.3)
                        .constrain_joint_velocity(0.3)
                        .set_linear())
                elif coordIndex_previous%4 == 0 or coordIndex_previous%4 == 1:
                    if tcpCoord[0]<tcpCoord_previous[0]:
                            m.add(Waypoint(tcpCoord_previous[0]-bunDiameter/2, tcpCoord_previous[1], extrudeCoord_z, math.pi, 0, math.pi / 2)
                                  .constrain_joint_acceleration(0.3)
                                  .constrain_joint_velocity(0.3)
                                  .set_linear())
                            m.add(Waypoint(tcpCoord_previous[0]-bunDiameter/2, tcpCoord[1], extrudeCoord_z, math.pi, 0,
                                           math.pi / 2)
                                  .constrain_joint_acceleration(0.3)
                                  .constrain_joint_velocity(0.3)
                                  .set_linear())
                    else:
                        m.add(Waypoint(tcpCoord_previous[0] + bunDiameter / 2, tcpCoord_previous[1], extrudeCoord_z,
                                       math.pi, 0, math.pi / 2)
                              .constrain_joint_acceleration(0.3)
                              .constrain_joint_velocity(0.3)
                              .set_linear())
                        m.add(Waypoint(tcpCoord_previous[0] + bunDiameter / 2, tcpCoord[1], extrudeCoord_z, math.pi, 0,
                                       math.pi / 2)
                              .constrain_joint_acceleration(0.3)
                              .constrain_joint_velocity(0.3)
                              .set_linear())
                elif coordIndex_previous%4 == 2 or coordIndex_previous%4 == 3:
                    if abs(tcpCoord[1]) < abs(tcpCoord_previous[1]):
                        m.add(Waypoint(tcpCoord_previous[0], tcpCoord_previous[1]-bunDiameter/2, extrudeCoord_z, math.pi, 0, math.pi / 2)
                              .constrain_joint_acceleration(0.3)
                              .constrain_joint_velocity(0.3)
                              .set_linear())
                        m.add(Waypoint(tcpCoord[0], tcpCoord_previous[1]-bunDiameter/2, extrudeCoord_z, math.pi, 0, math.pi / 2)
                              .constrain_joint_acceleration(0.3)
                              .constrain_joint_velocity(0.3)
                              .set_linear())
                    else:
                        m.add(Waypoint(tcpCoord_previous[0], tcpCoord_previous[1] + bunDiameter / 2, extrudeCoord_z,
                                       math.pi, 0, math.pi / 2)
                              .constrain_joint_acceleration(0.3)
                              .constrain_joint_velocity(0.3)
                              .set_linear())
                        m.add(Waypoint(tcpCoord[0], tcpCoord_previous[1] + bunDiameter / 2, extrudeCoord_z, math.pi, 0,
                                       math.pi / 2)
                              .constrain_joint_acceleration(0.3)
                              .constrain_joint_velocity(0.3)
                              .set_linear())
            m.add(Waypoint(tcpCoord[0], tcpCoord[1], extrudeCoord_z, math.pi, 0, math.pi/2)
                  .constrain_joint_acceleration(0.3)
                  .constrain_joint_velocity(0.3)
                  .set_linear())
            mc.run(m)
        coordIndex_previous = coordIndex
        tcpCoord_previous = tcpCoord
    initialPos(mc)
 
"Coordinates the running of the program"
def main():
    global calibrationData
    # Initialise ROS and movegroup
    rospy.init_node("buns")
    mc = MotionControlClient("default_move_group")
 
    initialPos(mc)
 
    actionInput = input("Press enter key when the tray of buns is in place. Alternatively, press 'C' to calibrate the robot camera: ").capitalize()
 
    if actionInput == "C":
        calibrateCamera()
        initialPos(mc)
        if input("Press enter key when the tray of buns is in place ") == "":
            pass
    else:
        try:
            with open("calibrationData.json", "r") as read_file:
                calibrationData = json.load(read_file)
            initialPos(mc)
        except:
            print("\nNo calibration data found. Starting robot camera calibration...")
            calibrateCamera()
            initialPos(mc)
            if input("Press enter key when the tray of buns is in place ") == "":
                pass
 
    with open("calibrationData.json", "r") as read_file:
        calibrationData = json.load(read_file)
 
    edges = prepImage()
 
    lineImage, lineCoordinates = HoughLines(edges)
 
    lineImage_straightened, lineCoordinates_straightened = straightenLines(lineCoordinates)
 
    lineImage_clustered_x, lineCoordinates_clustered_x = lineClusters(lineCoordinates_straightened, "x")
 
    lineCoordinates_y = list(filter(lambda a: a[0][0] == a[1][0], lineCoordinates_straightened))
 
    lineCoordinates_cross_y, median_height_y, lineImage_adjustedHeight = adjustHeight(lineCoordinates_clustered_x, lineCoordinates_y)
 
    lineImage_clustered_y, lineCoordinates_clustered_y = lineClusters(lineCoordinates_cross_y, "y")
 
    lineImage_cross, centreCoordinates = createCross(lineCoordinates_clustered_y, median_height_y)
 
    bestState, pathCoordinates = createPath(centreCoordinates)
    tcpCoords = mapCoords(pathCoordinates)
    extrudePath(mc, tcpCoords, bestState)
 
if __name__ == "__main__":
    main()
 
cv.waitKey()
