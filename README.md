# Hot-Cross Buns Computer Vision Project (Inovo Robotics Internship)
During summer 2021 I spent two weeks working with Inovo Robotics on a POC computer vision project to automate the decoration of hot-cross buns. 
My solution involved four stages:

1. Create a calibrate algorithm to map between pixel coordinates in an image and metre-based coordinates that the robot head can naviagte to
2. Develop an accurate and reliable computer vision algorithm to locate the centre coordinates of a number of buns placed in a random configuration on a tray. 
3. Write a path optimisation algorithm which determines the optimal route for the icing extruder head to decorate the crosses onto the buns, minimising the wasted icing. 
4. Use the _ROS_ library to interface with Inovo’s modular robotic arm to move the extruder head across the tray of buns with the desired route.


Check out my [blog post](https://orlandoalexander.wordpress.com/2021/08/29/computer-vision-project-with-inovo-robotics/) to read more about this project and to see the robot in action!

</br>

**Calibration**
- Uses three circular stickers:
  - First sticker is in the top left hand corner of the camera’s field of view (the origin)
  - Second and third stickers are somewhere else in the camera’s field of view
- Hough circles (_OpenCV_) used to detect these circles
- User must manually move the TCP (robot head) over the centre of each circle to enable the calibration to take place
- Magnitude of the distance between these two circles is used to find the scale factor to convert between pixels and metres
- Angle created by each point with the origin and the x-axis is used to find the angular offset between the grid of the Inovo robot and the camera grid
- These two pieces of data allow each pixel coordinate to correctly translate onto a coordinate which the TCP can move to

</br>

**Computer Vision**
- Prepare image of buns on tray:
  - Convert image to grayscale for _OpenCV_ processes 
  - Use a mask to remove unwanted elements of image and isolate the buns (using HSV colour range)
  - Blur image to remove noise (wrinkles on buns and sharp edges)
  - Erode image to increase separation between buns – removes chance of errors when detecting edges of buns
  - Canny edge detection 
- Hough lines to find straight lines from canny edges
- Average these lines into vertical and horizontal lines
- Cluster lines with close proximity into a single line
- Equate the height of all vertical lines from the same bun
- Find the centre point between all these groups of vertical lines

 </br>
 
**Path Optimisation**

- Machine Learning library _mlrose_ used to find optimal path between the individual start and endpoint coordinates of the vertical and horizontal lines of the crosses on the buns
- Adapting these results, the optimal path which joins up each start and endpoint of a line to create the vertical and horizontal lines of the crosses on the buns can be found
- Additional coordinates which the TCP must pass through are integrated to ensure the icing extruder does not pass over other buns as it travels between each bun

<img src="https://user-images.githubusercontent.com/67097862/162095779-f7745a7e-28e5-4bd6-ae0e-6684df174a03.png" align = "center" width="600">

