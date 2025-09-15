# <u>Computer Vision Path Optimisation</u>  
<img src="https://user-images.githubusercontent.com/67097862/162095779-f7745a7e-28e5-4bd6-ae0e-6684df174a03.png" align="center" width="400">


**Computer vision system** to automate the decoration of bakery goods. Decorating **6 buns in just 20 seconds** (compared to the 3–5 minutes a skilled baker would normally take) the system achieves **up to 90% greater efficiency** by identifying pastry positions, optimising the decorating path, and interfacing with a robotic arm to accurately decorate each pastry.

Developed as proof-of-concept project during a **summer internship** at **Inovo Robotics**.

📊 [Download Project Presentation](https://github.com/user-attachments/files/22311177/Inovo.Project.Presentation.pdf)<br><br>


## 🛠 Tech Stack

- **Software**: Python, OpenCV (computer vision), rospy (robot control), mlrose (path optimisation)
- **Algorithms**: Canny edge detection, Hough transforms, clustering algorithms
- **Hardware**: Inovo modular robotic arm (TCP control)<br><br>


## 📝 Project Overview

### 1. Calibration
- Uses three circular stickers:
  - First sticker is in the top left hand corner of the camera’s field of view (the origin)
  - Second and third stickers are somewhere else in the camera’s field of view
- Hough circles (_OpenCV_) used to detect these circles
- User must manually move the TCP (robot head) over the centre of each circle to enable the calibration to take place
- Magnitude of the distance between these two circles is used to find the scale factor to convert between pixels and metres
- Angle created by each point with the origin and the x-axis is used to find the angular offset between the grid of the Inovo robot and the camera grid
- These two pieces of data allow each pixel coordinate to correctly translate onto a coordinate which the TCP can move to

### 2. Computer Vision
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

### 3. Path Optimisation
- Machine Learning library _mlrose_ used to find optimal path between the individual start and endpoint coordinates of the vertical and horizontal lines of the crosses on the buns
- Adapting these results, the optimal path which joins up each start and endpoint of a line to create the vertical and horizontal lines of the crosses on the buns can be found
- Additional coordinates which the TCP must pass through are integrated to ensure the icing extruder does not pass over other buns as it travels between each bun

### 4. Robotic Arm Integration
- Uses _rospy_ to interface with Inovo’s robotic arm  
- Moves extruder along the optimised path to decorate buns accurately<br><br>
