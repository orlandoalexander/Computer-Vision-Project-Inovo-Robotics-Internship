# <u>Computer Vision Path Optimisation</u>  
<img src="https://user-images.githubusercontent.com/67097862/162095779-f7745a7e-28e5-4bd6-ae0e-6684df174a03.png" align="center" width="400">


**Computer vision system** designed to automate and optimise the decoration of bakery goods, for example hot-cross buns. The system detects the position of pastries on a tray, optimises the path for an icing extruder, and interfaces with a robotic arm to accurately decorate each pastry.  

Developed as proof-of-concept project during a **summer internship** at **Inovo Robotics**.

📊 [Download Project Presentation](https://github.com/user-attachments/files/22311177/Inovo.Project.Presentation.pdf)<br><br>


## 🛠 Tech Stack

- **Hardware**: Inovo modular robotic arm (TCP control)
- **Software**: Python, OpenCV (computer vision), rospy (robot control), mlrose (path optimisation)
- **Tools**: HSV masking, Canny edge detection, Hough transforms, clustering algorithms<br><br>


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

## 📊 Project Presentation  
<p float="left">
  <img src="https://github.com/user-attachments/assets/f640e2be-6605-4531-801e-00ef45bb9b1a" width="450" />
  <img src="https://github.com/user-attachments/assets/780204d2-1153-49bd-b611-425c1ca7b837" width="450" />
</p>

<p float="left">
  <img src="https://github.com/user-attachments/assets/24d1d57d-2e24-46ee-8ee9-09c935cfb25f" width="450" />
  <img src="https://github.com/user-attachments/assets/247b9184-e7ba-47ec-b45b-9ec0cb04b81d" width="450" />
</p>

<p float="left">
  <img src="https://github.com/user-attachments/assets/89e837d4-eef4-46e0-9349-7327ee4e4244" width="450" />
  <img src="https://github.com/user-attachments/assets/2e72b3df-4f36-4676-b5f4-2265d5f97fa2" width="450" />
</p>

<p float="left">
  <img src="https://github.com/user-attachments/assets/e90a9318-5a1c-4739-ba8b-4071142c0d34" width="450" />
  <img src="https://github.com/user-attachments/assets/ed2da222-215d-4845-b32e-ba1794dd2f54" width="450" />
</p>
