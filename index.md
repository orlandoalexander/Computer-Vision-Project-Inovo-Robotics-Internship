---
title: "Computer Vision Project - Inovo Robotics"
date: 2021-08-29
categories: 
  - "programming"
---

During summer 2021 I spent two weeks working with Inovo Robotics on a proof-of-concept computer vision project to automate the decoration of hot-cross buns. After writing an algorithm to calibrate the robot, I developed an accurate and reliable computer vision algorithm to locate the centre coordinates of a number of buns placed in a random configuration on a tray. Using this data, I wrote a path optimisation algorithm which determined the optimal route for the icing extruder head to decorate the crosses onto the buns, minimising the wasted icing. I then used the ROS library to interface with Inovo's modular robotic arm, which traversed this path across the tray of buns.

If you'd like to skip the pretty pictures and (overly) dramatic video, head over to my [GitHub Repo](https://github.com/orlandoalexander/Computer-Vision-Project-Inovo-Robotics-Internship) which has all the code I wrote for the project and explanations of my design choices.

- ![](images/20210827_164937-1.jpg)
    
- ![](images/mvimg_20210827_150131-e1630240420475.jpg)
    
- ![](images/mvimg_20210827_151134-1.jpg)
    

To celebrate an amazing two weeks at Inovo Robotics, I made a short **video showing the bun detection and decoration program I created**. The video shows the output of each stage of the back-end computer vision analysis to locate the buns on the tray as well as the robot arm decorating the buns with crosses (we didn't have any icing to hand, so you'll have to use your imagination a little bit!).

\[vimeo https://vimeo.com/594150643\]

Below is a high-level **summary of the processes** I used to create the final bun detection program:

- ![](images/slide1.jpg)
    
- ![](images/slide2.jpg)
    
- ![](images/slide3.jpg)
    
- ![](images/slide4.jpg)
    
- ![](images/slide5.jpg)
    
- ![](images/slide6.jpg)
    
- ![](images/slide7.jpg)
    
- ![](images/slide8.jpg)
    

Here is a **video displaying the camera calibration process** in action:

\[vimeo https://vimeo.com/594124125\]
