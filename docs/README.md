# Collision Prediction System Prototyping

>Collision Prediction with LIDAR and Camera

EE 464H/R Senior Design, Spring 2018 - Team 11

Team Members: Aditya Kharosekar, Chang Park, Evan Reid, Roberto Rioja, Steven Han, Trevor Eggenberger, and [Vivian Tan](https://vivianistan.github.io)

Faculty Mentor: Dr. Joydeep Ghosh

Graduate Student Mentors: Michael Motro and Taewan Kim

Special Thanks: TxDOT (Texas Department of Transportation)

[Project Site](https://ut18-apollo.github.io/Object-Detection-and-Calibrations/#/)

# Description:

Our project is a collision prediction system that can warn drivers of impending collisions. Our design can track a single object outside of the car and detect whether it will collide with the host vehicle 2 seconds before a predicted collision. It can also track multiple objects and their distances in real-time in restricted testing conditions. 


# System Overview

Here is the block diagram of the system: 

  <div class="grid-x grid-margin-x medium-up-2 grid-margin-x-bottom" id="boxShadow">
    <div class="cell">
      <div class="hoverTarget">
        <img class="imageTarget" src="images/systemblockdiagram.jpg" alt="abstract0">
      </div>
    </div>
  </div>  

  ## Object Detection:

  To detect objects present in the frame, we are using an object detection algorithm called You Only Look Once ([YOLO](https://pjreddie.com/darknet/yolo/)) which is capable of detecting bounding boxes of objects in real-time. It applies a neural network to the entire image or frame, splitting it into different regions for which it then calculates bounding boxes and probabilities.

  ## Lidar-Camera Transformation:

  Our LIDAR-Camera Transformation (LCT) system is used to compare 3D LIDAR points to positions inside of our camera footage. We use a rotation and translation matrix to convert the 3-dimensional point cloud into the 2-dimensional domain of the camera.


   <div class="grid-x grid-margin-x medium-up-2 grid-margin-x-bottom" id="boxShadow">
    <div class="cell">
      <div class="hoverTarget">
        <img class="imageTarget" src="images/lctoverlay.png" alt="abstract0">
      </div>
    </div>
  </div>  

   *Image of point cloud overlayed on truck using LCT*

  We were able to detect a car's distance within 4% error and a person's distance within 2% error. 

  ## Object Prediction

  We used Kalman filtering to predict the trajectory of detected objects.


  # Testing and Results 

  # Future work

  # Miscellaneous: 
  ## Getting Started:
  Most of the code we used is available on Github, however our modified YOLO code is not up yet.

### To run object detection + tracking: 

1. Make sure your camera and LIDAR are in the correct positions (see calibration for more detail)

2. Run the YOLO code from `/darknet` using 
	
		./darknet detector demo cfg/coco.data cfg/yolo.cfg yolo.weights

3. In run `python main.py`



### To check the LIDAR-Camera Matrix transformation:
1. Place your camera and LIDAR in the correct position

2. Run `python lidar_projection.py` to see the lidar projection on a single image with your calibration matrix


### To capture a point cloud in .csv format:

1. Make sure your LIDAR is conneted
2. Run `python velodyne_capture_v3.py` and your point cloud will be saved as `pcl.csv` by default 

  ## Progression:
  ### Apollo:

  ### ROS:
  ### YOLO + Python:
  ## Video Demos:


<!-- 
- [System Overview](overview.md)
- [Getting Started](started.md)
- [Progress Log](log.md)
- [Demos](effects.md) -->

