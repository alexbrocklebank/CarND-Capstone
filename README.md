### Individual Submission
Alex Brocklebank - [population96@gmail.com](population96@gmail.com)

### Introduction
This repo is my submission for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).  The project entailed working on 3 separate modules: Perception, Planning, and Control.  

![Project Architecture][rosmap]

[rosmap]: https://github.com/alexbrocklebank/CarND-Capstone/blob/master/imgs/final-project-ros-graph-v2.png "Project ROS Architecture"
Image &copy; Udacity

### Native Environment Setup

I attempted all three options for environment setup: virtual machine, Docker image, and native installation. I had the most success with the native install and it is what I used for the majority of the work for this project.  Here are the instructions for setting up the native environment:

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Install

1. Clone the project repository
```bash
git clone https://github.com/alexbrocklebank/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```

3. Compile
```bash
cd ros
catkin_make
source devel/setup.sh

```
4. Run styx.launch for Highway simulator
```bash
roslaunch launch/styx.launch
```

5. Or run styx2.launch for Test Lot simulator
```bash
roslaunch launch/styx2.launch
```

6. Run the simulator and select Highway or Test Lot

### Control Module

The Control module contains the Drive-By-Wire(DBW) Node, which updates the simulator or the car to control the steering, throttle, and brakes.  This node needs to not only to be able to control the car well, but also allow for control to be relinquished to the driver and then resume without issue.  This is tested by checking and unchecking the box labeled 'Manual' in the simulator.

To control the 3 major outputs, I used a variety of controllers that Udacity provided with the skeleton framework for this project rather than reinvent the wheel.  

* The velocity was filtered with the included `LowPassFilter.py` python class, to smooth out the velocity value when being read at approximately 50 times per second.  This was especially important given how many equations rely on the velocity, and any fluctuations of the velocity would cascade through them.

* For the throttle I used a PID controller, which was included in the `PID.py` python class.  It took a large amount of fine-tuning the PID parameters before I started to actually see the vehicle slow down and stop in the simulator.

* Steering was controlled with the included `YawController.py` python class.  I had utilized a simple equation of `desired_angular_velocity * steering_ratio` for a while to achieve a rough but fairly "on rails" effect for the steering, but this was partly due to a fault in my waypoint code at that time.  The yaw controller offered smoother steering, at the cost of a little meandering.

* Braking was the simplest to control.  Braking is applied whenever the throttle reaches a minimum and is switched to 0.  This is to avoid braking while hitting the gas, which for obvious reasons is a behavior no-one wants in a car as it leads to excessive brake wear and possibly damage or fire.  The braking is applied to keep the car in place, and is not utilized excessively in slowing down the car to avoid uncomfortable rapid deceleration and "slamming" on the brakes.

### Planning Module

The Planning module contains the Waypoint Updater node, which takes in traffic light data and updates the leading waypoints the vehicle is following, which contain localization data as well as desired velocity data.  I was introduced to the KDTree algorithm from the project walkthroughs and absolutely loved how well it works and how simple it provides a solution for finding the nearest neighbor, so I used this extensively in this node for both finding both the nearest waypoints and the nearest traffic light.  It works much more efficiently than a minimum distance loop or other more basic solutions.

I found myself trying to implement too much functionality into this module, and once I resolved the traffic light classification and the waypoint velocity update functionality, I was able to remove a lot of redundant or unnecessary functionality that clouded up connected nodes.

### Perception Module

The Perception module was the most difficult part of the project.  Detecting the color of traffic lights in the Traffic Light Detection Node is the main function of this module.  This node is what updates the Planning module of where the red lights are so it can update the Control node to tell it when to stop and when it is safe to go.

I assumed that I would need to employ transfer learning with a neural network already trained in object detection.  This way all I would need to do is train it on traffic signal colors, since it would already know what a traffic light was.  Luckily I had some help from Github user [swirlingsand](https://github.com/swirlingsand/deeper-traffic-lights) that had built a playground to utilize the [Tensorflow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) which I would not possibly been able to get this project done without.  It would have taken me forever to program all the TensorFlow functionality I needed to load/train/test/freeze models on my local machine.

I pulled 3 pre-trained models from the [TensorFlow Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md) to work with:
* `faster_rcnn_inception_v2_coco_2018_01_28`
  * Achieved an accuracy of 0.0% in simulation
* `ssd_inception_v2_coco_11_06_2017`
  * Achieved an accuracy of 28.87%, mainly detecting only Red
* `faster_rcnn_resnet101_coco_11_06_2017`
  * After simplifying the prediction labels, achieved an accuracy of 99.17%!

All datasets were trained on the [Bosch Traffic Light Dataset](https://hci.iwr.uni-heidelberg.de/node/6132) with some labeled output from the Udacity Simulator as well.  One major issue discovered with the Object Detection API is that PNG data (like the entire Bosch dataset) is not handled well by the TFRecord encoder, so I fixed the issue by converting every image to JPG beforehand.  Then training completed without error.  Initially I had set training at 20,000 steps (faster_rcnn_inception) but found that the ideal checkpoint was actually much sooner, around the 2,000 - 3,000 iteration mark through TensorBoard.

### Conclusion

As much as I really wanted to get this done by myself, I do wish I had signed up for a team to have gotten to collaborate with others.  It would have saved me a lot of time and headaches when I got stuck.  I definitely could not have done this project without the help of StackOverflow for many of the odd errors I recieved from having incompatible TensorFlow versions, Python2/Python3 clashes and conversions, and just plain odd environment issues.  I also have to that the members of [Team Vulture](https://github.com/diyjac/SDC-System-Integration) because without their known working solution I would still be trying and failing to get this project running and classifying traffic lights on my laptop.  I learned so much more than I sought out to during the course of this project, but I am happy with how it all turned out.

Some areas of improvements I can see would be adding more robust physics calculations to improve steering and throttle response.  If I had more time I would also find a way to shrink the neural network and speed up its response time a little bit, and fine-tune the parameters and refresh rates on some of the nodes to reduce load and improve reaction time.  I also could codify yellow light behavior instead of having a binary red/green system.
