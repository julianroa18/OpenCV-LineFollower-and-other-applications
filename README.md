# OpenCV LineFollower and other applications
OpenCV is a cross-platform library using which we can develop real-time computer vision applications. It mainly focuses on image processing, video capture and analysis including features like face detection and object detection.

In this repository you can find six projects using OpenCV with ROS, I will explain each of them below, first I have five projects using the web cam for the image processing and the last one is a line follower that uses gazebo for simulate the world and the robot.

## opencv_template_node_JR
This is a first step to understand how to use the opencv resources to do the image processing. The goal of this template is paint a circle over the image, you can see that in the next picture:

![templatenode](https://user-images.githubusercontent.com/84452263/120122043-0afe5180-c16c-11eb-840a-8902c7966d8f.jpg)

## opencv_grayImage_JR

This template shows how to convert an RGB color image to a gray scale one

![GrayImage](https://user-images.githubusercontent.com/84452263/120122281-6da41d00-c16d-11eb-8c3b-2091096f743b.jpg)

## opencv_change_contrast_JR

With this template, you can modify the contrast from an image with two trackbars, which are associated to alpha an beta

![ChangeContrast](https://user-images.githubusercontent.com/84452263/120122282-6ed54a00-c16d-11eb-879b-ec8f22939ea2.jpg)

## opencv_smoothingImages_JR

With this template you will learn how to apply diverse linear filters to smooth images using OpenCV functions

![SmoothingImages](https://user-images.githubusercontent.com/84452263/120122284-6ed54a00-c16d-11eb-976a-440e2daee6f5.jpg)


## opencv_houghCircleTransform_JR

This template uses houghcircle method to find circles on the processed image, and with this information, it draws a cirle on the original image.

![HoughCircles](https://user-images.githubusercontent.com/84452263/120122285-6f6de080-c16d-11eb-8b41-6ceb9baacb55.jpg)

*NOTE:* All this templates are documented in spanish, but if you want an english documentation you can find in the header of the codes, a link for that documentation

## LAUNCH
You can run this nodes with the launch file included in this repository, so download the package to an existent workspace following the next steps

```
cd yourWorkspace
git clone https://github.com/julianroa18/OpenCV-LineFollower-and-other-applications
catkin_make
```
Open the launch file and comment or uncomment the node that you want run, save changes and launch with the line:

```
roslaunch opencv_JR opencv_test_JR.launch
```

# Line Follower
For this project, I used a world finded in other github repository and I coded the template that is in this one for move the robot and follow the line. You can find that world in the next repository: https://github.com/sudrag/line_follower_turtlebot

## RUN

First download both repositories in a workspace, so create a new one like this:

```
cd
mkdir -p ~/yourWorkspace/src
```

Then compile it

```
cd yourWorkspace
catkin_make
```

Don't forget add the follow line in the .bashrc file

```
source ~/yourWorkspace/devel/setup.bash
```

Download the repositories:

```
git clone https://github.com/sudrag/line_follower_turtlebot
git clone https://github.com/julianroa18/OpenCV-LineFollower-and-other-applications
```

Compile the workspace again

```
cd yourWorkspace
catkin_make
```

Now launch the world with the next line:

```
roslaunch line_follower_turtlebot lf.launch
```

And then the node

```
rosrun opencv_JR follower_JR 
```
NOTE: This template is docummented in english

# Developer
-Julián Roa

# Tutor
-Hernán Hernandez
