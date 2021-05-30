# OpenCV LineFollower and other applications
OpenCV is a cross-platform library using which we can develop real-time computer vision applications. It mainly focuses on image processing, video capture and analysis including features like face detection and object detection.

In this repository you can find six proyects using OpenCV with ROS, I will explain each of them below, first I have five proyects using the web cam for the image processing and the last one is a line follower that uses gazebo for simulate the world and the robot.

## opencv_template_node_JR
This is a first step to understand how to use the opencv resources to do the image processing. The goal of this template is paint a circle over the image, you can see that in the next picture:

![templatenode](https://user-images.githubusercontent.com/84452263/120122043-0afe5180-c16c-11eb-840a-8902c7966d8f.jpg)

## opencv_grayImage_JR

This template shows how to convert an RGB color image to a gray scale one


## opencv_change_contrast_JR

With this template, you can modify the contrast from an image with two trackbars, which are associated to alpha an beta


## opencv_smoothingImages_JR

With this template you will learn how to apply diverse linear filters to smooth images using OpenCV functions


## opencv_houghCircleTransform_JR

This template uses houghcircle method to find circles on the processed image, and with this information, it draws a cirle on the original image.
