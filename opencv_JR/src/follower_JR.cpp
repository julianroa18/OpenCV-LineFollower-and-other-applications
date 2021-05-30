#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>   //Librería puente entre ros y opencv
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/geometry_msgs.h>

// Defines - General
#define    NODE_NAME       	"lineFollower_JR"   //Node name
#define    OPENCV_WINDOW       "Robot Vision"   //Name for the window that shows the vision of the robot
#define    OPENCV_WINDOW2       "Processed Image"   //Name for the window that shows the processed image
// Defines - Topics
#define    TOPIC1_SUB__IMAGE_INPUT      "camera/rgb/image_raw" 		//Image obtained from the camera (raw)
#define    TOPIC1_PUB__VELOCITY_OUTPUT     "/cmd_vel_mux/input/teleop"   //Movement of the robot

class lineFollower
{
    private:

    	ros::NodeHandle nh_;   //NodeHandle ROS

    	image_transport::ImageTransport it_;   //Object it_ from image transport clase (used to the digital image processing)
    	image_transport::Subscriber topic1_sub__image_input;   //Topic (subscriber)
      ros::Publisher topic1_pub__velocity_output;   //Topic (Pubisher)

    public:

      //Constructor Method
      lineFollower() : it_(nh_)
  	  {
    	  //Topics declaration
        topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &lineFollower::imageCb, this);
        topic1_pub__velocity_output = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);


    	  cv::namedWindow(OPENCV_WINDOW);   //Create the window 1
        cv::namedWindow(OPENCV_WINDOW2);   //Create the window 2
  	  }

      //Desctructor Method
  	  ~lineFollower()
  	   {
         //Close the windows
    	   cv::destroyWindow(OPENCV_WINDOW);
         cv::destroyWindow(OPENCV_WINDOW2);
   	   }

      //"CallBack" of the subscriber
      void imageCb(const sensor_msgs::ImageConstPtr& msg)   //msg is the image from the robot
  	  {
        //Convert ROS image to OpenCV image
    	  cv_bridge::CvImagePtr cv_OriginalImage_ptr;   //New image (Bridge)
    	  try   //Try to transform
    	  {
      	  cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // possible constants: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16"... Option when use "cv_bridge::CvImagePtr" data type
    	  }
	      catch (cv_bridge::Exception& e)   //ERROR
    	  {
          ROS_ERROR("cv_bridge exception: %s", e.what());   //Print error
      		return;
    	  }

        /**********************/
        /*Aply Gaussian filter*/
        /**********************/

        cv::Mat cv_OriginalImage_Mat = cv_OriginalImage_ptr->image;
        cv::Mat cv_ImageProcessed_Mat = cv_OriginalImage_Mat.clone();

        cv::GaussianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(31, 31), 0.1, 0.1);

        /**************************/
        /*Convert BGR to HSV image*/
        /**************************/

        cv::Mat cv_HSVImage;
        cv::cvtColor(cv_ImageProcessed_Mat, cv_HSVImage, CV_BGR2HSV);

        /******************************/
        /*Binary Image (Black & White)*/
        /******************************/

        cv::Mat cv_HSVImageFiltred;
        inRange(cv_HSVImage, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), cv_HSVImageFiltred);

        /***************************/
        /*Find Centroid of the Line*/
        /***************************/
        cv::Size s = cv_HSVImageFiltred.size();
        int w = s.width;
        int h = s.height;
        cv_HSVImageFiltred(cv::Rect(0, 0, w, 0.95*h))=0;   //Set an area of interest (More easy for find the centroid)

        cv::Moments M = cv::moments(cv_HSVImageFiltred);
        if (M.m00 > 0) {
          cv::Point p1(M.m10/M.m00, M.m01/M.m00);
          cv::circle(cv_HSVImageFiltred, p1, 5, cv::Scalar(155, 200, 0), -1);
        }

        /******************************************/
        /*Move the Robot to the Center of the Line*/
        /******************************************/

        float c_x = M.m10/M.m00;
        int tol = 15;
        int count = cv::countNonZero(cv_HSVImageFiltred);   //Number of pixels zero
        geometry_msgs::Twist velocity;

        if (c_x < w/2-tol)   //Turn left if centroid is to the left of the image center minus tolerance
        {
          velocity.linear.x = 0.15;
          velocity.angular.z = 0.15;
        }
        else if (c_x > w/2+tol)   //Turn right if centroid is to the right of the image center plus tolerance
        {
          velocity.linear.x = 0.15;
          velocity.angular.z = -0.15;
        }
        else   //Go straight if centroid is near image center
        {
          velocity.linear.x = 0.25;
          velocity.angular.z = 0;
        }

        if (count == 0)   // Search if no line detected
        {
          velocity.linear.x = 0;
          velocity.angular.z = 0.25;
        }

        /*****************/
        /*Publis Velocity*/
        /*****************/

        topic1_pub__velocity_output.publish(velocity);   //Publis the desicion to the robot

        /*****************/
        /*Show the images*/
        /*****************/

        cv::imshow(OPENCV_WINDOW, cv_OriginalImage_ptr->image);   //Original image from robot
    	  cv::waitKey(3);
   	    cv::imshow(OPENCV_WINDOW2, cv_HSVImageFiltred);   //Processed image
    	  cv::waitKey(3);

        /*****/
        /*END*/
        /*****/
  	  }
};

//---Main---
int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);   //Init ROS node

    lineFollower ic;   //Init object from class lineFollower

    ros::spin();   //Getting data from the subscriber
    return 0;
}
