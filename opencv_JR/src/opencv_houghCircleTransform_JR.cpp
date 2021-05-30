/* EXAMPLE 5 USING OPENCV
/*
/*   Base: Hough Circle Transform
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html#hough-circle
/*
/*   Base to draw into OpenCV:
/*   from = https://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html
/*
/*   Example in ROS:
/*   from: https://github.com/epsilonorion/ros_tutorials/blob/master/opencv_tut/src/findCircle.cpp
/*
/*   it was modificated by Julián Roa to be use like own template
*/

// Includes
#include <ros/ros.h>
#include <stdio.h> // needed to use the class: "vector"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_houghCircleTransform_JR"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "Image Filtered"

// Defines - Topics
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw"
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video"

//Nueva clase
class ImageConverter
{
    private:
    	// NodeHandle ROS
    	ros::NodeHandle nh_;

    	// Image used
      image_transport::ImageTransport it_;   //Objeto de la clase image_transport (Usada para procesamiento digital de la imagen)
    	image_transport::Subscriber topic1_sub__image_input;   //Topic para la imagen tomada de la cámara (subscriber)
    	image_transport::Publisher topic1_pub__image_output;   //Topic para la imagen que se publica en ROS (publisher)

    public:

      //Método constructor
      ImageConverter() : it_(nh_)
      {
        // Declaración de topics
       	topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

        cv::namedWindow(OPENCV_WINDOW1);   //Crea la ventana donde se muestra la imagen original
        cv::namedWindow(OPENCV_WINDOW2);   //Crea la ventana donde se muestra la imagen procesada
      }

      //Método Desctructor
      ~ImageConverter()
      {
        // Cierra las ventanas donde se mostraban las imagenes
    	  cv::destroyWindow(OPENCV_WINDOW1);
	      cv::destroyWindow(OPENCV_WINDOW2);
      }

      // Método que se ejecuta cuando se recibe un dato por el subscriber
      void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg is the Image get from camera (raw)
      {
        //Convertir la imagen de ROS a una imagen de opencv con ayuda del brigge (Puente)
    	  cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	  try
    	  {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	  }
	      catch (cv_bridge::Exception& e)
    	  {
          // Error
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
        }

        /******************************/
	      /* Procesamiento de la imagen */
	      /******************************/

    	  // Conversion de opencv a Mat
	      cv::Mat cvMat_Image = cv_OriginalImage_ptr->image;

	      // Transforma la imagen original a escala de grises
	      cv::Mat cvMat_GrayImage;
	      cv::cvtColor(cvMat_Image, cvMat_GrayImage, CV_BGR2GRAY);

	      //Difumina la imagen con el método Gaussiano, para evitar falsas detecciones de circulos
	      cv::Mat cvMat_GrayImage_filtered;
	      cv::GaussianBlur(cvMat_GrayImage, cvMat_GrayImage_filtered, cv::Size(9, 9), 2, 2);//Explicación en opencv_smoothingimages_JR.cpp

	      // Función para encontrar el círculo
	      std::vector<cv::Vec3f> circles;   //x, y, radio del circulo
	      cv::HoughCircles(cvMat_GrayImage_filtered, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 155, 0, 0 );
        //cv::HoughCircles(ImagenFuente (Escala de grises), Vector3, Método de detección, , DistanciaMinimaEntreCentros, Threshold, Threshold, RadioMin, RadioMax)

	      /* Example: HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
        Parameters:
        src_gray: Input image (grayscale)
        circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
	     	CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
	      dp = 1: The inverse ratio of resolution TODO
	      min_dist = src_gray.rows/8: Minimum distance between detected centers TODO
	     	param_1 = Upper threshold for the internal Canny edge detector TODO
	      param_2 = Threshold for center detection. TODO
    	  min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
    	  max_radius = 0: Maximum radius to be detected. If unknown, put zero as default */

        // Deteccion de circulo
        for(size_t i = 0; i < circles.size(); i++)
        {
          // Dibuja los círculos sobre la imagen original
          cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));   //Posición xy del centro del círculo detectado
          int radius = cvRound(circles[i][2]);   //Radio del círculo detectado
          circle(cvMat_Image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );   // Centro del círculo color verde
          circle(cvMat_Image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );   //Círculo real color rojo
          //circle(ImagenDondeDibujar(Mat), CentroCirculo, Radio, Color, Espesor, TipoContorno, 0)

          /* circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
          Parameters:
		      img – Image where the circle is drawn.
		      center – Center of the circle.
    		  radius – Radius of the circle.
    		  color – Circle color.
    		  thickness – Thickness of the circle outline. If it's negative thickness means that a filled circle is to be drawn.
    		  lineType – Type of the circle boundary. See the line() description. Default 8
    		  shift – Number of fractional bits in the coordinates of the center and in the radius value. Deafult 0*/

          // Muestra en el terminal
          ROS_INFO("Circle detected #%d / %d: ", int(i)+1, (int)circles.size());
		      ROS_INFO("    x=%d, y=%d, r=%d: ", cvRound(circles[i][0]), cvRound(circles[i][1]), cvRound(circles[i][2]));
        }

        /***************************/
	      /* Final del procesamiento */
	      /***************************/

        // Mostrar la imagen original en su respectiva ventana (Con detección de circulos)
   	    cv::imshow(OPENCV_WINDOW1, cvMat_Image);
    	  cv::waitKey(3);

	      // Mostrar la imagen en escala de grises en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_filtered);
	      cv::waitKey(3);
      }
};

//***Main***
int main(int argc, char** argv)
{
  ros::init(argc, argv, NODE_NAME);   //Inicializar el nodo de ROS

  ImageConverter ic;   //Inicializar un objeto de la clase ImageConverter

  ros::spin();   //Obtiene info del topic constantemente
  return 0;
}
