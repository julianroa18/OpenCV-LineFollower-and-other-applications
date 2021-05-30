/* Template to use Open Cv inside ROS.
/*
/*   Base: ROS tutorial
/*   from = http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
/*
/*   it was modificated by Julián Roa to be use like own template
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>   //Librería puente entre ros y opencv
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_template_node_JR"   //Nombre del nodo
#define    OPENCV_WINDOW       "Image window"   //Nombre para la ventana que se abrirá con el nodo

// Defines - Topics
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw" 		//Imagen tomada desde la cámara
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video"   //Imagen procesada

// Nueva clase (OpenCV)
class ImageConverter
{
    private:

    	ros::NodeHandle nh_;   //Nodo manejador para llevar datos afuera del código

    	image_transport::ImageTransport it_;   //Objeto de la clase image_transport (Usada para procesamiento digital de la imagen)
    	image_transport::Subscriber topic1_sub__image_input;   //Topic para la imagen tomada de la cámara (subscriber)
    	image_transport::Publisher topic1_pub__image_output;   //Topic para la imagen que se publica a ROS (publisher)

    public:

      //Método Constructor
      ImageConverter() : it_(nh_)
  	  {
    	  //Declaración de los topics
        topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

    	  cv::namedWindow(OPENCV_WINDOW);   //Crear la ventana donde se mostrará la imagen
  	  }

      //Método destructor
  	  ~ImageConverter()
  	   {
    	   cv::destroyWindow(OPENCV_WINDOW);   //Cierra la ventana donde se mostraba la imagen
   	   }

      //Método que se ejecuta cuando se recibe un dato por el subscriber
      void imageCb(const sensor_msgs::ImageConstPtr& msg)   //msg es la imagen que llega desde la cámara
  	  {
        //Convertir la imagen de ROS a una imagen de opencv con ayuda del bridge (Puente)
    	  cv_bridge::CvImagePtr cv_OriginalImage_ptr;   //Imagen que llega desde la cámara, de opencv como puntero
    	  try   //Intentar transformar la imagen
    	   {
      	   cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // possible constants: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16"... Option when use "cv_bridge::CvImagePtr" data type
    	   }
	      catch (cv_bridge::Exception& e)   //Si hay algún error, no se bloquee, ejecute lo siguiente
    	   {
      		 ROS_ERROR("cv_bridge exception: %s", e.what());   //Imprime error si es detectado alguno
      		 return;
    	   }

   	    /******************************/
	      /* Procesamiento de la imagen */
	      /******************************/

	      // TODO TODO TODO

  	    /***************************/
        /* Final del procesamiento */
	      /***************************/

    	  // Dibuja un circulo sobre la imagen
    	  if (cv_OriginalImage_ptr->image.rows > 60 && cv_OriginalImage_ptr->image.cols > 60)
      	    cv::circle(cv_OriginalImage_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    	  // Mostrar la imagen en la ventana
   	    cv::imshow(OPENCV_WINDOW, cv_OriginalImage_ptr->image);
    	  cv::waitKey(3);   //Pausa de 3ms

    	  //Transformar la imagen de opencv a ROS
    	  topic1_pub__image_output.publish(cv_OriginalImage_ptr->toImageMsg());
  	  }
};

//---Main---
int main(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);   //Inicializar el nodo de ROS

    ImageConverter ic;   //Inicializar un objeto de la clase ImageConverter

    ros::spin();   //Obtiene info del topic constantemente
    return 0;
}
