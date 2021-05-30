/* EXAMPLE 2 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Load, Modify, and Save an Image"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/introduction/load_save_image/load_save_image.html#load-save-image
/*
/*   Aditional: Presentation "ROS_Lecture10.pptx"
/*   File: Annexed
/*
/*   it was modificated by Julián Roa to be use like own template
*/

// Includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Defines - General
#define    NODE_NAME       	"opencv_grayImage_JR"   //Nombre del nodo
#define    OPENCV_WINDOW1       "Original Image"  //Nombre para la ventana donde se mostrará la imagen original
#define    OPENCV_WINDOW2       "Gray Image"   //Nombre para la ventana donde se mostrará la imagen procesada

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
    	image_transport::Publisher topic1_pub__image_output;   //Topic para la imagen que se publica en ROS (publisher)

    public:

      //Método constructor
      ImageConverter() : it_(nh_)
      {
        //Declaración de topics
        topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

        cv::namedWindow(OPENCV_WINDOW1);   //Crea la ventana donde se mostrará la imagen original
        cv::namedWindow(OPENCV_WINDOW2);   //Crea la ventana donde se mostrará la imagen procesada
      }

      //Método destructor
      ~ImageConverter()
      {
        // Cierra las ventanas
        cv::destroyWindow(OPENCV_WINDOW1);
        cv::destroyWindow(OPENCV_WINDOW2);
      }

      // Método que se ejecuta cuando se recibe un dato por el subscriber
      void imageCb(const sensor_msgs::ImageConstPtr& msg)   //msg es la imagen que llega desde la cámara
      {
        //Convertir la imagen de ROS a una imagen de opencv con ayuda del brigge (Puente)
        cv_bridge::CvImagePtr cv_OriginalImage_ptr;   //Imagen ue llega desde la cámara, de opencv como puntero
        try   //Intentar transformar la imagen
        {
          cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)   //Si hay error, no se bloquee, ejecute lo siguiente
        {
      		ROS_ERROR("cv_bridge exception: %s", e.what());   //Imprime error si lo hay
      		return;
        }

        /******************************/
        /* Procesamiento de la imagen */
        /******************************/

        // Conversion de opencv a Mat
        cv::Mat cvMat_OriginalImage_ptr = cv_OriginalImage_ptr->image;   //Instanciar objeto de la clase cv::Mat
                                                                         //Y se le atribuye la imagen original (Solo imagen)

        // Transforma la imagen original a imagen en gris
        cv::Mat cvMat_GrayImage_ptr;
        //cvtColor(Imagen original en formato cv::Mat, Donde se almacena la imagen resultante, Tipo de conversión)
        cv::cvtColor(cvMat_OriginalImage_ptr, cvMat_GrayImage_ptr, CV_BGR2GRAY);

        /***************************/
        /* Final del procesamiento */
        /***************************/

        // Mostrar la imagen original en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
        cv::waitKey(3);

        // Mostrar la imagen procesada en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW2, cvMat_GrayImage_ptr);
        cv::waitKey(3);

        /*// Transformar la imagen de opencv (Mat) a opencv (Bridge) a ROS (Para publicar)
        cv_bridge::CvImage cv_GrayImage;   // it's needed use the Class CvImage not CvImageP
        cv_GrayImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
        cv_GrayImage.encoding = sensor_msgs::image_encodings::MONO8; // MONO8 AND MONO16 are equal to GRAY format
        cv_GrayImage.image = cvMat_GrayImage_ptr; // data

        // Publica en el topic correspondiente
        topic1_pub__image_output.publish(cv_GrayImage.toImageMsg());*/
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
