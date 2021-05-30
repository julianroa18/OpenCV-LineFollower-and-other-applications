/* EXAMPLE 3 USING OPENCV
/*
/*   Base: OpenCV tutorial: "Changing the contrast and brightness of an image!"
/*   from = https://docs.opencv.org/2.4/doc/tutorials/core/basic_linear_transform/basic_linear_transform.html#basic-linear-transform
/*
/*   For implement the scrollbar/trackbar was used the tutorial: "Adding a Trackbar to our applications!"
/*   From = https://docs.opencv.org/2.4/doc/tutorials/highgui/trackbar/trackbar.html
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
#define    NODE_NAME       	"opencv_change_contrast_JR"
#define    OPENCV_WINDOW1       "Original Image"
#define    OPENCV_WINDOW2       "New Image (Contrast & brightness)"

// Defines - Topics
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw"
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video"

//***STATIC FUNCTION: Trackbar Method for Alpha value. High-GUI of OpenCV***
double Alpha = 1.5;   //Valor atribuido al contraste [1.0 a 3.0]
int trackbar1_slider;   //Valor actual del trackbar1
//Callback del trackbar1
static void trackbar1_func(int, void*)
{
    //Escalar el valor del trackbar1 de [0-100] a [0-3] para que sea el valor de Alpha
    Alpha = trackbar1_slider*3.0/100.0;
}

//***STATIC FUNCTION: Trackbar Method for Beta value. High-GUI of OpenCV***
int Beta = 30;   //Valor atribuido al brillo [0%-100%]
int trackbar2_slider;   //Valor actual del trackbar2

//Callback del trackbar2
static void trackbar2_func(int, void*)
{
    //Toma el valor del trackbar2 y lo guarda en Beta
    Beta = trackbar2_slider;
}

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
        //Declaración de topics
       	topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);

        cv::namedWindow(OPENCV_WINDOW1);   //Crea la ventana donde se muestra la imagen original
        cv::namedWindow(OPENCV_WINDOW2);   //Crea la ventana donde se muestra la imagen procesada

        //Crea los dos trackbar y establece el valor máximo de los mismos
        int trackbar_maxValue = 100;   //Porcentaje
        cv::createTrackbar("Alpha [0-100%]", OPENCV_WINDOW2, &trackbar1_slider, trackbar_maxValue, trackbar1_func); // Note the following: 1) Our Trackbar has a label "Alpha", 2) The Trackbar is located in the window “OPENCV_WINDOW2”, 3) The Trackbar values will be in the range from 0 to "trackbar_maxValue" (the minimum limit is always zero), 4) The numerical value of Trackbar is stored in "trackbar_slider", and 5) Whenever the user moves the Trackbar, the callback function on_trackbar is called
        cv::createTrackbar("Beta [0-100%]", OPENCV_WINDOW2, &trackbar2_slider, trackbar_maxValue, trackbar2_func);
        //createTrackbar(Texto, Ubicación, Variable para guardar el valor, Valor máximo, Callback(Cuando se mueve el trackbar))

      }

      //Método Desctructor
      ~ImageConverter()
      {
        //Cierra las ventanas donde se mostraban las imagenes
        cv::destroyWindow(OPENCV_WINDOW1);
        cv::destroyWindow(OPENCV_WINDOW2);
      }

      // Método que se ejecuta cuando se recibe un dato por el subscriber
      void imageCb(const sensor_msgs::ImageConstPtr& msg)   //msg es la imagen que llega desde la cámara
      {
        //Convertir la imagen de ROS a una imagen de opencv con ayuda del brigge (Puente)
    	  cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	  try//Intenta la transformación
        {
          cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	  }
        catch (cv_bridge::Exception& e)
    	  {
          //Error
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    	  }

        /******************************/
	      /* Procesamiento de la imagen */
	      /******************************/

        // Conversion de opencv a Mat
	      cv::Mat cvMat_Image_ptr = cv_OriginalImage_ptr->image;   //Instanciar objeto de la clase cv::Mat
                                                                 //Y se le atribuye la imagen original (Solo imagen)

	      //Opercaión new_image(i,j) = Alpha*Orginal_image(i,j) + Beta
 	      cv::Mat cvMat_NewImage_ptr = cv::Mat::zeros(cvMat_Image_ptr.size(), cvMat_Image_ptr.type());   //Matriz con el mismo tamaño y tipo de la imagen

        //Recorremos la imagen pixel a pixel y la modificamos dependiendo de los valores de Alpha y Beta
        for( int y = 0; y < cvMat_Image_ptr.rows; y++ )   //y-->row->fila
        {
          for( int x = 0; x < cvMat_Image_ptr.cols; x++ )   //c-->cols->columna
         	{
            for( int c = 0; c < 3; c++ )   //c-->R, G ó B->0, 1 ó 2
            {
              cvMat_NewImage_ptr.at<cv::Vec3b>(y,x)[c] =
              cv::saturate_cast<uchar>( Alpha*(cvMat_Image_ptr.at<cv::Vec3b>(y,x)[c]) + Beta );
              //sature_cast para asegurar que esté dentro de los valores correctos
            }
          }
        }
        /***************************/
	      /* Final del procesamiento */
	      /***************************/

    	  // Mostrar la imagen original en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	  cv::waitKey(3);

        // Mostrar la imagen procesada en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW2, cvMat_NewImage_ptr);
        ROS_INFO("Alpha %f ------ Beta %d", Alpha, Beta);   //Imprime valores de Alpha y Beta
        cv::waitKey(3);

    	  // Transformar la imagen de opencv (Mat) a opencv (Bridge) a ROS (Para publicar)
	      cv_bridge::CvImage cv_NewImage; // it's needed use the Class CvImage not CvImagePtr
	      cv_NewImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	      cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Same format as Original image
	      cv_NewImage.image = cvMat_NewImage_ptr; // data

        // Publica en el topic correspondiente
        topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
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
