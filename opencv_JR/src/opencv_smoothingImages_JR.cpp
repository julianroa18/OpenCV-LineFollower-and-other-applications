/* EXAMPLE 4 USING OPENCV
/*
/*   Base: Smoothing Images
/*   from = https://docs.opencv.org/2.4/doc/tutorials/imgproc/gausian_median_blur_bilateral_filter/gausian_median_blur_bilateral_filter.html#smoothing
/*
/*   Video explain the Gaussian Blur Filter
/*   From = https://www.youtube.com/watch?v=7LW_75E3A1Q
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
#define    NODE_NAME       	"opencv_smoothingImages_JR"   //Nombre que llevará el nodo
#define    OPENCV_WINDOW1       "Original Image"   //Nombre para la ventana de la imagen original
#define    OPENCV_WINDOW2       "New Image (Filter Applied)"   //Nombre para la ventana de la imagen procesada

// Defines - Topics
#define    TOPIC1_SUB__IMAGE_INPUT      "/usb_cam/image_raw"   //Imagen tomada desde la cámara
#define    TOPIC1_PUB__IMAGE_OUTPUT     "/image_converter/output_video"   //Imagen procesada

// Define - Filtros que se pueden usar
#define    HOMOGENEOUS_BLUR	0
#define    GAUSSIAN_BLUR	1
#define    MEDIAN_BLUR		2
#define    BILATERAL_BLUR	3
int Filter_Selected = 0;   //Variable para guardar el valor del slider que elige el filtro deseado

int Kernel_length_slider = 3;   //Valor inicial
static void trackbar1_func(int, void*)   //Callback del trackbar1 que da la longitud del Kernel
{
  //Longitud del Kernel (Qué tan difuminada estará la imagen)
  //Kernel_length_slider desde 0 a 30
}

static void trackbar2_func(int, void*)   //Callback del trackbar2 que define el filtro que se usará
{
    // Valor mínimo 0. Valor máximo 3
    // Filtro deseado
}

//Nueva clase
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
        // Declaración de Topics
       	topic1_sub__image_input = it_.subscribe(TOPIC1_SUB__IMAGE_INPUT, 1, &ImageConverter::imageCb, this);
   	    topic1_pub__image_output = it_.advertise(TOPIC1_PUB__IMAGE_OUTPUT, 1);


    	  cv::namedWindow(OPENCV_WINDOW1);   //Crea la ventana donde se muestra la imagen original
	      cv::namedWindow(OPENCV_WINDOW2);   //Crea la ventana donde se muestra la imagen procesada

	      //Crea los dos trackbar y establece el valor máximo para cada uno
	      int trackbar1_maxValue = 50; // In units.
	      cv::createTrackbar("Kernel length [1-50]", OPENCV_WINDOW2, &Kernel_length_slider, trackbar1_maxValue, trackbar1_func);
	      int trackbar2_maxValue = 3; // In units.
	      cv::createTrackbar("Filter [1-3]", OPENCV_WINDOW2, &Filter_Selected, trackbar2_maxValue, trackbar2_func);
        //createTrackbar(Texto, Ubicación, Variable para guardar el valor, Valor máximo, Callback(Cuando se mueve el trackbar))
      }

      //Método Desctructor
  	  ~ImageConverter()
  	  {
        // Cierra las ventanas donde se mostraban las imagenes
    	  cv::destroyWindow(OPENCV_WINDOW1);
	      cv::destroyWindow(OPENCV_WINDOW2);
  	  }

      // Método que se ejecuta cuando se recibe un dato por el subscriber
      void imageCb(const sensor_msgs::ImageConstPtr& msg) // msg es la imagen que llega desde la cámara
      {
        //Convertir la imagen de ROS a una imagen de opencv con ayuda del brigge (Puente)
    	  cv_bridge::CvImagePtr cv_OriginalImage_ptr;
    	  try   //intentar transformar la imagen
    	  {
      		cv_OriginalImage_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);   // possible constants: "mono8", "bgr8", "bgra8", "rgb8", "rgba8", "mono16"... Option when use "cv_bridge::CvImagePtr" data type
    	  }
	      catch (cv_bridge::Exception& e)
    	  {
          // En caso de error
      		ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
        }

        /******************************/
	      /* Procesamiento de la imagen */
	      /******************************/

        // Conversion de opencv a Mat
        cv::Mat cv_OriginalImage_Mat = cv_OriginalImage_ptr->image;

        // Nueva imagen de cv::Mat para procesar (Clonada de la original)
	      cv::Mat cv_ImageProcessed_Mat = cv_OriginalImage_Mat.clone();

        //Aplicación del filtro
        int l = kernel_format(Kernel_length_slider);   //Guarda el valor del trackbar1

        switch(Filter_Selected)
	      {
          // Homogeneous blur
          case 0:
          //cv::blur(ImagenFuente, DestinoImagen, TamañoKernel, PuntoAnclaje)
		      cv::blur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), cv::Point(-1,-1));
	    	  ROS_INFO("FILTER APPLIED: ** 0. Homogeneous blur **");
          break;
          // Gaussian blur
          case GAUSSIAN_BLUR:
          //cv::GaussianBlur(ImagenFuente, DestinoImagen, TamañoKernel, DSx, DSy)
		      cv::GaussianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, cv::Size(l, l), 0, 0);
		      ROS_INFO("FILTER APPLIED: ** 1. Gaussian blur **");
          break;
          // Median blur
          case MEDIAN_BLUR:
          //cv::medianBlur(ImagenFuente, DestinoImagen, TamañoKernel)
          cv::medianBlur(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l);
		      ROS_INFO("FILTER APPLIED: ** 2. Median blur **");
          break;
          // Bilateral Filter
          case BILATERAL_BLUR:
          //cv::bilateralFilter(ImagenFuente, DestinoImagen, DiametroPixel, DScolor, DScordinate)
          cv::bilateralFilter(cv_OriginalImage_Mat, cv_ImageProcessed_Mat, l, l*2, l/2 );
          ROS_INFO("FILTER APPLIED: ** 3. Bilateral blur **");
          break;
        }
        ROS_INFO("    KERNEL_LENGTH: %d x %d", l, l);
	      ROS_INFO(" ");
        //Final de la aplicación del filtro

        /***************************/
	      /* Final del procesamiento */
	      /***************************/

        // Mostrar la imagen original en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW1, cv_OriginalImage_ptr->image);
    	  cv::waitKey(3);

        // Mostrar la imagen procesada en su respectiva ventana
   	    cv::imshow(OPENCV_WINDOW2, cv_ImageProcessed_Mat);
	      cv::waitKey(3);



    	    // Convert OpenCV image (Mat) to OpenCV image (Bridge) to ROS image (Topic)
	    //cv_bridge::CvImage cv_NewImage; // it's needed use the Class CvImage not CvImagePtr
	    //cv_NewImage.header = cv_OriginalImage_ptr->header; // Same timestamp and tf frame as Original image. The seq is assigned by ROS automatically
	    //cv_NewImage.encoding = cv_OriginalImage_ptr->encoding; // Same format as Original image
	    //cv_NewImage.image = cvMat_NewImage_ptr; // data
    	    // Output modified video stream
	    //topic1_pub__image_output.publish(cv_NewImage.toImageMsg());
      }

      /* Solo numeros impares */
      int kernel_format(int value)
      {
        if(value%2 == 0)
        return value + 1;
        else
        return value;
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
