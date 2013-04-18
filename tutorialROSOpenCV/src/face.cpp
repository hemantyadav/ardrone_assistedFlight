//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <iostream>
#include <opencv2/objdetect/objdetect.hpp>
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "OpenCV Stream";
static const std::string face_cascade_name = "haarcascade_frontalface_alt.xml";
static const std::string eyes_cascade_name = "haarcascade_eye_tree_eyeglasses.xml";


class ImageConverter
{

	cv::CascadeClassifier face_cascade;
	cv::CascadeClassifier eyes_cascade;
	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	
	
	cv::Mat frame_gray;
	std::vector<cv::Rect> faces;
	
	public:
	 ImageConverter()
	 	: it_(nh_)
	 	{
	 	// image_pub_ = it_.advertise("ardrone/bottom/image_processed",1);
	 	// image_sub_ = it_.subscribe("ardrone/bottom/image_raw" ,1, &ImageConverter::imageCb, this);
	 	image_pub_ = it_.advertise("camera/image_processed",1);
	 	image_sub_ = it_.subscribe("camera/image_raw" ,1, &ImageConverter::imageCb, this);  
	 	 
	 	 cv::namedWindow(WINDOW);
	 	}
	 	
	 ~ImageConverter()
	 {
	 	cv::destroyWindow(WINDOW);
	 }

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try{ 
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		 }
		catch(cv_bridge::Exception& e){	ROS_ERROR("cv_bridge exception: %s", e.what()); return;	}
		
		if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return; };
		if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return; };
		
		cv::cvtColor(cv_ptr->image, frame_gray, CV_BGR2GRAY);
		cv::equalizeHist( frame_gray, frame_gray);
		
		face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );
		for(unsigned int i = 0; i < faces.size(); i++ )
		  {
		    cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		    cv::ellipse( cv_ptr->image, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

		    cv::Mat faceROI = frame_gray( faces[i] );
		    std::vector<cv::Rect> eyes;

		    //-- In each face, detect eyes
		    eyes_cascade.detectMultiScale( faceROI, eyes, 1.1, 2, 0 |CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

		    for( int j = 0; j < eyes.size(); j++ )
		     {
		       cv::Point center( faces[i].x + eyes[j].x + eyes[j].width*0.5, faces[i].y + eyes[j].y + eyes[j].height*0.5 );
		       int radius = cvRound( (eyes[j].width + eyes[j].height)*0.25 );
		       circle( cv_ptr->image, center, radius, cv::Scalar( 255, 0, 0 ), 4, 8, 0 );
		     }
		  }
		
		cv::imshow(WINDOW, cv_ptr->image);
		cv::waitKey(3);
		
		image_pub_.publish(cv_ptr->toImageMsg());
	}
 };

int main(int argc, char** argv){
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}


