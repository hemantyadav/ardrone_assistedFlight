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
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "OpenCV Stream";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	cv::Mat contours;
	cv::Mat gray_image;
	std::vector<cv::Mat> channels;
	cv::Mat hsv;	
	public:
	 ImageConverter()
	 	: it_(nh_)
	 	{
	 	 image_pub_ = it_.advertise("ardrone/bottom/image_processed",1);
	 	 image_sub_ = it_.subscribe("ardrone/bottom/image_raw" ,1, &ImageConverter::imageCb, this);
	 	 
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
		
		cv::cvtColor( cv_ptr->image, hsv, CV_RGB2HSV );
		cv::split(hsv, channels);
		gray_image = channels[0];
		cv::Canny(cv_ptr->image, contours, 35, 90);
		cv_ptr->image = contours;
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

