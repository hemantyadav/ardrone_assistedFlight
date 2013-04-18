//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32MultiArray.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <math.h>
#include <iostream>
 
using namespace cv;
using namespace std; 
 
 
//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "OpenCV Stream";


bool use_default_color = true;
bool set_next_frame = false;
Scalar detect_color_min Scalar(50, 100, 100);
Scalar detect_color_max Scalar(255, 255, 255);



class ImageConverter
{

	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber colorSwitch;

	ros::Publisher pubFrameSize;
	ros::Publisher pubPosition;
	
	geometry_msgs::Point lpad;
	std_msgs::Int32MultiArray frameSize;
	
	Mat frame;
	Mat frame_hsv;
	Mat frame_gray;
	Mat thresh;
	Mat canny;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	public:
	 ImageConverter()
	 	: it_(nh_)
	 	{
	 //	image_pub_ = it_.advertise("ardrone/bottom/image_processed",1);
	// 	image_sub_ = it_.subscribe("ardrone/bottom/image_raw" ,1, &ImageConverter::imageCb, this);
	 	
	 	image_pub_ = it_.advertise("camera/image_processed",1);
	 	image_sub_ = it_.subscribe("camera/image_raw" ,1, &ImageConverter::imageCb, this);  
	 	 
	 	 pubFrameSize = nh_.advertise<std_msgs::Int32MultiArray>("landingPadTracker/frameSize", 5);
	 	 pubPosition = nh_.advertise<geometry_msgs::Point>("landingPadTracker/landingPadPosition", 5);
	 	 
	 //	 colorSwitch = nh_.subscribe("landingPadTracker/setColorMsg",1,&ImageConverter::colorSetCb, this);


	 	 
	 	 frameSize.data.clear();
	 	 
		 lpad.x = 0.0f;
		 lpad.y = 0.0f;
		 lpad.z = 0.0f;
	 	 
	 	 
	 	 namedWindow(WINDOW);
	 	}
	 	
	 ~ImageConverter()
	 {
	 	destroyWindow(WINDOW);
	 }
	 
	 void colorSetCb(const sensor_msgs::ImageConstPtr& msg)
	 {
	 	use_default_color = !use_default_color;
	 	set_next_frame = true;
	 	if(use_default_color)
	 		printf("Using default color\n");
	 	else	printf("Using new color\n");
	 
	 }
	 

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		RNG rng(12345);
		
		cv_bridge::CvImagePtr cv_ptr;
		
		try{ 
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		 }
		catch(cv_bridge::Exception& e){	ROS_ERROR("cv_bridge exception: %s", e.what()); return;	}
		
		
		frame = cv_ptr->image;
		cvtColor(frame, frame_hsv, CV_BGR2HSV);
		inRange(frame_hsv, detect_color_min, detect_color_max, thresh);
		
		findContours(thresh.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		Mat dst = Mat::zeros(frame.size(), frame.type());
		drawContours(dst,contours,-1, Scalar::all(255), CV_FILLED);
		
		dst &= frame;		
		
		   vector<vector<Point> >hull( contours.size() );
		   for( size_t i = 0; i < contours.size(); i++ )
		      {  convexHull( Mat(contours[i]), hull[i], false ); }

		   Mat drawing = Mat::zeros( thresh.size(), CV_8UC3 );
		   Moments moment;
		   int x_mass, y_mass, x_center = frame.cols / 2, y_center = frame.rows / 2;
		   CvPoint mass_center = cvPoint(0,0), img_center = cvPoint(x_center,y_center);
		   for( size_t i = 0; i< contours.size(); i++ )
		      {
		      	if (contourArea(contours[i]) > 400){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
			drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			drawContours( drawing, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			moment = moments( contours[i], false);
			lpad.x = mass_center.x = x_mass = moment.m10 / moment.m00;
			lpad.y = mass_center.y = y_mass = moment.m01 / moment.m00;
//			lpad.z = arcLength( contours[i], true);			
			lpad.z = 0.0f;

			line(drawing,mass_center, img_center, color, 1, 8 , 0);
			//printf("Contour Length: %.2f and Center of Contour: (%d, %d), Vector center Drone: (%d, %d) \r", 
			 arcLength( contours[i], true ),
			 x_mass,
			 y_mass,
			 x_center - x_mass,
			 y_center - y_mass
			 ); //this can be used to estimate height.  Output to ROS
			
			 frameSize.data.clear();
			 frameSize.data.push_back(frame.size().width);
			 frameSize.data.push_back(frame.size().height);
			 
			 pubFrameSize.publish(frameSize);
			 pubPosition.publish(lpad);			
			
			}
		      }
		
		imshow(WINDOW, drawing);
		waitKey(3);
		
		image_pub_.publish(cv_ptr->toImageMsg());
	}
 };

int main(int argc, char** argv){
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}


