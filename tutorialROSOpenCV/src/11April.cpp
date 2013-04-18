//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
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
static const char WINDOW2[] = "Unedited Stream";

bool use_default_color = true;
bool set_next_frame = false;
Scalar detect_color_min = Scalar(50, 100, 100);
Scalar detect_color_max = Scalar(255, 255, 255);
Scalar color_min_RGB = Scalar(0,0,0);
Scalar color_max_RGB = Scalar(0,0,0);





class ImageConverter
{

	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	ros::Publisher pubFrameSize;
	ros::Publisher pubPosition;
	ros::Subscriber colorSwitch;
	
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
	 	image_pub_ = it_.advertise("ardrone/bottom/image_processed",1);
		image_sub_ = it_.subscribe("ardrone/bottom/image_raw" ,1, &ImageConverter::imageCb, this);
	 	
	 //	image_pub_ = it_.advertise("camera/image_processed",1);
	 //	image_sub_ = it_.subscribe("camera/image_raw" ,1, &ImageConverter::imageCb, this);  
	 	 
	 	colorSwitch = nh_.subscribe("landingPadTracker/setColorMsg",1,&ImageConverter::colorSetCb, this); 
	 	 
	 	 pubFrameSize = nh_.advertise<std_msgs::Int32MultiArray>("landingPadTracker/frameSize", 5);
	 	 pubPosition = nh_.advertise<geometry_msgs::Point>("landingPadTracker/landingPadPosition", 5);
	 	 frameSize.data.clear();
	 	 
		 lpad.x = 0.0f;
		 lpad.y = 0.0f;
		 lpad.z = 0.0f;
	 	 
	 	 
	 	 namedWindow(WINDOW);
	//	 namedWindow(WINDOW2);
	 	}
	 	
	 ~ImageConverter()
	 {
	 	destroyWindow(WINDOW);
	// 	destroyWindow(WINDOW2);
	 }
	 
 	 void colorSetCb(const std_msgs::Bool::ConstPtr& msg)
	 {

	 	set_next_frame = true;
	 	printf("Scanning new color!\n");
	 
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
		
		if(set_next_frame){

			Mat croppedHSV = frame_hsv(Rect(Point(frame.cols/2 - 15, frame.rows/2 - 15), Point(frame.cols/2 + 15, frame.rows/2 + 15)));
			Mat croppedFrame   = frame(Rect(Point(frame.cols/2 - 15, frame.rows/2 - 15), Point(frame.cols/2 + 15, frame.rows/2 + 15)));
			Scalar meanScalar, stdScalar;
			meanStdDev(croppedHSV, meanScalar, stdScalar);
			printf("Average Hue: %f, Saturation: %f, Value: %f\n", meanScalar.val[0], meanScalar.val[1], meanScalar.val[2]);
			printf("Std Dev Hue: %f, Saturation: %f, Value: %f\n", stdScalar.val[0], stdScalar.val[1], stdScalar.val[2]);
			detect_color_min = Scalar( meanScalar.val[0] - stdScalar.val[0], meanScalar.val[1] - 3*stdScalar.val[1], meanScalar.val[2] - 3*stdScalar.val[2]);
			detect_color_max = Scalar( meanScalar.val[0] + stdScalar.val[0], meanScalar.val[1] + 3*stdScalar.val[1], meanScalar.val[2] + 3*stdScalar.val[2]);
			
			
			meanStdDev(croppedFrame, meanScalar, stdScalar);
			color_min_RGB = Scalar( meanScalar.val[0] - stdScalar.val[0], meanScalar.val[1] - 3*stdScalar.val[1], meanScalar.val[2] - 3*stdScalar.val[2]);
			color_max_RGB = Scalar( meanScalar.val[0] + stdScalar.val[0], meanScalar.val[1] + 3*stdScalar.val[1], meanScalar.val[2] + 3*stdScalar.val[2]);
			

			set_next_frame = false;
			waitKey(3);
			image_pub_.publish(cv_ptr->toImageMsg());
		}
		
		
		
		else{
		
		
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
			   double max_area = 0, current_area;
			   int max_area_index = -1;
			   for( size_t i = 0; i< contours.size(); i++ )
				{
					current_area = contourArea(contours[i]);
			    		if(max_area < current_area )  
			      		{
			      			max_area = current_area;
			      			max_area_index = i;
			      		}
			
				}
				
			Scalar color = Scalar( 0,0,0 );
			rectangle(frame, Point(frame.cols/2 - 15, frame.rows/2 - 15), Point(frame.cols/2 + 15, frame.rows/2 + 15), color, 2,8,0);
			
			rectangle(frame, Point(5,0), Point(12,7), color_min_RGB , 4,8,0);
			rectangle(frame, Point(17,0), Point(24,7), color_max_RGB, 4,8,0);	
			if (max_area_index > -1){
				color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
				drawContours( frame, contours, max_area_index, color, 1, 8, vector<Vec4i>(), 0, Point() );
				drawContours( frame, hull, max_area_index, color, 1, 8, vector<Vec4i>(), 0, Point() );
				moment = moments( contours[max_area_index], false);
				lpad.x = mass_center.x = x_mass = moment.m10 / moment.m00;
				lpad.y = mass_center.y = y_mass = moment.m01 / moment.m00;
				//			lpad.z = arcLength( contours[i], true);			
				lpad.z = 0.0f;

				line(frame,mass_center, img_center, color, 4, 8 , 0);
				

				frameSize.data.clear();
				frameSize.data.push_back(frame.size().width);
				frameSize.data.push_back(frame.size().height);

				pubFrameSize.publish(frameSize);
				pubPosition.publish(lpad);				
			}
		
			//imshow(WINDOW, drawing);
			//imshow(WINDOW2, frame);
			imshow(WINDOW, frame);
			waitKey(3);
		
			image_pub_.publish(cv_ptr->toImageMsg());
		}
	}
 };

int main(int argc, char** argv){
	ros::init(argc, argv, "Landing_Pad_Detection");
	ImageConverter ic;
	ros::spin();
	return 0;
}


