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
static const char WINDOW[] = "Landing Pad Tracker";
static const char WINDOW2[] = "Unedited Stream";

bool set_A_next_frame = false;
bool set_B_next_frame = false;

Scalar detect_color_A1 = Scalar(50, 100, 100);
Scalar detect_color_A2 = Scalar(255, 255, 255);
Scalar color_A1_RGB = Scalar(0,0,0);
Scalar color_A2_RGB = Scalar(0,0,0);
Scalar detect_color_A_min = Scalar(50, 100, 100);
Scalar detect_color_A_max = Scalar(255,255, 255); 

Scalar detect_color_B1 = Scalar(0, 100, 100);
Scalar detect_color_B2 = Scalar(50, 255, 255);
Scalar color_B1_RGB = Scalar(0,0,0);
Scalar color_B2_RGB = Scalar(0,0,0);
Scalar detect_color_B_min = Scalar(100, 100, 100);
Scalar detect_color_B_max = Scalar(200,255, 255); 

int onetwoA = 0;
int onetwoB = 0;

Rect center_mat;

float min_val(float a, float b){
	if (a < b) return a;
	return b;
}

float max_val(float a, float b){
	if (a > b) return a;
	return b;
}

void recalculate_detection(int AorB){
	if(AorB == 0){
		detect_color_A_max = Scalar( max_val(detect_color_A1.val[0], detect_color_A2.val[0]),
					max_val(detect_color_A1.val[1], detect_color_A2.val[1]),
					max_val(detect_color_A1.val[2], detect_color_A2.val[2]));
		detect_color_A_min = Scalar( min_val(detect_color_A1.val[0], detect_color_A2.val[0]),
					min_val(detect_color_A1.val[1], detect_color_A2.val[1]),
					min_val(detect_color_A1.val[2], detect_color_A2.val[2]));
	
		detect_color_A_max = Scalar( detect_color_A_max.val[0] * 1.2, detect_color_A_max.val[1] * 1.2, detect_color_A_max.val[2] * 1.2);
		detect_color_A_min = Scalar( detect_color_A_min.val[0] * 0.8, detect_color_A_min.val[1] * 0.8, detect_color_A_min.val[2] * 0.8);			
		printf("A Hue range: (%.2f :: %.2f)\nA Saturation range: (%.2f :: %.2f)\nA Value range: (%.2f :: %.2f)\n\n",  
				detect_color_A_min.val[0], detect_color_A_max.val[0],
				detect_color_A_min.val[1], detect_color_A_max.val[1],
				detect_color_A_min.val[2], detect_color_A_max.val[2]);
	}
}


class ImageConverter
{

	
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	ros::Publisher pubFrameSize;
	ros::Publisher pubPosition;
	ros::Subscriber colorSwitch;
	
	geometry_msgs::Point lpad_A;
	std_msgs::Int32MultiArray frameSize;
	
	
	vector<Vec4i> hierarchy;
	
	Mat frame;
	Mat frame_hsv, frame_hsv_1;
	Mat frame_gray;
	Mat thresh_A, thresh_B;

	vector<vector<Point> > contours_A, contours_B;
	
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
	 	 
		 lpad_B.x = lpad_A.x = 0.0f;
		 lpad_B.y = lpad_A.y = 0.0f;
		 lpad_B.z = lpad_A.z = 0.0f;
	 	 
	 	 
	 	 namedWindow(WINDOW);
	//	 namedWindow(WINDOW2);
	 	}
	 	
	 ~ImageConverter()
	 {
	 	destroyWindow(WINDOW);
	 //	destroyWindow(WINDOW2);
	 }
	 
 	 void colorSetCb(const std_msgs::Bool::ConstPtr& msg)
	 {
		if(msg->data){
	 		set_A_next_frame = true;
	 		printf("Scanning new a color!\n");
	 	}
	 	if(!msg->data){
	 	//	set_B_next_frame = true;
	 		printf("Received bad message\n");
	 	
	 	}
	 
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
		center_mat = Rect(Point(frame.cols/2 - 10, frame.rows/2 - 10), Point(frame.cols/2 + 10, frame.rows/2 + 10));
		
		if(set_A_next_frame){

			Mat croppedHSV = frame_hsv(center_mat);
			Mat croppedFrame   = frame(center_mat);
			Scalar meanScalar, stdScalar;
			meanStdDev(croppedHSV, meanScalar, stdScalar);
		
			if(onetwoA == 0){
				detect_color_A2 = Scalar( meanScalar.val[0], meanScalar.val[1], meanScalar.val[2]);
				meanStdDev(croppedFrame, meanScalar, stdScalar);
				color_A2_RGB = Scalar( meanScalar.val[0], meanScalar.val[1], meanScalar.val[2]);		
			}
			if(onetwoA == 1){
				detect_color_A1 = Scalar( meanScalar.val[0], meanScalar.val[1], meanScalar.val[2]);
				meanStdDev(croppedFrame, meanScalar, stdScalar);
				color_A1_RGB = Scalar( meanScalar.val[0], meanScalar.val[1], meanScalar.val[2]);
			}
			
			onetwoA = 1 - onetwoA;
			recalculate_detection(0);	

			set_A_next_frame = false;
			waitKey(3);
			image_pub_.publish(cv_ptr->toImageMsg());
		}
				
		else{
			Scalar color = Scalar( 0,255,255 );
				
			rectangle(frame,center_mat, color, 2,8,0);
			
			rectangle(frame, Point(5,0), Point(12,7), color_A1_RGB , 4,8,0);
			rectangle(frame, Point(17,0), Point(24,7), color_A2_RGB, 4,8,0);
						
			inRange(frame_hsv, detect_color_A_min, detect_color_A_max, thresh_A);
			
			findContours(thresh_A, contours_A, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

			Moments moment;
			int x_mass, y_mass, x_center = frame.cols / 2, y_center = frame.rows / 2;
			CvPoint mass_center = cvPoint(0,0), img_center = cvPoint(x_center,y_center);
			double max_area = 0, current_area;
			int max_area_index = -1;
			
			for( size_t i = 0; i< contours_A.size(); i++ )
			{
				current_area = contourArea(contours_A[i]);
				if(max_area < current_area )  
				{
					max_area = current_area;
					max_area_index = i;
				}
			}
							
			if (max_area_index > -1){
				color = Scalar( 240, 100, 100 );
				drawContours( frame, contours_A, max_area_index, color, 2, 8, vector<Vec4i>(), 0, Point() );
				moment = moments( contours_A[max_area_index], false);
				lpad_A.x = mass_center.x = x_mass = moment.m10 / moment.m00;
				lpad_A.y = mass_center.y = y_mass = moment.m01 / moment.m00;
				lpad_A.z = 0;
				line(frame,mass_center, img_center, color, 4, 8 , 0);
				

				frameSize.data.clear();
				frameSize.data.push_back(frame.size().width);
				frameSize.data.push_back(frame.size().height);

				pubFrameSize.publish(frameSize);
				pubPosition.publish(lpad_A);				
			}
			
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


