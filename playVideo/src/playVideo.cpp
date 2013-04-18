/* play a video file */
 
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <unistd.h>

using namespace cv;
using namespace std;
 
int main(int argc, char** argv) {
	/* ROS publisher initializations */
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	//image_transport::Publisher pub = it.advertise("camera/image", 1);
	image_transport::Publisher pub = it.advertise("/ardrone/image_raw", 1);
    /* Create a window */
    //cvNamedWindow("playVideo.cpp output", CV_WINDOW_AUTOSIZE);
    /* capture frame from video file */
    CvCapture* capture = cvCreateFileCapture( argv[1]);
        
    /* Create IplImage to point to each frame */
    IplImage* frame;
    /* Loop until frame ended or ESC is pressed */
    while(1) {
        /* grab frame image, and retrieve */
        frame = cvQueryFrame(capture);
        /* exit loop if fram is null / movie end */
        if(!frame) break;
        /* display frame into window */
        //cvShowImage("playVideo.cpp output", frame);
        /* convert to publishable message */
        IplImage* image(frame);
        
        //IplImage temp_img, *img_out;
        //img_out = cvGetImage(image, &temp_img);
        sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image,"bgr8");
        
        if (nh.ok()) {
			pub.publish(msg);
			ros::spinOnce();
		}
		//sleep(0.1);
        /* if ESC is pressed then exit loop */
        char c = cvWaitKey(33);
        if(c==27) break;
    }
    /* destroy pointer to video */
    cvReleaseCapture(&capture);
    /* delete window */
    //cvDestroyWindow("playVideo.cpp output");
 
    return EXIT_SUCCESS;
}
