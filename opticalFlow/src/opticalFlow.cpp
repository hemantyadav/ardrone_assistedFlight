#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/Bool.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <boost/timer.hpp>
#include <queue>
#define _USE_MATH_DEFINES
#include <math.h>
#include <ardrone_autonomy/Navdata.h>

#define GRIDSIZE 16
#define NODEHISTSZ 1
#define X_THRES_SIZE 3
#define Y_THRES_SIZE 5
// How many nearby collisions are needed to actually trigger avoidance
#define COL_FLTR_THRES 2

//#define USE_ISMOVING

// Control image drawing macros
//#define HIDE_NO_DANGER
#define HIDE_COL_REGION
#define SHOW_GRIDLINES
#define SHOW_AVOID_DIR

// Debug Defines
//#define D_PQ_ORDER
//#define D_COL_DETECT
//#define D_FINDAVOID
//#define D_VECTOR_VAR
//#define D_NAVDATA_MOVING
//#define D_AVOIDVECTOR
#define D_FLTR_COL

// STRUCTS #############################################################
struct FlowNode
{
	cv::Point2f start;
	cv::Point2f end;
	float magnitude;
	/*
	 * Positive numbers close to 0 mean it's moving directly at us!
	 * Positive numbers <180 are moving away
	 */ 
	float angle;
	int i;
	int j;
};
struct Compare
{
	// t2 has highest prio than t1 if t2 is earlier than t1
	/*
	 * f2 has higher priority when its angle is closer to 0+ and
	 * has a larger magnitude
	 */ 
	bool operator()(FlowNode& f1, FlowNode& f2 ){
		if(f2.angle < f1.angle){
			return true;
		}
		return false;
	}
};

//! Implements a circular buffer of FlowNodes to maintain history
struct NodeHist
{
	int curNodeIdx; // Indicates which is the most recent node
	int curHistSz;  // Stores how many of the buffers are valid
	FlowNode flowBuf[NODEHISTSZ];
};

// GLOBALS #############################################################
NodeHist 	imageGrid[GRIDSIZE][GRIDSIZE];

std::priority_queue<FlowNode, std::vector<FlowNode>, Compare> flowPQ;
cv::Point2f GblStart;
cv::Point2f GblEnd;

double vidWidth;
double vidHeight;
double cellWidth;
double cellHeight;

float xMin;
float xMax;
float yMin;
float yMax;

int SpinRate;
int SpinCount;

cv::Point2f center;
cv::Point2f avoidDir;
ardrone_autonomy::Navdata navdata;

// Published data
geometry_msgs::Point avoidanceDirection;
std_msgs::Bool willCollide;


//! Determines where the vector belongs and re-calculates the
//! new average of that location.
void AddVector(cv::Point2f start, cv::Point2f end, float magnitude, float angle)
{	
	double cellWidth		= vidWidth/GRIDSIZE;
	double cellHeight		= vidHeight/GRIDSIZE;

	int i = floor(start.x/cellWidth);
	int j = floor(start.y/cellHeight);
	assert(i <= GRIDSIZE);
	assert(j <= GRIDSIZE);
	
	int curNodeIdx = imageGrid[i][j].curNodeIdx;
	FlowNode curNode = imageGrid[i][j].flowBuf[curNodeIdx];
	
	curNode.i = i;
	curNode.j = j;
	
	/*
	 * Calculate the global flow vector
	 */ 
	 if(isnan(GblStart.x)){
		 GblStart.x = start.x;
	 }else{
		 GblStart.x = (GblStart.x + start.x)/2;
	 }
	 if(isnan(GblStart.y)){
		 GblStart.y = start.y;
	 }else{
		 GblStart.y = (GblStart.y + start.y)/2;
	 }
	 
	 if(isnan(GblEnd.x)){
		 GblEnd.x = end.x;
	 }else{
		 GblEnd.x = (GblEnd.x + end.x)/2;
	 }
	 if(isnan(GblEnd.y)){
		 GblEnd.y = end.y;
	 }else{
		 GblEnd.y = (GblEnd.y + end.y)/2;
	 }
	
	/*
	 * Calculate the average vector (start and end points), magnitude
	 * and angle to the center. Check each of these entries for NaN, that
	 * indicates that we need to initialize the node.
	 */ 
	if(isnan(curNode.start.x)){
		curNode.start.x = start.x;
	}else{
		curNode.start.x = (curNode.start.x + start.x)/2;
	}
	if(isnan(curNode.start.y)){
		curNode.start.y = start.y;
	}else{
		curNode.start.y = (curNode.start.y + start.y)/2;
	}
	if(isnan(curNode.end.x)){
		curNode.end.x = end.x;
	}else{
		curNode.end.x = (curNode.end.x + end.x)/2;
	}
	if(isnan(curNode.end.y)){
		curNode.end.y = end.y;
	}else{
		curNode.end.y = (curNode.end.y + end.y)/2;
	}
	if(isnan(curNode.magnitude)){
		curNode.magnitude = magnitude;
	}else{
		curNode.magnitude = (curNode.magnitude + magnitude)/2;
	}
	if(isnan(curNode.angle)){
		curNode.angle = angle;
	}else{
		curNode.angle = (curNode.angle + angle)/2;
	}
	
	imageGrid[i][j].flowBuf[curNodeIdx] = curNode;
}

//! Checks previous values to eliminate outliers
float angleHistVar(NodeHist hist)
{
	int historySz = hist.curHistSz;
	int curNodeIdx = hist.curNodeIdx;
	float sum1 = std::numeric_limits<float>::quiet_NaN();
	/*
	 * Calculate the Mean angle from the history.
	 * Start at the current node and examine history going backwards.
	 */ 
	bool histDone = false;
	int numLoops = 0;
	int idx = curNodeIdx;
	while(!histDone){
		if(numLoops == historySz){
			histDone = true;
			break;
		}
		float thisAng = hist.flowBuf[idx].angle;
		if(!isnan(thisAng)){
			if(isnan(sum1)){
				sum1 = thisAng;
			}else{
				sum1 = sum1 + thisAng;
			}
		}
		idx--;
		if((idx < 0) && (historySz == NODEHISTSZ)){
			idx = historySz - 1;
		}
		else{
			histDone = true;
			break;
		}
		numLoops++;
	}
	
	float mean = sum1/historySz;
	float sum2 = std::numeric_limits<float>::quiet_NaN();
	// Calculate the sum of the squares
	histDone = false;
	numLoops = 0;
	idx = curNodeIdx;
	while(!histDone){
		if(numLoops == historySz){
			histDone = true;
			break;
		}
		float thisAng = hist.flowBuf[idx].angle;
		if(!isnan(thisAng)){
			if(isnan(sum2)){
				sum2 = pow((thisAng - mean), 2.0f);
			}else{
				sum2 = sum2 + pow((thisAng - mean), 2.0f);
			}
		}
		idx--;
		if((idx < 0) && (historySz == NODEHISTSZ)){
			idx = historySz - 1;
		}
		else{
			histDone = true;
			break;
		}
		numLoops++;
	}
	
	float var = sum2/(historySz - 1);
	return var;
}


float magnitudeHistVar(NodeHist hist)
{
	int historySz = hist.curHistSz;
	float sum1 = std::numeric_limits<float>::quiet_NaN();
	// Calculate the mean angle value
	for(int i = 0; i < historySz; i++){
		float thisMag = hist.flowBuf[i].magnitude;
		if(!isnan(thisMag)){
			if(isnan(sum1)){
				sum1 = thisMag;
			}else{
				sum1 = sum1 + thisMag;
			}
		}else{
			historySz--;
		}
	}
	
	float mean = sum1/historySz;
	float sum2 = std::numeric_limits<float>::quiet_NaN();
	// Calculate the sum of the squares
	for(int i = 0; i < historySz; i++){
		float thisMag = hist.flowBuf[i].magnitude;
		if(!isnan(thisMag)){
			if(isnan(sum2)){
				sum2 = pow((thisMag - mean), 2.0f);
			}else{
				sum2 = sum2 + pow((thisMag - mean), 2.0f);
			}
		}
	}
	
	float var = sum2/(historySz - 1);
	return var;
}

//! Checks to see if this node could be a collision
bool potentialCollision(FlowNode thisNode){
	// Define the region of were flow ends are would be considered
	// dangerous
	//ROS_INFO("mag: %f, ang: %f, i:%i, j:%i", thisNode.magnitude, thisNode.angle, thisNode.i, thisNode.j);
	if((thisNode.angle < 20) && (thisNode.magnitude >10) 
		&& (thisNode.end.x > xMin) && (thisNode.end.x < xMax)
		&& (thisNode.end.y > yMin) && (thisNode.end.y < yMax)
		&& (thisNode.start.x > xMin) && (thisNode.start.x < xMax)
		&& (thisNode.start.y > yMin) && (thisNode.start.y < yMax))
	{
		return true;
	}else{
		return false;
	}
}

//! Pushes FlowNodes into the PQ after the matrix has been populated
void BuildPQ(cv::Mat image)
{
	// Remove any currently existing items
	while(!flowPQ.empty()){
		flowPQ.pop();
	}
	#ifndef HIDE_COL_REGION
	cv::Point2f topLeft;
	cv::Point2f botRight;
	topLeft.x = xMin; topLeft.y=yMin;
	botRight.x = xMax; botRight.y = yMax;
	cv::rectangle(image, topLeft, botRight, CV_RGB(0,255,0),5,8);
	#endif
	#ifdef SHOW_AVOID_DIR
	cv::Scalar colVecColor = CV_RGB(0,255,0);
	cv::line(image, center, avoidDir, colVecColor, 2); //green
	cv::rectangle(image, avoidDir-cv::Point2f(3,3), avoidDir+cv::Point2f(2,2), colVecColor,   4);
	#endif
	for(int i = 0; i<GRIDSIZE; i++){
		#ifdef SHOW_GRIDLINES
			cv::Point2f hStart;
			cv::Point2f hEnd;
			cv::Point2f vStart;
			cv::Point2f vEnd;
			cv::Scalar gridColor = CV_RGB(0,255,0);
			hStart.x = cellWidth*(i+1); hStart.y = 0;
			hEnd.x = cellWidth*(i+1); hEnd.y = vidHeight;
			vStart.x = 0; vStart.y = cellHeight*(i+1);
			vEnd.x = vidWidth; vEnd.y = cellHeight*(i+1);
			cv::line(image, hStart, hEnd, gridColor, 1);
			cv::line(image, vStart, vEnd, gridColor, 1);
		#endif
		for(int j = 0; j<GRIDSIZE; j++){
			bool dangerVector = true;
			int curNodeIdx = imageGrid[i][j].curNodeIdx;
			FlowNode pqElem = imageGrid[i][j].flowBuf[curNodeIdx];
			// NaN presence mean there were no flow vectors, ignore
			if(!isnan(pqElem.angle) && !isnan(pqElem.magnitude)){
				cv::Scalar lineColor;
				int line_thickness = 2;
				/*
				 * Compensate for the global flow vector
				 
				pqElem.end.x = pqElem.end.x - GblEnd.x;
				pqElem.end.y = pqElem.end.y - GblEnd.y;
				if(pqElem.end.x < 0){
					pqElem.end.x = 0.0f;
				}
				if(pqElem.end.x > vidWidth){
					pqElem.end.x = vidWidth;
				}
				if(pqElem.end.y < 0){
					pqElem.end.y = 0.0f;
				}
				if(pqElem.end.y > vidHeight){
					pqElem.end.y = vidHeight;
				}
				* */ 
				// These are immenant collisions
				if(potentialCollision(pqElem) == true){
					// Dangerous, red line
					lineColor = cv::Scalar(0, 0, 255);
					// Add this to the queue of nodes to path find around
					flowPQ.push(imageGrid[i][j].flowBuf[curNodeIdx]);
				}else{
					// Not dangerous, blue line
					dangerVector = false;
					lineColor = cv::Scalar(255, 0 , 0);
				}
				#ifdef HIDE_NO_DANGER
				if(dangerVector){
				#endif
				// Now we draw the main line of the arrow.
				cv::line(image, pqElem.start, pqElem.end, lineColor, line_thickness);
				
				// Draw a rectangle at the end of the vector for direction
				cv::rectangle(image, pqElem.end-cv::Point2f(3,3), pqElem.end+cv::Point2f(2,2), lineColor,   4);
				#ifdef HIDE_NO_DANGER
				}
				#endif
				
			}
		}
	}
}

//! Sets inital values to the image matrix
/**
 * Initializes each FlowNode in the matrix to:
 * start.x 		= NaN
 * start.y 		= NaN
 * end.x		= NaN
 * end.y		= NaN
 * magnitude	= NaN
 * angle		= NaN
 * i			= i (index in array)
 * j			= j (indes in array)
 */ 
void initFlowNode(FlowNode &node, int i, int j)
{
	// Initialize all floats to NaN
	node.start.x = std::numeric_limits<float>::quiet_NaN();
	node.start.y = std::numeric_limits<float>::quiet_NaN();
	node.end.x = std::numeric_limits<float>::quiet_NaN();
	node.end.y = std::numeric_limits<float>::quiet_NaN();
	node.magnitude = std::numeric_limits<float>::quiet_NaN();
	node.angle = std::numeric_limits<float>::quiet_NaN();
				
	// Store index so we can jump to the imageGrid from the PQ.
	node.i = i;
	node.j = j;
}

//! Inits current history or all history
void InitImageMatrix(bool initHistBuf)
{
	for(int i = 0; i<GRIDSIZE; i++){
		for(int j = 0; j<GRIDSIZE; j++){
			if(initHistBuf){
				//Loop those each node hist item
				for(int k = 0; k<NODEHISTSZ; k++){
					//clear out node varibles
					FlowNode clearNode = imageGrid[i][j].flowBuf[k];
					// Initialize all floats to NaN
					initFlowNode(clearNode, i,j);
					// Update the node
					imageGrid[i][j].flowBuf[k] = clearNode;
				}
				// Reset the starting index
				imageGrid[i][j].curNodeIdx = 0;
				imageGrid[i][j].curHistSz = 0;
			// Clear the next history buffer
			}else{
				int curNodeIdx = imageGrid[i][j].curNodeIdx;
				// Caculate where to store the new node
				int nextNodeIdx = (curNodeIdx + 1) % NODEHISTSZ;
				// Clear the node at this location
				FlowNode clearNode = imageGrid[i][j].flowBuf[nextNodeIdx];
				initFlowNode(clearNode, i,j);
				// Update the current node index
				imageGrid[i][j].curNodeIdx = nextNodeIdx;
				//update number stored until max is reached
				if(imageGrid[i][j].curHistSz < NODEHISTSZ){
					imageGrid[i][j].curHistSz++;
				}
			}
		}
	}
}

//! Calculates the angle of approach relative to the center of
//! the screen.
float innerAngle(cv::Point2f oldPoint, cv::Point2f newPoint)
{
	bool movingToCenter = false;
	float distOld = sqrt(pow(oldPoint.x-center.x, 2))
						+ sqrt(pow(oldPoint.y - center.y, 2));
	float distNew = sqrt(pow(newPoint.x-center.x, 2)) 
						+ sqrt(pow(newPoint.y - center.y, 2)); 
	
	cv::Point2f A;
	cv::Point2f B;
	
	//Object is moving towards us
	if(distNew < distOld){
		movingToCenter = true;
	}
	
	cv::Point2f u;
	cv::Point2f v;
	//Create our vectors to find the angle between
	u.x = center.x - oldPoint.x;
	u.y = center.y - oldPoint.y;
	v.x = newPoint.x - oldPoint.x;
	v.y = newPoint.y - oldPoint.y;
 
	// Find angle
	float denom = sqrt(pow(u.x,2)+pow(u.y,2)) * sqrt(pow(v.x,2)+pow(v.y, 2));
	float numerator = v.x*u.x+v.y*u.y;
	float angle = acos(numerator/denom);
	if(angle != 0){
		if(movingToCenter){  
			angle = (angle*180/M_PI);
		} 
		else
		{
			// Negative for moving away from us
			angle = (angle*180/M_PI) + 180;
		}
	}
	return angle;
} 

// ######################################################################
class OpticalFlow
{
  public:
    OpticalFlow();
    ~OpticalFlow();

  protected:
    void imageCallback(sensor_msgs::ImageConstPtr const & input_img_ptr);
    void navdataCallback(ardrone_autonomy::Navdata const & new_navdata);
    void CollisionAvoidance();
    void FindAvoidVector(FlowNode collisionNode);
	bool stopAvoidanceTimer();
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber navdata_sub;
    ros::Publisher pubAvoidDirection;
    ros::Publisher pubCollisionDetect;
    
    
    cv::Mat key_image_;
    std::vector<cv::Point2f> key_corners_;

    int num_keypoints_param_;
    double matchscore_thresh_param_;
};

OpticalFlow::OpticalFlow() : it_(nh_)
{
	// Subscriptions/Advertisements
	image_sub_ = it_.subscribe("ardrone/front/image_raw", 1, &OpticalFlow::imageCallback, this);
	navdata_sub = nh_.subscribe("ardrone/navdata",  1, &OpticalFlow::navdataCallback, this);
	
	//Published messages
	pubAvoidDirection = nh_.advertise<geometry_msgs::Point>("collisionAvoid/avoidanceDirection", 5);
	pubCollisionDetect = nh_.advertise<std_msgs::Bool>("collisionDetect/willCollide", 5);
	
	nh_.param("num_keypoints", num_keypoints_param_, 500);
	nh_.param("matchscore_thresh", matchscore_thresh_param_, 10e8);
	// Clear the willCollide flag upon each loop
	pubCollisionDetect.publish(willCollide);
	

}

OpticalFlow::~OpticalFlow() 
{ 
}

 

// ######################################################################
//! Draw the features onto the image, and draw lines between matches
void drawFeatures(cv::Mat & image,
std::vector<cv::Point2f> const & old_corners, std::vector<cv::Point2f> const & new_corners)
{
	//Set image varibles for PQ

	assert(old_corners.size() == new_corners.size());
	InitImageMatrix(true);
	for(size_t i=0; i<new_corners.size(); ++i)
	{
		float const pointDist = sqrt(pow(new_corners[i].x - old_corners[i].x, 2) + pow(new_corners[i].y - old_corners[i].y, 2));
		float const angle = innerAngle(old_corners[i], new_corners[i]);
		//Only track points that have a valid angle
		if(!isnan(angle) && !isinf(angle) && (pointDist != 0)){
			AddVector(old_corners[i], new_corners[i], pointDist, angle);
		}
	}
	BuildPQ(image);
	
	#ifdef D_PQ_ORDER
		ROS_INFO("Start PQ READ");
		FlowNode pqElem;
		while(!flowPQ.empty())
		{
			pqElem = flowPQ.top();
			flowPQ.pop();
			ROS_INFO("mag: %f, ang: %f, i:%i, j:%i", pqElem.magnitude, pqElem.angle, pqElem.i, pqElem.j);
		}
		ROS_INFO("End PQ READ");
	#endif
}


// ######################################################################
//! Filter out all points from p1 and p2 who's corresponding status byte == 0
void filterPoints(std::vector<cv::Point2f> & p1, std::vector<cv::Point2f> & p2,
    std::vector<unsigned char> const & status)
{
	std::vector<cv::Point2f> p1_filt;     p1_filt.reserve(p1.size());
	std::vector<cv::Point2f> p2_filt; p2_filt.reserve(p1.size());

	std::vector<cv::Point2f>::iterator p1_it = p1.begin();
	std::vector<cv::Point2f>::iterator p2_it = p2.begin();
	std::vector<unsigned char>::const_iterator status_it = status.begin();

	while(status_it != status.end())
	{
		//Don't track points that go off the screen
		if((*status_it > 0) && (p2_it->x < vidWidth) && (p2_it->y < vidHeight)
		&& (p1_it->x < vidWidth) && (p1_it->y < vidHeight)){
			p1_filt.push_back(*p1_it);
			p2_filt.push_back(*p2_it);
		}
		++p1_it;
		++p2_it;
		++status_it;
	}
	p1 = p1_filt;
	p2 = p2_filt;
}

//! merge newFeatures into features
/*! @param features	The currently tracked features
 *  @param newFeatures	New features found in the frame
 * 
 */ 
void mergeFeatures(std::vector<cv::Point2f> & features, std::vector<cv::Point2f> & newFeatures)
{
	features.insert(features.end(), newFeatures.begin(), newFeatures.end());
}

// ######################################################################
//! Perform LK tracking.
/*! @param key_image The old keyframe image
    @param curr_image The new incoming image
    @param corners The old corners, found (e.g. by cv::goodFeaturesToTrack) in the key_image
    @param new_corners A vector which will be filled with the locations of the corners in curr_image
    
*/
void trackFeatures(cv::Mat key_image, cv::Mat curr_image, std::vector<cv::Point2f> & corners, std::vector<cv::Point2f> & new_corners)
{
	cv::Size const searchWindow(40, 40);

	new_corners.resize(corners.size());

	if(corners.size() == 0) return;

	// Perform the forward LK step
	std::vector<unsigned char> status(corners.size());
	std::vector<float> error(corners.size());
	calcOpticalFlowPyrLK(key_image, curr_image, corners, new_corners, status, error, searchWindow, 3);

	// Filter out any untrackable points
	filterPoints(corners, new_corners, status);

	if(corners.size() == 0) return;

	// Filter out points that are untrackable by LK (their status byte == 0)
	std::vector<cv::Point2f> filt_corners;
	std::vector<cv::Point2f> filt_old_corners;
	std::vector<cv::Point2f>::iterator corners_it = corners.begin();
	std::vector<cv::Point2f>::iterator new_corners_it = new_corners.begin();
	std::vector<unsigned char>::iterator status_it = status.begin();
	while(status_it != status.end())
	{
		if(*status_it)
		{
			filt_old_corners.push_back(*corners_it);
			filt_corners.push_back(*new_corners_it);
		}
		++corners_it;
		++new_corners_it;

		++status_it;
	}
	new_corners = filt_corners;
	corners     = filt_old_corners;
}

//! When a new image is posted we run the algo.
void OpticalFlow::imageCallback(sensor_msgs::ImageConstPtr const & input_img_ptr)
{
	cv_bridge::CvImageConstPtr cv_ptr_gray;
	cv_bridge::CvImageConstPtr cv_ptr_color;
	//Increment the number of image frames we have seen
	try
	{
		cv_ptr_gray = cv_bridge::toCvShare(input_img_ptr, sensor_msgs::image_encodings::MONO8);
		cv_ptr_color = cv_bridge::toCvShare(input_img_ptr, sensor_msgs::image_encodings::BGR8);
		
		
		cv::Mat input_image_gray = cv_ptr_gray->image;
		cv::Mat input_image_color = cv_ptr_color->image;
		
		//Set the video params
		
		vidWidth = input_image_gray.cols;
		vidHeight = input_image_gray.rows;
		cellWidth = vidWidth/GRIDSIZE;
		cellHeight = vidHeight/GRIDSIZE;
		center.x = input_image_gray.cols/2.0f;
		center.y = input_image_gray.rows/2.0f;
		
		xMin = center.x - (vidWidth/X_THRES_SIZE);
		xMax = center.x + (vidWidth/X_THRES_SIZE);
		yMin = center.y - (vidHeight/Y_THRES_SIZE);
		yMax = center.y + (vidHeight/Y_THRES_SIZE);

		// Grab a new keyframe whenever we have lost more than 1/2 of our tracks
		std::vector<cv::Point2f> new_features;
		if(key_corners_.size() < (size_t(num_keypoints_param_ / 2)))
		{
			cv::goodFeaturesToTrack(input_image_gray, new_features, (num_keypoints_param_ - key_corners_.size()), 0.01, 30);
			//add the new features to the existing ones
			mergeFeatures(key_corners_, new_features);
			key_image_ = input_image_gray.clone(); 
		}

		// Track the features from the keyframe to the current frame
		std::vector<cv::Point2f> new_corners;
		trackFeatures(key_image_, input_image_gray, key_corners_, new_corners);

		// Draw the features on the input image
		if(key_corners_.size()){
			drawFeatures(input_image_color, key_corners_, new_corners);
			// Read the PQ and determine immenant collisions
			CollisionAvoidance();
		}
		
		cv::imshow("tracker (press key for keyframe)", input_image_color);
		cv::waitKey(2);
	}
	catch(cv_bridge::Exception & e)
	{
		ROS_ERROR("%s:%d: Exception: %s", __FILE__, __LINE__, e.what());  
		return;
	}
}

//! Callback for new nav data
void OpticalFlow::navdataCallback(ardrone_autonomy::Navdata const & new_navdata)
{
	navdata = new_navdata;
}
 

std::vector<FlowNode> GetNodeNeighbors(int i, int j)
{
	//ROS_INFO("curent node at (%i,%i)", i, j);
	// TODO filter nodes based on history
	std::vector<FlowNode> neighbors;
	for(int x = (i-1); x < (i+2); x++){
		for(int y = (j-1); y < (j+2); y++){
			// Do not add the center node
			if((0 <= x) && (x < GRIDSIZE) && (0 <= y) && (y < GRIDSIZE)){
				if((x == i) && (y == j)){
					continue;
				}
				int curNodeInHist = imageGrid[x][y].curNodeIdx;
				FlowNode nbr = imageGrid[x][y].flowBuf[curNodeInHist];
				neighbors.push_back(nbr);
			}
		}
	}
	return neighbors;
}

//! Adds new FlowNodes to the open list
/*! @param openList	The current open nodes
 *  @param newNodes	The new nodes discovered to add to openList
 * 
 */ 
void addToOpenList(std::vector<FlowNode> & openList, std::vector<FlowNode> & newNodes)
{
	openList.insert(openList.end(), newNodes.begin(), newNodes.end());
}

//! Calculates the vector for the drone to move in that direction
void OpticalFlow::FindAvoidVector(FlowNode collisionNode)
{
	// Find where on the image the collision was detected so we can
	// path find
	int i = collisionNode.i;
	int j = collisionNode.j;
	
	int numNbrCollisions = 0;
	
	std::vector<FlowNode> neighbors = GetNodeNeighbors(i,j);
	std::vector<FlowNode> openList;
	addToOpenList(openList, neighbors);
	std::vector<FlowNode>::iterator open_it = openList.begin();

	FlowNode bestNode = collisionNode;
	#ifdef D_FINDAVOID
		ROS_INFO("CollisionNode:  mag: %f, ang: %f, i:%i, j:%i", collisionNode.magnitude, collisionNode.angle, collisionNode.i, collisionNode.j);
	#endif
	while(open_it != openList.end()){
		#ifdef D_FINDAVOID
			ROS_INFO("Candidates: mag: %f, ang: %f, i:%i, j:%i", open_it->magnitude, open_it->angle, open_it->i, open_it->j);
		#endif
		// NaN means there is no flow so this is a good region
		if(potentialCollision(*open_it)){
			numNbrCollisions++;
		}
		if((isnan(open_it->angle)) || (isnan(open_it->magnitude))){
			bestNode = *open_it;
			break;
		}
		if((open_it->angle > bestNode.angle) || (open_it->magnitude < bestNode.magnitude)){
			bestNode = *open_it;
		}
		++open_it;
	}
	#ifdef D_FINDAVOID
		ROS_INFO("BEST: mag: %f, ang: %f, i:%i, j:%i", bestNode.magnitude, bestNode.angle, bestNode.i, bestNode.j);
	#endif
	if((bestNode.i == collisionNode.i) && (bestNode.j == collisionNode.j)){
		ROS_WARN("BEST NODE WAS THE CURRENT NODE!");
		return;
	}
	// Other collisions nearby so it's most likely a real collision
	// Set the collision flag and set the direction to move
	if(numNbrCollisions >= COL_FLTR_THRES){
		#ifdef D_FLTR_COL
			ROS_INFO("Collision threshold exceeded, will avoid collision");
		#endif
		// Set the imminent collision flag
		willCollide.data = true;
		pubCollisionDetect.publish(willCollide);
	}
	float moveX;
	float moveY;
	float cellXPos = bestNode.i * cellWidth;
	float cellYPos = bestNode.j * cellHeight;
	avoidDir.x = cellXPos;
	avoidDir.y = cellYPos;
	moveX = -1.0f*((vidWidth/2.0f)-cellXPos)/(vidWidth/2.0f);
	moveY = -1.0f*((vidHeight/2.0f)-cellYPos)/(vidHeight/2.0f);
	#ifdef D_AVOIDVECTOR
		ROS_INFO("movX: %f, movY: %f", moveX, moveY);
	#endif
	avoidanceDirection.x = moveX;
	avoidanceDirection.y = moveY;
	avoidanceDirection.z = 0.0f;
	pubAvoidDirection.publish(avoidanceDirection);	
}

//! If the drone is moving then do collision avoidance
bool isMoving(){
	#ifdef D_NAVDATA_MOVING
		ROS_INFO("vx: %f,vy: %f, vz: %f", navdata.vx, navdata.vy, navdata.vz);
	#endif
	if(navdata.vx < 100){
		return false;
	}else{
		return true;
	}
}

//! This routine checks for an immenant collision and path finds
//! around the collision.
void OpticalFlow::CollisionAvoidance()
{
	#ifdef USE_ISMOVING
	if(!isMoving()){
		return;
	}
	#endif
	if(stopAvoidanceTimer() == true){
		return;
	}
	// TODO add some filtering using the flowNode buffer
	//bool noImminent = false;
	FlowNode pqElem;
	if(!flowPQ.empty()){
		while(!flowPQ.empty())
		{
			pqElem = flowPQ.top();
			flowPQ.pop();
			
			// Check if collision is imminent
			if(potentialCollision(pqElem)){
				// Generate the vector to publish
				FindAvoidVector(pqElem);
				break;
			}
			else
			{
				//ROS_INFO("NO COL");
				willCollide.data = false;
				pubCollisionDetect.publish(willCollide);
				break;
			}
		}
	}else{
		willCollide.data = false;
		pubCollisionDetect.publish(willCollide);
	}
	//ROS_INFO("mag: %f, ang: %f, i:%i, j:%i", pqElem.magnitude, pqElem.angle, pqElem.i, pqElem.j);
	//ROS_INFO("Will collide %s",(willCollide.data)?"true":"false");
	return;
}

bool OpticalFlow::stopAvoidanceTimer(){
	if(willCollide.data == true){
		if(SpinCount > (SpinRate/2))
		{	
			//stop moving
			avoidanceDirection.x = 0.0f;
			avoidanceDirection.y = 0.0f;
			avoidanceDirection.z = 0.0f;
			pubAvoidDirection.publish(avoidanceDirection);
			
			ROS_INFO("STOP COLLISON AVOIDANCE");
			// Turn off collision flag
			willCollide.data = false;
			pubCollisionDetect.publish(willCollide);
			return true;
		}
	}
	return false;	
}

//! Controls the rate in which optical the algo is ran
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "contrast_enhancer");
	InitImageMatrix(true);
	willCollide.data = false;
	OpticalFlow ofInit;
	SpinRate = 30;
	SpinCount = 0;
	ros::Rate r(SpinRate);
	while(ros::ok()){
		// Init global mag and angle
		GblStart.x = std::numeric_limits<float>::quiet_NaN();
		GblStart.y = std::numeric_limits<float>::quiet_NaN();
		GblEnd.x = std::numeric_limits<float>::quiet_NaN();
		GblEnd.y = std::numeric_limits<float>::quiet_NaN();
		SpinCount++;
		OpticalFlow of;
		ros::spinOnce();
		r.sleep();
		//ROS_INFO("Will collide %s",(willCollide.data)?"true":"false");
	}
	return 0;
}
