#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <stdlib.h>
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"

//-----These parameters need to be adjusted by the user -----------------------

//Adjust camera resolution here
int  imageWidth= 640;
int  imageHeight = 400;

//Adjust the black circle diameter [m]
float circleDiameter = 0.037;

/*Adjust the X and Y dimensions of the coordinate system*/
float fieldLength = 1.00;
float fieldWidth = 1.00;
//----------------------------------------------------------------------------

/*robot detection variables*/
int numBots = 0;		//num of robots to track
int numFound = 0;		//num of robots detected in the last step
int numStatic = 0;		//num of non-moving robots
CCircleDetect *detectorArray[MAX_PATTERNS];	//detector array (each pattern has its own detector)
SSegment currentSegmentArray[MAX_PATTERNS];	//segment array (detected objects in image space)
SSegment lastSegmentArray[MAX_PATTERNS];	//segment position in the last step (allows for tracking)
STrackedObject objectArray[MAX_PATTERNS];	//object array (detected objects in metric space)
CTransformation *trans;				//allows to transform from image to metric coordinates

/*variables related to (auto) calibration*/
const int autoCalibrationSteps = 30; 			//how many measurements to average to estimate calibration pattern position (automatic calib)
const int autoCalibrationPreSteps = 10;		//how many measurements to discard before starting to actually auto-calibrating (automatic calib)
STrackedObject calib[5];			//array to store calibration patterns positions
int calibStep = 0;				//actual calibration step (num of measurements of the actual pattern)
bool autocalibrate = false;			//is the autocalibration in progress ?
ETransformType lastTransformType = TRANSFORM_2D;//pre-calibration transform (used to preserve pre-calibation transform type)
int wasBots = 1;				//pre-calibration number of robots to track (used to preserve pre-calibation number of robots to track)

/*program flow control*/
bool saveLog = true;		//save log to output folder?
FILE *robotPositionLog = NULL;	//file to log robot positions
int frameID = 0;
CTimer globalTimer;

/*communication input (ROS 2 image topic) and output (ROS 2 topic)*/
CRawImage *image;
rclcpp::Node::SharedPtr node;
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr posePub;

/*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
void autocalibration()
{
	bool saveVals = true;
	for (int i = 0;i<numBots;i++){
		if (detectorArray[i]->lastTrackOK == false) saveVals=false;
	}
	if (saveVals){
		int index[] = {0,0,0,0};
		int maxEval = 0;
		int eval = 0;
		int sX[] = {-1,+1,-1,+1};
		int sY[] = {+1,+1,-1,-1};
		for (int b = 0;b<4;b++)
		{
			maxEval = -10000000;
			for (int i = 0;i<numBots;i++)
			{
				eval = 	sX[b]*currentSegmentArray[i].x +sY[b]*currentSegmentArray[i].y;
				if (eval > maxEval)
				{
					maxEval = eval;
					index[b] = i;
				}
			}
		}
		printf("INDEX: %i %i %i %i\n",index[0],index[1],index[2],index[3]);
		for (int i = 0;i<4;i++){
			if (calibStep <= autoCalibrationPreSteps) calib[i].x = calib[i].y = calib[i].z = 0;
			calib[i].x+=objectArray[index[i]].x;
			calib[i].y+=objectArray[index[i]].y;
			calib[i].z+=objectArray[index[i]].z;
		}
		calibStep++;
		if (calibStep == autoCalibrationSteps){
			for (int i = 0;i<4;i++){
				calib[i].x = calib[i].x/(autoCalibrationSteps-autoCalibrationPreSteps);
				calib[i].y = calib[i].y/(autoCalibrationSteps-autoCalibrationPreSteps);
				calib[i].z = calib[i].z/(autoCalibrationSteps-autoCalibrationPreSteps);
			}
			trans->calibrate2D(calib,fieldLength,fieldWidth);
			trans->calibrate3D(calib,fieldLength,fieldWidth);
			trans->calibrate4D(calib,fieldLength,fieldWidth);
			numBots = wasBots;
			trans->saveCalibration("../etc/default.cal");
			trans->transformType = lastTransformType;
			autocalibrate = false;
			RCLCPP_INFO(node->get_logger(),"Autocalibration finished, saved to ../etc/default.cal.");
		}
	}
}

/*service whycon/autocalibrate - searches for 4 outermost circular patterns and uses them to establish the coordinate system*/
void autocalibrateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
	if (numBots < 4){
		res->success = false;
		res->message = "autocalibration needs at least 4 patterns";
		return;
	}
	if (autocalibrate){
		res->success = false;
		res->message = "autocalibration already in progress";
		return;
	}
	calibStep = 0;
	lastTransformType=trans->transformType;
	wasBots = numBots;
	autocalibrate = true;
	trans->transformType=TRANSFORM_NONE;
	res->success = true;
	res->message = "autocalibration started";
}

/*initialize logging*/
bool initializeLogging()
{
	char logFileName[1000];
	char timeStr[100];
	time_t timeNow;
	time(&timeNow);
	strftime(timeStr, sizeof(timeStr), "%Y-%m-%d_%H-%M-%S",localtime(&timeNow));
	sprintf(logFileName,"output/WhyCon_%s.txt",timeStr);
	robotPositionLog = fopen(logFileName,"w");
	if (robotPositionLog == NULL)
	{
		fprintf(stderr,"Cannot write to log file %s. Does the \"output\" directory exist?\n",logFileName);
		return false;
	}
	return true;
}

/*process one mono8 image*/
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	if (msg->encoding != "mono8"){
		RCLCPP_ERROR_THROTTLE(node->get_logger(),*node->get_clock(),5000,"Expected mono8 image, got '%s'.",msg->encoding.c_str());
		return;
	}
	if ((int)msg->width != imageWidth || (int)msg->height != imageHeight){
		RCLCPP_ERROR_THROTTLE(node->get_logger(),*node->get_clock(),5000,"Expected %ix%i image, got %ix%i. Adjust imageWidth and imageHeight in main/whycon.cpp.",imageWidth,imageHeight,msg->width,msg->height);
		return;
	}
	if (msg->step < msg->width || msg->data.size() < (size_t)msg->step*msg->height){
		RCLCPP_ERROR_THROTTLE(node->get_logger(),*node->get_clock(),5000,"Malformed image message.");
		return;
	}
	//the detector modifies the image, so it works on a copy
	image->copyFrom(msg->data.data(),msg->step);

	numFound = numStatic = 0;
	int64_t frameTime = globalTimer.getRealTime();

	//track the robots found in the last attempt
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
		}
	}

	//search for untracked (not detected in the last frame) robots
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid == false){
			lastSegmentArray[i].valid = false;
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
		}
		if (currentSegmentArray[i].valid == false) break;		//does not make sense to search for more patterns if the last one was not found
	}

	//perform transformations from camera to world coordinates
	geometry_msgs::msg::PoseArray poses;
	poses.header = msg->header;
	for (int i = 0;i<numBots;i++){
		if (currentSegmentArray[i].valid){
			objectArray[i] = trans->transform(currentSegmentArray[i],false);
			numFound++;
			if (currentSegmentArray[i].x == lastSegmentArray[i].x) numStatic++;

			geometry_msgs::msg::Pose pose;
			pose.position.x = objectArray[i].x;
			pose.position.y = objectArray[i].y;
			pose.position.z = objectArray[i].z;
			pose.orientation.w = 1.0;
			if (trans->transformType == TRANSFORM_2D){
				pose.orientation.z = sin(objectArray[i].yaw/2);
				pose.orientation.w = cos(objectArray[i].yaw/2);
			}
			poses.poses.push_back(pose);
		}
	}
	printf("Pattern detection time: %i us. Found: %i Static: %i.\n",globalTimer.getTime(),numFound,numStatic);

	//send the data to other systems
	posePub->publish(poses);

	//establishing the coordinate system by autocalibration
	if (autocalibrate && numFound == numBots) autocalibration();

	for (int i = 0;i<numBots;i++){
		if (robotPositionLog != NULL) fprintf(robotPositionLog,"Frame %i Time %ld Object %03i %03i %.5f %.5f %.5f \n",frameID,frameTime,i,currentSegmentArray[i].ID,objectArray[i].x,objectArray[i].y,objectArray[i].yaw);
	}
	frameID++;
}

//process command line arguments (ROS arguments already removed)
void processArgs(const std::vector<std::string> &args)
{
	if (args.size() > 2) numBots = atoi(args[2].c_str());
	if (numBots < 0) numBots = 0;
	if (numBots > MAX_PATTERNS) numBots = MAX_PATTERNS;
	for (size_t i = 3;i<args.size();i++){
		if (args[i] == "nolog") saveLog=false;
	}
}

int main(int argc,char* argv[])
{
	rclcpp::init(argc,argv);
	std::vector<std::string> args = rclcpp::remove_ros_arguments(argc,argv);
	if (args.size() < 2) {
		fprintf(stderr,"usage: %s image_topic num_robots [nolog]\ne.g. %s /camera/image_raw 1\n",args[0].c_str(),args[0].c_str());
		rclcpp::shutdown();
		return 0;
	}

	//initialize logging system
	processArgs(args);
	if (saveLog) initializeLogging();

	//initialize image structures, coordinate transformation modules
	image = new CRawImage(imageWidth,imageHeight);
	trans = new CTransformation(imageWidth,imageHeight,circleDiameter,true);
	trans->transformType = TRANSFORM_NONE;		//in our case, 2D is the default

	//initialize the circle detectors - each circle has its own detector instance
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(imageWidth,imageHeight,i);

	//setup timer to assess system performance
	globalTimer.reset();
	globalTimer.start();

	//ROS 2 input and output
	node = rclcpp::Node::make_shared("whycon");
	posePub = node->create_publisher<geometry_msgs::msg::PoseArray>("whycon/markers",10);
	auto imageSub = node->create_subscription<sensor_msgs::msg::Image>(args[1],rclcpp::SensorDataQoS(),imageCallback);
	auto calibSrv = node->create_service<std_srvs::srv::Trigger>("whycon/autocalibrate",autocalibrateCallback);
	rclcpp::spin(node);

	calibSrv.reset();
	imageSub.reset();
	posePub.reset();
	node.reset();
	rclcpp::shutdown();

	if (robotPositionLog != NULL) fclose(robotPositionLog);
	delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}
