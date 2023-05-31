#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <cmath>
#include <string>
#include <vector> 
#include <algorithm>
#include <chrono>
#include <numeric>
#include <iostream>
#include <fstream>
#include <cmath>
#include <tuple>
#include "ros/time.h"	

// image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// message
#include "risk_assessment/risk.h"
#include "sha/sha.h"
#include "radiation/sievert.h"
#include "situational_semantics_richness/situational_semantics_richness.h"
#include "noise_detection/noise.h"

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher ssr_pub;
image_transport::Publisher scoreImgpub; 
situational_semantics_richness::situational_semantics_richness SSRscore;
std::ofstream datawrite;

using namespace message_filters;
using namespace std;
using namespace sha;
using namespace radiation;
using namespace noise_detection;
using namespace risk_assessment;




float sv;
float shascore;
float ssr;
float riskscore;
std::vector<double> weight = {1,1,1};

int lowscore_cnt = 0;
int highscore_cnt = 0;
int zero_cnt = 0;
const int ARRAY_SIZE = 5;




std::vector<double> softmax(const std::vector<double>& inputs) 
{
    std::vector<double> softmaxresult;
    double sumExp = 0.0;

    // Compute exponentials and sum of exponentials
    for (const auto& input : inputs) {
        double expVal = std::exp(-(input - std::pow(input, 2)) / 2.0);
        softmaxresult.push_back(expVal);
        sumExp += expVal;
    }
    // Normalize by dividing by the sum of exponentials
    for (auto& softmaxresults : softmaxresult) {
        softmaxresults /= sumExp;
    }

    return softmaxresult;
}

double dotProduct(const std::vector<double>& v1, const std::vector<double>& v2) {
    // Check if the vectors have the same size
    if (v1.size() != v2.size()) {
        std::cerr << "Error: Vectors must have the same size." << std::endl;
        return 0.0;  // Return an error value
    }

    double result = 0.0;

    // Calculate the dot product
    for (size_t i = 0; i < v1.size(); ++i) {
        result += v1[i] * v2[i];
    }

    return result;
}

double attention(double newSSR)
{

    std::vector<double> SSRArray(ARRAY_SIZE, 0.0);  // Initialize array with zeros
	std::vector<double> softmaxArray(ARRAY_SIZE, 0.0);
	std::vector<double> time = {1,2,3,4,5};
    int currentIndex = 0;  // Tracks the current index to update
	double SSR_hat;
    // Add numbers to the array until the user inputs a negative number

	
	if (SSRArray.size() < ARRAY_SIZE)
	{
		SSRArray.push_back(newSSR);

	}
	else
	{
		SSRArray.erase(SSRArray.begin());
		SSRArray.push_back(newSSR);
	}
	
	SSR_hat = dotProduct(softmax(time), SSRArray);
	// ROS_INFO("sv:%.3f", SSR_hat);

	return SSR_hat; 

}
void processcallback(const radiation::sievertConstPtr msg1, const risk_assessment::riskConstPtr msg2, const noise_detection::noiseConstPtr msg3, const geometry_msgs::PoseWithCovarianceStampedConstPtr amcl_pose_msg)
{ 	
	std::vector<std::array<double, ARRAY_SIZE>> matrix;

    std::vector<double> ssrVector;
	// std::array<double, ARRAY_SIZE> riskArray;
    // std::array<double, ARRAY_SIZE> SHAArray;
    // std::array<double, ARRAY_SIZE> radiationArray;
	// std::array<double, ARRAY_SIZE> noiseArray;

	datawrite.open ("data.csv",std::ios_base::app);

	double robot_x = amcl_pose_msg->pose.pose.position.x;
	double robot_y = amcl_pose_msg->pose.pose.position.y;
	double robot_z = amcl_pose_msg->pose.pose.position.z;



	int SV_timestamp = msg1->header.stamp.toSec();
	int riskscore_timestamp = msg2->header.stamp.toSec();
	int lasercore_timestamp = msg3->header.stamp.toSec();


	float SV = msg1->sv;
	float riskscore = msg2->riskScore;
	float lasercore = msg3->score;


	ssrVector.push_back(SV);
	ssrVector.push_back(riskscore);
	ssrVector.push_back(lasercore);
	
	double ssr = dotProduct(weight, ssrVector);
	double ssr_hat = attention(ssr);
	
	
	
	// ROS_INFO("ssr:%.3f", ssr);
	ROS_INFO("ssr_hat:%.3f  X:%.2f Y:%.2f  Z:%.2f ", ssr_hat, robot_x, robot_y, robot_z);


	SSRscore.ssr = ssr_hat;
		
	// set the message timestamp
	SSRscore.header.stamp = ros::Time::now();

	ssr_pub.publish(SSRscore);
		
	
	datawrite << ros::Time::now() <<SV<<"," <<riskscore<<","<<lasercore<<","<<ssr_hat<<","<<robot_x<<","<<robot_y<<","<<robot_z<<'\n';


	datawrite.close();

	if (ssr_hat > 0.8)
		{
			highscore_cnt ++;
			
			if (highscore_cnt>5)
			{	
				lowscore_cnt = 0;
	
				std::string victimwarning = "Situational semantics richness warning! SSR score:" + std::to_string(float(SSRscore.ssr)) ;
				cv::Mat image(400, 1000, CV_8UC3, cv::Scalar(0));
				cv::putText(image, //target image
            	victimwarning, //text
            	cv::Point(10, image.rows / 2), //MIDDLE position
            	cv::FONT_HERSHEY_DUPLEX,
            	1.0, //size
            	CV_RGB(255, 255, 0), //font color
            	1 //thick ness
            	);
				
				sensor_msgs::ImagePtr scoreImgmsg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
				scoreImgpub.publish(scoreImgmsg);
				
			}
		}
			
		
		else
		{
			lowscore_cnt ++;
			
			
			if (lowscore_cnt > 5)
			{
				highscore_cnt = 0;
				
				std::string scoreString = "SSR score: " + std::to_string(float(SSRscore.ssr));
    			cv::Mat image(100, 300, CV_8UC3, cv::Scalar(0));
				cv::putText(image, //target image
        		scoreString, //textold_framescore
        		cv::Point(10, image.rows / 2), //MIDDLE position
        		cv::FONT_HERSHEY_DUPLEX,
            		1.0, //size
            	CV_RGB(255, 0, 0), //font color
            	1 //thick ness
            	);
				sensor_msgs::ImagePtr scoreImgmsg=cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
		
				scoreImgpub.publish(scoreImgmsg);
			}
		}
	

}




int main(int argc, char *argv[])
{


	datawrite.open ("data.csv");
	// std::cout<< "write to data.csv" << std::endl;
	datawrite << "Timestamp" <<","<<"Radiation"<<","<<"Risk"<<"," <<"Noise"<<","<<"SSR estimation"<<","<<"X"<<","<<"Y"<<","<<"Z"<<'\n';
	datawrite.close();

	// Init ROS
	ros::init(argc, argv, "ssr");
	// ROS_INFO("Start evaluating SSR, waiting for detections result...");
	ros::NodeHandle nh;


	message_filters::Subscriber<radiation::sievert> sv_sub(nh, "/radiation/sievert", 1);
	message_filters::Subscriber<risk_assessment::risk> risk_sub(nh, "/risk/score", 1);
	message_filters::Subscriber<noise_detection::noise> laser_sub(nh, "/noise_evaluation", 1);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_pose_sub(nh, "/amcl_pose", 1);
	typedef message_filters::sync_policies::ApproximateTime<radiation::sievert, risk_assessment::risk, noise_detection::noise, geometry_msgs::PoseWithCovarianceStamped> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sv_sub, risk_sub, laser_sub, amcl_pose_sub);
	sync.registerCallback(boost::bind(&processcallback, _1, _2, _3, _4));
		


	// ros::Publisher ssr_pub = nh.advertise<situational_semantics_richness::situational_semantics_richness>("SSR/score/", 10);
	ssr_pub = nh.advertise<situational_semantics_richness::situational_semantics_richness>("/SSR/score", 10);
	// ssr_pub.publish(ssr);
		
		
	// Score GUI
	image_transport::ImageTransport imgt(nh);

	scoreImgpub = imgt.advertise("/SSR/score_image",1);

	ros::spin();
	return 0;


}
