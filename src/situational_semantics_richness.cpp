#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <cmath>
#include <string>
#include <vector> 
#include <algorithm>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <numeric>
#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/time.h"	

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sha/sha.h"
#include "radiation/sievert.h"
#include "situational_semantics_richness/situational_semantics_richness.h"

ros::Publisher ssr_pub;
image_transport::Publisher scoreImgpub; 
situational_semantics_richness::situational_semantics_richness SSRscore;


using namespace message_filters;
using namespace std;
using namespace sha;
using namespace radiation;


float sv;
float shascore;
float ssr;
std::vector<double> weight = {1,1};

int lowscore_cnt = 0;
int highscore_cnt = 0;
int zero_cnt = 0;
const int ARRAY_SIZE = 5;



// void expert_weight(const std::vector<double>& inputs)
// {
	

// }


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

double dotProduct(const std::vector<double>& v1, const std::vector<double>& v2) 
{
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

	return SSR_hat; 

}

void processcallback(const radiation::sievertConstPtr msg1, const sha::shaConstPtr msg2)
{ 
    // ros::Time msg_time = msg1->header.stamp;
	
	std::vector<std::array<double, ARRAY_SIZE>> matrix;

    std::vector<double> ssrVector;
	std::array<double, ARRAY_SIZE> riskArray;
    std::array<double, ARRAY_SIZE> SHAArray;
    std::array<double, ARRAY_SIZE> radiationArray;
	std::array<double, ARRAY_SIZE> noiseArray;


	int SV_timestamp = msg1->header.stamp.toSec();
	int shascore_timestamp = msg2->header.stamp.toSec();


	float SV = msg1->sv;
	float shascore = msg2->SHAScore;
	
	// build ssr vector
	ssrVector.push_back(SV);
	ssrVector.push_back(shascore);
	
	// process ssr and ssr_hat
	double ssr = dotProduct(weight, ssrVector);
	double ssr_hat = attention(ssr);
	

	ROS_INFO("ssr:%.3f", ssr);
	ROS_INFO("ssr_hat:%.3f", ssr_hat);

	SSRscore.ssr = ssr_hat;
		
		// set the message timestamp
	SSRscore.header.stamp = ros::Time::now();

	ssr_pub.publish(SSRscore);

	
	if (ssr > 0.9)
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
    			cv::Mat image(400, 1000, CV_8UC3, cv::Scalar(0));
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


	// Init ROS
	ros::init(argc, argv, "ssr");
	// ROS_INFO("Start evaluating SSR, waiting for detections result...");
	ros::NodeHandle nh;
	

	message_filters::Subscriber<radiation::sievert> sv_sub(nh, "/radiation/sievert", 1);
	message_filters::Subscriber<sha::sha> sha_sub(nh, "/SHA/score/", 1);
	typedef message_filters::sync_policies::ApproximateTime<radiation::sievert, sha::sha> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sv_sub, sha_sub);
    sync.registerCallback(boost::bind(&processcallback, _1, _2));
	


	// ros::Publisher ssr_pub = nh.advertise<situational_semantics_richness::situational_semantics_richness>("SSR/score/", 10);
	ssr_pub = nh.advertise<situational_semantics_richness::situational_semantics_richness>("/SSR/score/", 10);
	// ssr_pub.publish(ssr);
	
	
	// Score GUI
	image_transport::ImageTransport imgt(nh);
	scoreImgpub = imgt.advertise("/SSR/score_image",1);
	

	ros::spin();
	return 0;


}
