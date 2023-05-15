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
#include <chrono>


#include <numeric>
#include <iostream>
#include <fstream>
#include <tuple>
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


int lowscore_cnt = 0;
int highscore_cnt = 0;
int zero_cnt = 0;




void processcallback(const radiation::sievertConstPtr msg1, const sha::shaConstPtr msg2)
{ 
    // ros::Time msg_time = msg1->header.stamp;
	ROS_INFO("Processing");
	int SV_timestamp = msg1->header.stamp.toSec();
	int shascore_timestamp = msg2->header.stamp.toSec();

	float SV = msg1->sv;
	float shascore = msg2->SHAScore;
	
    // std::cout<< "processcallback is running" << std::endl;
	float ssr =  SV*1000 + shascore;
	ROS_INFO("sv:%.3f", ssr);
	
	SSRscore.ssr = ssr;
		
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
