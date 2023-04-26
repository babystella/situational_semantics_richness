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
#include <tuple>
#include "ros/time.h"	

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "sha/sha.h"
#include "radiation/sievert.h"
#include "situational_semantics_richness/situational_semantics_richness.h"


using namespace message_filters;
using namespace std;

float sv;
float shascore;
float ssr;


int lowscore_cnt = 0;
int highscore_cnt = 0;
int zero_cnt = 0;





//     	// Publish the message
//     ssr_pub.publish(SSRscore);
// 	}message_filters/sync_policies/approximate_time.h

// }





// float processcallback(float a, float b)
// { 
//     float sv = a;
// 	float SHAScore = b;

//     float ssr = SHAScore + sv*50000;
// 	ROS_INFO("ssr score:%.3f", ssr);
// 	return(ssr);
// }




// void processcallback(const std_msgs::Float32::ConstPtr& msg, int topic_num)
// { 
//     ros::Time msg_time = msg->header.stamp;
	
// 	if (topic_num == 1) {
//         float sv = msg->data;
// 		ROS_INFO("sv score:%.3f", sv);
//     } else if (topic_num == 2) {
//         float SHAScore = msg->data;
// 		// ROS_INFO("ssr score:%.3f", SHAScore);
//     }
	

// 	// float sv = msg->data;
// 	// float SHAScore = msg->data;


//     float ssr =  sv*50000 + SHAScore;

// }





void processcallback(const radiation::sievert::ConstPtr& msg1, const sha::sha::ConstPtr& msg2)
{ 
    // ros::Time msg_time = msg1->header.stamp;


	float SV = msg1->sv;
	float shascore = msg2->SHAScore;
	
	ROS_INFO("sv:%.3f", SV);
    std::cout<< "processcallback is running" << std::endl;
	float ssr =  SV*1000 + shascore;

}




// void processcallback(const std_msgs::Float32::ConstPtr& msg1, const std_msgs::Float32::ConstPtr& msg2)
// { 
//     // ros::Time msg_time = msg1->header.stamp;

	

// 	float sv = msg1->data;
// 	float SHAScore = msg2->data;


//     float ssr =  sv*50000 + SHAScore;

	

// }


// void svcallback(const radiation::sievert::ConstPtr& sv_digit)
// { 
//     float sv = sv_digit->sv;

// }

// void shacallback(const sha::sha::ConstPtr& shascore)
// { 
// 	float shaindex = shascore->SHAScore;

// }



int main(int argc, char *argv[])
{


	// Init ROS
	ros::init(argc, argv, "ssr");
	ROS_INFO("Start evaluating SSR, waiting for detections result...");
	ros::NodeHandle nh;
	


	// ros::Subscriber sv_sub = nh.subscribe<std_msgs::Float32>("/sievert", 1, svcallback);
	// ros::Subscriber sha_sub = nh.subscribe<std_msgs::Float32>("/SHA/score", 1, shacallback);


	// ros::Subscriber sv_sub = nh.subscribe<radiation::radiation>("/radiation/sievert", 10, svcallback);
	// ros::Subscriber sha_sub = nh.subscribe<sha::sha>("/SHA/score/", 10, shacallback);

	// ros::Subscriber sv_sub = nh.subscribe<std_msgs::Float32>("/sievert", 10, boost::bind(&processcallback, _1, 1));
	// ros::Subscriber sha_sub = nh.subscribe<std_msgs::Float32>("/SHA/score", 10, boost::bind(&processcallback, _1, 2));
	
	// typedef message_filters::sync_policies::ApproximateTime<std_msgs::Float32, std_msgs::Float32> MySyncPolicy;
	
	typedef sync_policies::ApproximateTime<radiation::sievert, sha::sha> MySyncPolicy;
	
	// message_filters::Subscriber<std_msgs::Float32> sv_sub(nh, "/sievert", 1);
	// message_filters::Subscriber<std_msgs::Float32> sha_sub(nh, "/SHA/score", 1);



	message_filters::Subscriber<radiation::sievert> sv_sub(nh, "/radiation/sievert", 1);
	message_filters::Subscriber<sha::sha> sha_sub(nh, "/SHA/score/", 1);
	
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sv_sub, sha_sub);
    sync.registerCallback(boost::bind(&processcallback, _1, _2));


	// ros::Subscriber sha_sub = nh.subscribe("SSR/sha", 10, boost::bind(&processcallback, _2, &sv_digit));
	
	ros::Publisher ssr_pub = nh.advertise<situational_semantics_richness::situational_semantics_richness>("SSR/score/", 10);


	// Score GUI
	image_transport::ImageTransport imgt(nh);
	image_transport::Publisher scoreImgpub = imgt.advertise("SSR/score_image",1);
	

	//cv::Mat image(100, 300, CV_8UC3, cv::Scalar(0));
	// Start wating for the publisher
	while(ros::ok())
	{
		ros::spinOnce(); 
		// std_msgs::Float32 SSRscore;
		situational_semantics_richness::situational_semantics_richness SSRscore;
		SSRscore.ssr = ssr;
		
		// set the message timestamp
		SSRscore.header.stamp = ros::Time::now();

    	// Publish the message
    	ssr_pub.publish(SSRscore);

		if (ssr > 0.9)
		{
			highscore_cnt ++;
			
			if (highscore_cnt>10)
			{	
				lowscore_cnt = 0;
	
				std::string victimwarning = "Situational semantics richness warning! SSR score:" + std::to_string(float(ssr)) ;
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
			
			
			if (lowscore_cnt > 10)
			{
				highscore_cnt = 0;
				
				std::string scoreString = "SSR score: " + std::to_string(float(ssr));
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

	// End of the program
	// detectionwrite.close();
	// std::cout<< "detection.csv finish" << std::endl;
	return 0;


}
