/*
 * color_detector.cpp
 *
 *  Created on: 31 Oct 2014
 *      Author: joshua
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>

static int N_SAMPLES, currentSample;
static int delimanRedL, delimanRedH, delimanGreenL, delimanGreenH, delimanBlueL,
		delimanBlueH;
static int postmanRedL, postmanRedH, postmanGreenL, postmanGreenH, postmanBlueL,
		postmanBlueH;
static int threshold;
static ros::Publisher detectedPerson;
static bool startDetecting;
static bool initFlag = true;




void init(ros::NodeHandle n) {
	delimanRedH = 136;
	delimanGreenH = 166;
	delimanBlueH = 167;
	delimanRedL = 51;
	delimanGreenL = 87;
	delimanBlueL = 80;

	postmanRedL = 100;
	postmanGreenL = 100;
	postmanBlueL = 100;
	postmanRedH = 100;
	postmanGreenH = 100;
	postmanBlueH = 100;

	threshold=1.8e+07;

	detectedPerson = n.advertise<std_msgs::String>("/recognition/response", 1);
	startDetecting = false;
	N_SAMPLES = 10;
	currentSample = 0;
	cv::namedWindow("deliman");
	cv::namedWindow("normal");
}

void cameraInit(ros::NodeHandle n) {
	init(n);
	cv::namedWindow("Calibrate", CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("LowR", "Calibrate", &delimanRedL, 255); //Hue (0 - 179)
	cv::createTrackbar("HighR", "Calibrate", &delimanRedH, 255);

	cv::createTrackbar("LowG", "Calibrate", &delimanGreenL, 255); //Saturation (0 - 255)
	cv::createTrackbar("HighG", "Calibrate", &delimanGreenH, 255);

	cv::createTrackbar("LowB", "Calibrate", &delimanBlueL, 255); //Value (0 - 255)
	cv::createTrackbar("HighB", "Calibrate", &delimanBlueH, 255);

}

void detect(const sensor_msgs::ImageConstPtr& img) {
	if (initFlag) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(img,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::Mat thresholdedImg;
		cv::inRange(cv_ptr->image,
				cv::Scalar(delimanBlueL, delimanGreenL, delimanRedL),
				cv::Scalar(delimanBlueH, delimanGreenH, delimanRedH),
				thresholdedImg);
		//morphological opening (remove small objects from the foreground)
		cv::erode(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		cv::dilate(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::imshow("Calibrate", thresholdedImg);
		cv::waitKey(1);
	}
	if (startDetecting) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(img,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		std::cout << "go\n";
		flush(std::cout);
		cv::Mat thresholdedImg;
		cv::inRange(cv_ptr->image,
				cv::Scalar(delimanBlueL, delimanGreenL, delimanRedL),
				cv::Scalar(delimanBlueH, delimanGreenH, delimanRedH),
				thresholdedImg);
		//morphological opening (remove small objects from the foreground)
		cv::erode(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		cv::dilate(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::imshow("normal", cv_ptr->image);
		cv::imshow("deliman", thresholdedImg);
		cv::waitKey(1000);
		cv::Moments moment = cv::moments(thresholdedImg);
		double delimanArea = moment.m00;
		std::cout << delimanArea << " deliman area\n";
		flush(std::cout);
		if (delimanArea > threshold) {
			std_msgs::String person;
			person.data = "Deliman";
			detectedPerson.publish(person);
			startDetecting = false;
			currentSample = 0;
			return;
		}

		cv::inRange(cv_ptr->image,
				cv::Scalar(postmanBlueL, postmanGreenL, postmanRedL),
				cv::Scalar(postmanBlueH, postmanGreenH, postmanRedH),
				thresholdedImg);
		//morphological opening (remove small objects from the foreground)
		cv::erode(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		cv::dilate(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(thresholdedImg, thresholdedImg,
				cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

		moment = cv::moments(thresholdedImg);
		double postmanArea = moment.m00;

		if (postmanArea > threshold) {
			std_msgs::String person;
			person.data = "Postman";
			detectedPerson.publish(person);
			startDetecting = false;
			currentSample = 0;
			return;
		}
		if (currentSample >= N_SAMPLES) {
			std_msgs::String person;
			person.data = "Unknown";
			detectedPerson.publish(person);
			currentSample = 0;
			startDetecting = false;
		} else {
			currentSample++;
		}

	}
}

void start(const std_msgs::EmptyConstPtr& empty) {
	startDetecting = true;
	initFlag = false;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "color_detector");
	ros::NodeHandle n;
	cameraInit(n);
	ros::Subscriber node_sub = n.subscribe("/camera/image_raw", 1, detect);
	ros::Subscriber startDetect = n.subscribe("/color_detect/start", 1, start);
	ros::spin();
	return 0;
}

