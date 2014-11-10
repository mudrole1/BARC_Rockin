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
#include <fstream>
#include <string>

static int N_SAMPLES, currentSample;
//static int delimanRedL, delimanRedH, delimanGreenL, delimanGreenH, delimanBlueL,
//		delimanBlueH;
static int deliman[6], postman[6];
//static int postmanRedL, postmanRedH, postmanGreenL, postmanGreenH, postmanBlueL,
//		postmanBlueH;
static int threshold;
static ros::Publisher detectedPerson;
static bool startDetecting;
static bool initFlag = true;
static std::string calibFile;

void init(ros::NodeHandle n) {

	n.param<std::string>("/color_detector/calibFile", calibFile, "");
	ROS_INFO("Got param: %s", calibFile.c_str());
	std::ifstream inFile(calibFile.c_str());
	std::string lineIn;
	while (std::getline(inFile, lineIn)) {
		ROS_INFO("READ LINE: %s", lineIn.c_str());
		if (std::strcmp(lineIn.c_str(), "Deliman") == 0) {
			std::getline(inFile, lineIn);
			int i = 0;
			while (std::strcmp(lineIn.c_str(), "Stop")) {
				ROS_INFO("READ LINE: %s", lineIn.c_str());
				deliman[i] = atoi(lineIn.c_str());
				i++;
				std::getline(inFile, lineIn);
			}
		} else {
			std::getline(inFile, lineIn);
			int i = 0;
			while (std::strcmp(lineIn.c_str(), "Stop")) {
				postman[i] = atoi(lineIn.c_str());
				i++;
				std::getline(inFile, lineIn);
			}
		}
	}
	inFile.close();
//	delimanRedH = deliman[0];
//	delimanGreenH = deliman[1];
//	delimanBlueH = deliman[2];
//	delimanRedL = deliman[3];
//	delimanGreenL = deliman[4];
//	delimanBlueL = deliman[5];

//	postmanRedL = 100;
//	postmanGreenL = 100;
//	postmanBlueL = 100;
//	postmanRedH = 100;
//	postmanGreenH = 100;
//	postmanBlueH = 100;

	threshold = 1.8e+07;

	detectedPerson = n.advertise<std_msgs::String>("/recognition/response", 1);
	startDetecting = false;
	N_SAMPLES = 10;
	currentSample = 0;
}

void cameraInit(ros::NodeHandle n) {
	init(n);
	cv::namedWindow("Calibrate Deliman", CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("LowR", "Calibrate Deliman", &deliman[2], 255); //Hue (0 - 179)
	cv::createTrackbar("HighR", "Calibrate Deliman", &deliman[5], 255);

	cv::createTrackbar("LowG", "Calibrate Deliman", &deliman[1], 255); //Saturation (0 - 255)
	cv::createTrackbar("HighG", "Calibrate Deliman", &deliman[4], 255);

	cv::createTrackbar("LowB", "Calibrate Deliman", &deliman[0], 255); //Value (0 - 255)
	cv::createTrackbar("HighB", "Calibrate Deliman", &deliman[3], 255);

	cv::namedWindow("Calibrate Postman", CV_WINDOW_AUTOSIZE);
	cv::createTrackbar("LowR", "Calibrate Postman", &postman[2], 255); //Hue (0 - 179)
	cv::createTrackbar("HighR", "Calibrate Postman", &postman[5], 255);

	cv::createTrackbar("LowG", "Calibrate Postman", &postman[1], 255); //Saturation (0 - 255)
	cv::createTrackbar("HighG", "Calibrate Postman", &postman[4], 255);

	cv::createTrackbar("LowB", "Calibrate Postman", &postman[0], 255); //Value (0 - 255)
	cv::createTrackbar("HighB", "Calibrate Postman", &postman[1], 255);

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

		/******************** DELIMAN *******************************/
		cv::Mat thresholdedImg;
//		cv::inRange(cv_ptr->image,
//				cv::Scalar(delimanBlueL, delimanGreenL, delimanRedL),
//				cv::Scalar(delimanBlueH, delimanGreenH, delimanRedH),
//				thresholdedImg);
		cv::inRange(cv_ptr->image,
				cv::Scalar(deliman[0], deliman[1], deliman[2]),
				cv::Scalar(deliman[3], deliman[4], deliman[5]), thresholdedImg);
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
		cv::imshow("Calibrate Deliman", thresholdedImg);

		/******************** POSTMAN *******************************/
		cv::inRange(cv_ptr->image,
				cv::Scalar(postman[0], postman[1], postman[2]),
				cv::Scalar(postman[3], postman[4], postman[5]), thresholdedImg);
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
		cv::imshow("Calibrate Postman", thresholdedImg);

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
				cv::Scalar(deliman[0], deliman[1], deliman[2]),
				cv::Scalar(deliman[3], deliman[4], deliman[5]), thresholdedImg);
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

		/******************* POSTMAN ********************************************/
		cv::inRange(cv_ptr->image,
				cv::Scalar(postman[0], postman[1], postman[2]),
				cv::Scalar(postman[3], postman[4], postman[5]), thresholdedImg);
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
			sleep(2);
		}

	}
}

void start(const std_msgs::EmptyConstPtr& empty) {
	startDetecting = true;
	initFlag = false;
	if (std::strcmp(calibFile.c_str(), "") != 0) {
		std::ofstream outFile(calibFile.c_str());
		outFile << "Deliman\n";
		int i;
		for (i = 0; i < 6; i++) {
			outFile << deliman[i] << "\n";
		}
		outFile << "Stop\n";
		outFile << "Postman\n";
		for (i = 0; i < 6; i++) {
			outFile << postman[i] << "\n";
		}
		outFile <<"Stop\n";
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "color_detector");
	ros::NodeHandle n;
	cameraInit(n);
	ros::Subscriber node_sub = n.subscribe("/usb_cam/image_raw", 1, detect);
	ros::Subscriber startDetect = n.subscribe("/color_detect/start", 1, start);
	ros::spin();
	return 0;
}

