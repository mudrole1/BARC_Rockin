#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
using namespace cv; 
using namespace std;

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, char separator = ';') {
    std::ifstream file(filename.c_str(), ifstream::in);
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line, path, classlabel;
    while (getline(file, line)) {
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}

void PrintError( int ErrorID );
void ImageCallback( const sensor_msgs::Image::ConstPtr& msg );
void WakeUpCallback( const std_msgs::String::ConstPtr& msg );


const int N_SAMPLES = 5; // Number of recognizing loops
const int ERROR_ARG = 1;
const string DATASET_FOLDER_NAME = "face_dataset";
const int N_ARGS = 2; //<PathCascadeFILE> <PathCSVFILE>
const int THRESHOLD_CUT = 100;
int RecLoopCount; // The count of the recognizing loops performed...
string PathCascade;
string PathCSV;
CascadeClassifier haar_cascade; // Face detection to be built
Ptr<FaceRecognizer> model; // Face recognizer (classifier to e built)
int im_width;
int im_height;
bool IsThereAFace;
string MsgToPublish;
bool RunFaceRecognition;


int main( int argc, char **argv ){
  
  // Wrong arguments error
  cout << "NARGS = " << argc << endl;
  if( argc != ( N_ARGS + 1 ) ){
     PrintError( ERROR_ARG );
    return -1;
  }
  // Processing arguments
  PathCascade = string( *( argv + 1 ) );
  PathCSV = string( *( argv + 2 ) );
  // ARGUMENT VERIFICATION NEEDED HERE

 
  vector<Mat> images; //|-> These vectors hold the images and corresponding labels
  vector<int> labels; //|
  // Read CSV file
  try {
    read_csv(PathCSV, images, labels);
  } catch (cv::Exception& e) {
    cerr << "Error opening file \"" << PathCSV<< "\". Reason: " << e.msg << endl;
    exit(1);
  }

  // Size of imgs
  im_width = images[0].cols; 
  im_height = images[0].rows;

  // Create a FaceRecognizer and train it with the dataset
  // model = createFisherFaceRecognizer();
  model = createLBPHFaceRecognizer();
  model->train(images, labels);
  
  // Building the facedetector with the specified Haar Cascade Filter
  haar_cascade.load( PathCascade );
  
  
  // > > >  R O S    I N I T  < < <

  // Init ros node
  ros::init( argc, argv, "face_recognizer_node" );
  ros::NodeHandle n;  
  ros::Publisher  node_pub = n.advertise <std_msgs::String>("face_recognizer_ID", 2);
  ros::Subscriber node_sub = n.subscribe( "/camera/image_raw", 2, ImageCallback );
  ros::Subscriber node_sub_WakeUp = n.subscribe( "/face_rec_wakeup", 2, WakeUpCallback );

  RecLoopCount = 0;
  RunFaceRecognition = false;
  IsThereAFace = false;

  std_msgs::String msg;

  while( ros::ok() ){
    if( RecLoopCount > N_SAMPLES )
      ros::shutdown();
    if( IsThereAFace == true  && RunFaceRecognition == true ){
      //msg.data = MsgToPublish;
      msg.data = "DOCTOR";
      node_pub.publish( msg );
      RunFaceRecognition = false;
    }
    ros::spinOnce();
  }

  return 0;
}


void WakeUpCallback( const std_msgs::String::ConstPtr& msg ){
  if( ( msg->data ).compare( "DoFaceRecognition" ) == 0 )
    RunFaceRecognition = true;
}


void ImageCallback( const sensor_msgs::Image::ConstPtr& msg ){
  
  IsThereAFace = false;
  
  if( RunFaceRecognition == true ){ // Program has been woke up
    // ROS to OPENCV data structure conversion
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Create & load cascade classifier
    bool tryflip = false;
    CascadeClassifier cascade, nestedCascade;
    double scale = 1;
    
    //Get Frame & convert it to grayscale
    Mat frame = cv_ptr->image; 
    Mat original = frame.clone();
    Mat gray;
    cvtColor(original, gray, CV_BGR2GRAY); // convert frame to grayscale
    
    // Run face detection (Haar Cascade Filter) & save faces in a vector
    vector< Rect_<int> > faces;
    haar_cascade.detectMultiScale(gray, faces);
    
    // Classification & labeling of faces
    double confidence, BestHuman_confidence = -1;
    int prediction, BestHuman_prediction = -1;	
    for(int i = 0; i < faces.size(); i++) {
      
      Rect face_i = faces[i];
      // Crop the face from the image. 
      Mat face = gray(face_i);
      
      // Depending on the classifier we're using (e.g. Fischer, or Eigen) we may need resizing
      Mat face_resized;
      cv::resize( face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC );
      
      // Perform the prediction:
      model->predict(face_resized, prediction, confidence);
      
      if( BestHuman_prediction == -1 ){ // Just for the first case, we need new (real) values.
	BestHuman_confidence = confidence;
	BestHuman_prediction = prediction;
      }
      else if( BestHuman_prediction > prediction ){
	BestHuman_confidence = confidence;
	BestHuman_prediction = prediction;
      }
      // cout << prediction << "-" << confidence << " ";
      // if( confidence > 123 )
      //   prediction = 4;
    }
    
    if( BestHuman_confidence != -1 ){ // We have found at least one face to compare.
      IsThereAFace = true;
      if( BestHuman_confidence < THRESHOLD_CUT ){
	MsgToPublish = "DOCTOR";
	// cout << "MARCO :: ";
	// cout << BestHuman_prediction << "-" << BestHuman_confidence << " ";
	// cout << endl;
      }
      else{
	MsgToPublish = "ANY";      
	// cout << "ANY   :: ";
	// cout << BestHuman_prediction << "-" << BestHuman_confidence << " ";
	// cout << endl;
      }
    }
  }
  
}


void PrintError( int ErrorID ){
  switch( ErrorID ){
  case ERROR_ARG:
    cout << "ERROR: You need to run this program with the following arguments:" << endl;
    cout << "    rosrun face_recognizer face_recognizer_node <PathCascadeFILE> <PathCVSFILE>" << endl;
    break;
    cout << "ERROR: Non recognized error ocurred!" << endl;
    break;
  }
}



