#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <ctime>
using namespace cv; 
using namespace std;

void PrintError( int ErrorID );
void ImageCallback( const sensor_msgs::Image::ConstPtr& msg );
void FaceDetectAndDraw( Mat& img, Mat& face, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, double scale, bool tryflip );

const int N_SAMPLES = 5;
const int ERROR_ARG = 1;
const int ERROR_MANYHUMANS = 2;
const string DATASET_FOLDER_NAME = "face_dataset";
const int N_ARGS = 5; //<PathCascadeNameFILE> <PathNestedCascadeNameFILE> <PathToDatasetFOLDER> <PathCVSFILE> <#Human>

string PathCascadeName;
string PathNestedCascadeName;
string PathDataset;
string PathCSV;
int    NoHuman;

bool IsThereAFace;

int FileCount;


int main( int argc, char **argv ){
  
  // Wrong arguments error
  if( argc != ( N_ARGS + 1 ) ){
     PrintError( ERROR_ARG );
    return -1;
  }

  // Processing arguments
  PathCascadeName = string( *( argv + 1 ) );
  PathNestedCascadeName = string( *( argv + 2 ) );
  PathDataset = string( *( argv + 3 ) );
  PathCSV = string( *( argv + 4 ) );
  NoHuman = atoi( *( argv + 5 ) );


  // ARGUMENT VERIFICATION NEEDED HERE
  // ----------------------------------------
  if( ( NoHuman < 1 ) || ( NoHuman > 99 ) ){
    PrintError( ERROR_MANYHUMANS );
    return -1;
  }
  if( NoHuman == 1 ) //If first human, we erase the previous CSV file
    system( ( "rm -rf " + PathCSV ).c_str() );

  // Building name of folder for human
  string FolderHuman = "";
  if( NoHuman < 10 )
    FolderHuman = "0";
  stringstream ss_aux;
  ss_aux << NoHuman;
  FolderHuman = "s" + FolderHuman + ss_aux.str();
  if( PathDataset.at( PathDataset.size() - 1 ) != '/' )
    PathDataset.push_back( '/' );
  PathDataset = PathDataset + FolderHuman;
  
  
  // Deleting previous overlapping folder and creating new one
  system( ( "rm -rf " + PathDataset ).c_str() );
  system( ( "mkdir -p " + PathDataset ).c_str() );  
  
  // > > >  R O S    I N I T  < < <

  // Init ros node
  ros::init( argc, argv, "face_add_node" );
  ros::NodeHandle n;
  
  ros:: Subscriber node_sub = n.subscribe( "/camera/image_raw", 2, ImageCallback );

  while( ros::ok() ){
    if( FileCount > N_SAMPLES )
      ros::shutdown();
    ros::spinOnce();
  }
  return 0;
}





void ImageCallback( const sensor_msgs::Image::ConstPtr& msg ){
  
  // ROS to OPENCV data structure conversion
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }  
  // The OpenCV image is in cv_ptr


  // Create & load cascade classifier
  bool tryflip = false;
  CascadeClassifier cascade, nestedCascade;
  double scale = 1;


  Mat InputFrame, OutputFace;
  if( !cascade.load( PathCascadeName ) )
    cerr << "ERROR: Could not load classifier cascade" << endl;
  else if( !nestedCascade.load( PathNestedCascadeName ) )
    cerr << "ERROR: Could not load eye classifier cascade" << endl;
  
  else{
    // Get Image
    InputFrame = cv_ptr->image;  
    //cout << "R[" << InputFrame.rows << "] C[" << InputFrame.cols << "] === ";
    
    if( InputFrame.rows != 0 && InputFrame.cols != 0 ){
    
      // Apply face detector
      FaceDetectAndDraw( InputFrame, OutputFace, cascade, nestedCascade, scale, tryflip );

      if( IsThereAFace == true ){
	// Saving images in files...
	FileCount++;
	stringstream FileName;
	FileName << PathDataset + "/img" << FileCount << ".png";
	imwrite( FileName.str(), OutputFace );

	// Writing CSV file
	FileName.str( "" ); // Erasing this variable
	FileName << "echo \"" << PathDataset << "/img" << FileCount << ".png;" << ( NoHuman - 1 ) << "\" >> " << PathCSV;
	//cout << FileName.str() << endl;
	system( ( FileName.str() ).c_str() );

	
	// if( FileCount >= N_SAMPLES )
	//   ros::shutdown();
	
	// Wait 0.25 sec
	//ros::Duration( 0.25 ).sleep(); 
      }
    }
  }
}


void PrintError( int ErrorID ){
  switch( ErrorID ){
  case ERROR_ARG:
    cout << "ERROR: You need to run this program with the following arguments:" << endl;
    cout << "    rosrun face_add face_add_node <PathCascadeNameFILE> <PathNestedCascadeNameFILE> <PathToDatasetFOLDER> <PathCVSFILE> <#Human>" << endl;
    break;
  case ERROR_MANYHUMANS:
    cout << "ERROR: Too many humans on the dataset; only can handle [1, 99]." << endl;
    break;
  default:
    cout << "Error Ocurred!" << endl;
    break;
  }
}


void FaceDetectAndDraw( Mat& img, Mat& face, CascadeClassifier& cascade, CascadeClassifier& nestedCascade, double scale, bool tryflip ){
  IsThereAFace = false;
  int i = 0;
  double t = 0;
  vector<Rect> faces, faces2;
  const static Scalar colors[] =  { CV_RGB(0,0,255),
				    CV_RGB(0,128,255),
				    CV_RGB(0,255,255),
				    CV_RGB(0,255,0),
				    CV_RGB(255,128,0),
				    CV_RGB(255,255,0),
				    CV_RGB(255,0,0),
				    CV_RGB(255,0,255) };
  Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );


  cvtColor( img, gray, CV_BGR2GRAY );
  resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
  equalizeHist( smallImg, smallImg );

  t = (double)cvGetTickCount();
  cascade.detectMultiScale( smallImg, faces,
			    1.1, 2, 0
			    //|CV_HAAR_FIND_BIGGEST_OBJECT
			    //|CV_HAAR_DO_ROUGH_SEARCH
			    |CV_HAAR_SCALE_IMAGE
			    ,
			    Size(30, 30) );
  if( tryflip ){
    flip(smallImg, smallImg, 1);
    cascade.detectMultiScale( smallImg, faces2,
			      1.1, 2, 0
			      //|CV_HAAR_FIND_BIGGEST_OBJECT
			      //|CV_HAAR_DO_ROUGH_SEARCH
			      |CV_HAAR_SCALE_IMAGE
			      ,
			      Size(30, 30) );
    for( vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); r++ ){
      faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
    }
  }
  t = (double)cvGetTickCount() - t;
  printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );


  //=======================
  if( faces.size() > 0 )
    IsThereAFace = true;
  //=======================

  for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ ){
    Mat smallImgROI;
    vector<Rect> nestedObjects;
    Point center;
    Scalar color = colors[i%8];
    int radius;

    double aspect_ratio = (double)r->width/r->height;
    if( 0.75 < aspect_ratio && aspect_ratio < 1.3 ){
      // center.x = cvRound((r->x + r->width*0.5)*scale);
      // center.y = cvRound((r->y + r->height*0.5)*scale);
      // radius = cvRound((r->width + r->height)*0.25*scale);
      // circle( img, center, radius, color, 3, 8, 0 );

      // Copy face rectangle
      Mat M_aux ( img, Rect( r->x, r->y, r->width, r->height ) );
      // Resize the image (300x300 px)
      resize( M_aux, M_aux, Size(300, 300) );
      
      face = M_aux.clone();

    }
    else
      rectangle( img, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
		 cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
		 color, 3, 8, 0);
    if( nestedCascade.empty() )
      continue;
    

  }
}
