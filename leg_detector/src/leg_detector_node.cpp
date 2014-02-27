// FILE: "Leg_detector_node"
// AUTHOR: Marco Antonio Becerra Pedraza (http://www.marcobecerrap.com)
// COLLABORATORS: Shaun Bastable & Manolis Chiou 
// SUMMARY: This program receives the LRF msgs and executes a leg detector algorithm 
// > to search for persons. At the end publishes a a vector with all the persons found 
// > and their position relative to the sensor.
//
// NOTES: This leg detector is based on the work described in:
// Bellotto, N. & Hu, H. 
// Multisensor-Based Human Detection and Tracking for Mobile Service Robots 
// IEEE Trans. on Systems, Man, and Cybernetics -- Part B, 2009, 39, 167-181


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <leg_detector/LegDetectorVector.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>

#define PI 3.1416
#define FILTER_SIZE 2
#define FLANK_THRESHOLD 0.1

#define FLANK_U 1
#define FLANK_D -1

//Antropometric parameters
#define ANTRO_a0 0.1 //|
#define ANTRO_a1 0.2 //|-> Leg width
#define ANTRO_b0 0   //  |
#define ANTRO_b1 0.4 //  |-> Free space between two legs
#define ANTRO_c0 0.1 //    |
#define ANTRO_c1 0.4 //    |-> Two legs together width

// Pattern Type
#define TYPE_LA 1
#define TYPE_FS 2
#define TYPE_SL 3 

// Uncertainties
#define UN_LA 0.99
#define UN_FS 0.95
#define UN_SL 0.90


using namespace std;

bool publish_on  = false;
bool robot_on    = false;

int g_counter = 0;

vector < double > rec_x;
vector < double > rec_y;
vector < double > rec_u;

void LaserFilter_Mean( vector <double> *vector_r, unsigned size );
void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg);
void FindPattern( string str, string pattern, list <int> *element_found );
void ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y);
double Dist2D( double x0, double y0, double x1, double y1 );
void HumanPose( vector <double> *r_x, vector <double> *r_y, vector <double> *r_u, double UN, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y );



int main(int argc, char **argv)
{

  ros::init(argc, argv, "leg_detector_node");

  ros::NodeHandle n;

  ros::Publisher  node_pub = n.advertise <leg_detector::LegDetectorVector>("leg_detector_vector", 2); // Humans in the environment
  ros::Subscriber node_sub = n.subscribe("/scan", 2, LaserCallback);

  leg_detector::LegDetectorVector msg; // robot_vel
  
  while( ros::ok() ){
    if( robot_on == true ){
      msg.x = rec_x;
      msg.y = rec_y;
      msg.uncertainty = rec_u;
      node_pub.publish( msg );
      publish_on = false;
    }
    ros::spinOnce();
  }
  return 0;
}

void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg){
  //sensor_msgs::PointCloud PCLmsg;
  //sensor_msgs::PointCloud *LaserCloud;

  
  // Vectors...
  rec_x.clear(); 
  rec_y.clear(); 
  rec_u.clear(); 
  
  publish_on = true;
  robot_on = true;

  
  double px, py, pr, pt;
  vector < double >  laser_x;
  vector < double >  laser_y;
  vector < double >  laser_r;
  vector < double >  laser_t;
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    pr = msg->ranges[ i ];
    pt = msg->angle_min + ( i * msg->angle_increment);
    laser_r.push_back( pr );
    laser_t.push_back( pt );
  }
  
  // Filtering laser scan
  LaserFilter_Mean( &laser_r, FILTER_SIZE );
  for( unsigned i = 0; i < msg->ranges.size(); i++ ){    
    px = laser_r[ i ] * cos( laser_t[ i ] );
    py = laser_r[ i ] * sin( laser_t[ i ] );
    laser_x.push_back( px );
    laser_y.push_back( py );
  }
	 
  string str_aux = "";
  // Finding flanks in the laser scan...
  vector < int > laser_flank;
  laser_flank.assign(laser_r.size(), 0);
  for( unsigned i = 1; i < laser_flank.size(); i++ ){
    if( fabs( laser_r[ i ] - laser_r[ i - 1 ] ) > FLANK_THRESHOLD ){
      laser_flank[ i ] = ( ( laser_r[ i ] - laser_r[ i - 1 ] ) > 0 ) ? FLANK_U : FLANK_D;
      //cout << ( laser_r[ i ] - laser_r[ i - 1 ] ) << endl;
    }
  }
  
  vector < int >    flank_id0;
  vector < int >    flank_id1;
  string flank_string = "";
  int past_value = 0;
  int idx = 0;
  for( unsigned i = 1; i < laser_flank.size(); i++ ){
    if( laser_flank[ i ] != 0 ){
      if( past_value != laser_flank[ i ] ){
	flank_id0.push_back( i - 1 );
	flank_id1.push_back( i );
	flank_string += ( laser_flank[ i ] > 0 ) ? "S" : "B";
	idx++;
      }
      else
      	flank_id1[ idx - 1 ] =  i;    
    }
    past_value = laser_flank[ i ];
  }
  
  //cout << flank_string << endl;
  //cout << "------------------" << endl;
  
    
  // PATTERN RECOGNITION
  string LEGS_LA  = "BSBS";
  string LEGS_FS1 = "BBS";
  string LEGS_FS2 = "BSS";
  string LEGS_SL = "BS";
  
  list <int> Pattern_LA;
  list <int> Pattern_FS1;
  list <int> Pattern_FS2;
  list <int> Pattern_SL;
 

  FindPattern( flank_string, LEGS_LA,  &Pattern_LA  );
  FindPattern( flank_string, LEGS_FS1, &Pattern_FS1 );
  FindPattern( flank_string, LEGS_FS2, &Pattern_FS2 );
  FindPattern( flank_string, LEGS_SL,  &Pattern_SL  );  


  // ANTROPOMETRIC VALIDATION (the non antropometric patterns are erased from the list)
  ValidatePattern( &Pattern_LA,  TYPE_LA, flank_id0, flank_id1,  laser_x, laser_y);
  ValidatePattern( &Pattern_FS1, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
  ValidatePattern( &Pattern_FS2, TYPE_FS, flank_id0, flank_id1,  laser_x, laser_y);
  ValidatePattern( &Pattern_SL,  TYPE_SL, flank_id0, flank_id1,  laser_x, laser_y);


  //CENTROID PATTERN COMPUTATION & UNCERTAINTY
  rec_x.clear();
  rec_y.clear();
  rec_u.clear();
  
  HumanPose( &rec_x, &rec_y, &rec_u, UN_LA, Pattern_LA,  TYPE_LA,  flank_id0, flank_id1,  laser_x, laser_y);
  HumanPose( &rec_x, &rec_y, &rec_u, UN_FS, Pattern_FS1, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
  HumanPose( &rec_x, &rec_y, &rec_u, UN_FS, Pattern_FS2, TYPE_FS,  flank_id0, flank_id1,  laser_x, laser_y);
  HumanPose( &rec_x, &rec_y, &rec_u, UN_SL, Pattern_SL,  TYPE_SL,  flank_id0, flank_id1,  laser_x, laser_y);

  //cout << "I see " << rec_x.size() << " persons" << endl;
  //cout << "--------------------------" << endl;
}










// Mean value of the 'size' adjacent values...
void LaserFilter_Mean( vector <double> *vector_r, unsigned size ){
  for( unsigned i = 0; i < ( (*vector_r).size() - size ); i++ ){
      double mean = 0;
      for( unsigned k = 0; k < size; k++  ){
	mean += (*vector_r)[ i + k ];
      }
      (*vector_r)[ i ] = mean / size;
  }
}


// Reports a found string pattern in a list...
void FindPattern( string str, string pattern, list <int> *element_found ){
  size_t found = 0;

  while( string::npos != ( found = str.find( pattern, found ) ) ){
    (*element_found).push_back( found ); 
    found++;
  }
  
} 


// Performs the antropometric validation of the leg patterns
void ValidatePattern( list <int> *Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y){
  
  double ANTRO_a_1, ANTRO_a_2, ANTRO_b, ANTRO_c; // Antropometric constansts
  bool SavePattern = true;

  bool cond_a = true, cond_b = true, cond_c = true;

  list<int>::iterator it;
  
  for( it = (*Pattern_list).begin(); it != (*Pattern_list).end(); it++ ){

    // Obtain antropometric constants
    switch( TYPE ){
      case TYPE_LA: //BSBS
	ANTRO_a_1 = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
	ANTRO_a_2 = Dist2D( laser_x[ flank_id1[ *it + 2 ] ], laser_y[ flank_id1[ *it + 2 ] ], laser_x[ flank_id0[ *it + 3 ] ], laser_y[ flank_id0[ *it + 3 ] ]);
	ANTRO_b = Dist2D( laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ], laser_x[ flank_id1[ *it + 2 ] ], laser_y[ flank_id1[ *it + 2 ] ] );
	ANTRO_c = 0;
	cond_a = ( ( ANTRO_a_1 >= ANTRO_a0 ) && ( ANTRO_a_1 <= ANTRO_a1 ) ) && ( ( ANTRO_a_2 >= ANTRO_a0 ) && ( ANTRO_a_2 <= ANTRO_a1 ) );
	cond_b = ( ( ANTRO_b >= ANTRO_b0 ) && ( ANTRO_b <= ANTRO_b1 ) );
	cond_c = true;
        break;
      case TYPE_FS: // BBS & BSS
	ANTRO_a_1 = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
	ANTRO_a_2 = Dist2D( laser_x[ flank_id1[ *it + 1 ] ], laser_y[ flank_id1[ *it + 1 ] ], laser_x[ flank_id0[ *it + 2 ] ], laser_y[ flank_id0[ *it + 2 ] ]);
	ANTRO_b = Dist2D( laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ], laser_x[ flank_id1[ *it + 1 ] ], laser_y[ flank_id1[ *it + 1 ] ] );
	ANTRO_c = 0;
	cond_a = ( ( ANTRO_a_1 >= ANTRO_a0 ) && ( ANTRO_a_1 <= ANTRO_a1 ) ) && ( ( ANTRO_a_2 >= ANTRO_a0 ) && ( ANTRO_a_2 <= ANTRO_a1 ) );
	cond_b = ( ( ANTRO_b >= ANTRO_b0 ) && ( ANTRO_b <= ANTRO_b1 ) );
	cond_c = true;
        break;
    case TYPE_SL: // BS
      	ANTRO_a_1 = 0;
	ANTRO_a_2 = 0;
	ANTRO_b = 0;
	ANTRO_c = Dist2D( laser_x[ flank_id1[ *it ] ], laser_y[ flank_id1[ *it ] ], laser_x[ flank_id0[ *it + 1 ] ], laser_y[ flank_id0[ *it + 1 ] ]);
	cond_a = true;
	cond_b = true;	
	cond_c = ( ( ANTRO_c >= ANTRO_c0 ) && ( ANTRO_c <= ANTRO_c1 ) );
	break;
    }

    SavePattern = cond_a && cond_b && cond_c;
    
    if( !SavePattern ){
      it = (*Pattern_list).erase( it );
      it--;
    }
    
    //cout << "Type = " << TYPE << "]" << endl;
    //cout << "a1[" << ANTRO_a_1 << "]" << endl;
    //cout << "a2[" << ANTRO_a_2 << "]" << endl;
    //cout << "b[" << ANTRO_b << "]" << endl;
    //cout << "c[" << ANTRO_c << "]" << endl;
  }  
}

// Euclidean distance between two coordinate points
double Dist2D( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}


void HumanPose( vector <double> *r_x, vector <double> *r_y, vector <double> *r_u, double UN, list <int> Pattern_list, int TYPE,  vector <int> flank_id0,  vector <int> flank_id1, vector <double> laser_x, vector <double> laser_y ){
  
  double c_x, c_y, c_u;

  int l1, l2, l3, l4;
  int count; 

  list<int>::iterator it;

  for( it = Pattern_list.begin(); it != Pattern_list.end(); it++ ){
    c_x = 0;
    c_y = 0;
    c_u = 0;
    count = 0;

    l1 = flank_id1[ *it ];
    l2 = flank_id0[ *it + 1 ];
    
    switch( TYPE ){
    case TYPE_LA:
      l3 = flank_id1[ *it + 2 ];
      l4 = flank_id0[ *it + 3 ];
      c_u = UN_LA;
      break;
    case TYPE_FS:
      l3 = flank_id1[ *it + 1 ];
      l4 = flank_id0[ *it + 2 ];
      c_u = UN_FS;
      break;
    case TYPE_SL:
      l3 = 1;
      l4 = 0;
      c_u = UN_SL;
      break;
    }

    for( int i = l1; i <= l2; i++ ){
      c_x += laser_x[ i ];
      c_y += laser_y[ i ];
      count++;
    }
    for( int i = l3; i <= l4; i++ ){
      c_x += laser_x[ i ];
      c_y += laser_y[ i ];
      count++;
    }
    
    c_x /= (double) count;
    c_y /= (double) count;
    
    (*r_x).push_back( c_x );
    (*r_y).push_back( c_y );
    (*r_u).push_back( c_u );

  }
  
}
