// FILE: "Tracker01.cpp"
// AUTHOR: Marco Antonio Becerra Pedraza
// > http://www.marcobecerrap.com
// SUMMARY: This program receives the a list of humans found by different human-sensing 
// > modules makes the tracking & fusion.

#include <ros/ros.h>
#include <follow_sim/Pose.h>
#include <follow_sim/Recognizer.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>

#define TRACK_HISTORY_SIZE 30
#define TRACK_ALPHA 0.05

#define MIN_VAL 0.00001

using namespace std;

bool FIRST_TRACK = true;

bool publish_on  = false;
bool robot_on    = false;

int g_counter = 0;

double track_x = 0;
double track_y = 0;
double track_u = 0;

list < double > history_x;
list < double > history_y;
list < double > history_u;


void RecognizerCallback (const follow_sim::Recognizer::ConstPtr& msg);
double Dist2D( double x0, double y0, double x1, double y1 );
void FilterPU( double h_x, double h_y, double h_u, list <double> *hist_x, list <double> *hist_y, list <double> *hist_u );



int main(int argc, char **argv)
{

  ros::init(argc, argv, "Tracker");

  ros::NodeHandle n;

  ros::Publisher  node_pub = n.advertise <follow_sim::Pose>("track", 2); // Humans recognized
  ros::Subscriber node_sub = n.subscribe("/rec", 2, RecognizerCallback);

  follow_sim::Pose msg; // human_pose
  
  while( ros::ok() ){
    if( robot_on == true ){
      msg.x = track_x;
      msg.y = track_y;
      msg.uncertainty = track_u;
      node_pub.publish( msg );
      publish_on = false;
    }
    ros::spinOnce();
  }
  return 0;
}


void RecognizerCallback (const follow_sim::Recognizer::ConstPtr& msg){
  
  // Callback conclusion
  track_x = 0; 
  track_y = 0; 
  track_u = 0; 

  vector <double> rec_x;
  vector <double> rec_y;
  vector <double> rec_u;
  vector <double> rec_r;
  vector <double> rec_t;

  rec_x = (*msg).x;
  rec_y = (*msg).y;
  rec_u = (*msg).uncertainty;  
  publish_on = true;
  robot_on = true;

  if( FIRST_TRACK == true ){
    history_x.push_back( 0 ); //|-> Choose the closest person to the robot...
    history_y.push_back( 0 ); //|
    history_u.push_back( 1 ); //|
  }

  double distance_human = 10;
  unsigned idx_human;
  for( unsigned i = 0; i < rec_x.size(); i++ ){
    list<double>::iterator it_x;    
    list<double>::iterator it_y;    
    it_x = history_x.end();
    it_y = history_y.end();
    it_x--;
    it_y--;    
    rec_r.push_back( Dist2D( *it_x, *it_y, rec_x[ i ], rec_y[ i ] ) );    
    if( rec_r[ i ] < distance_human ){
      idx_human = i;
      distance_human = rec_r[ i ];
    }
  }

  if( FIRST_TRACK == true ){
    history_x.pop_front();
    history_y.pop_front();
    history_u.pop_front();
    FIRST_TRACK = false;
  }
  
  // Choosed human from the lsr recognizer
  double human_lsr_x;
  double human_lsr_y;
  double human_lsr_u;
  if( rec_x.size() != 0 ){
    human_lsr_x = rec_x[ idx_human ];
    human_lsr_y = rec_y[ idx_human ];
    human_lsr_u = rec_u[ idx_human ];
  }
  else{
    human_lsr_x = 0;
    human_lsr_y = 0;
    human_lsr_u = 0;    
  }

  // Filtering sensed humans & history...
  FilterPU( human_lsr_x, human_lsr_y, human_lsr_u, &history_x, &history_y, &history_u );
  
  track_x = history_x.front();
  track_y = history_y.front();
  track_u = history_u.front();  
}


// Euclidean distance between two coordinate points
double Dist2D( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}


void FilterPU( double h_x, double h_y, double h_u, list <double> *hist_x, list <double> *hist_y, list <double> *hist_u ){

  double num_x = h_x * h_u;
  double num_y = h_y * h_u;
  double num_u = h_u;

  double den_x = h_u;
  double den_y = h_u;
  double den_u = 1;   // Only one recognizer ( LRF )...

  list<double>::iterator it;

  int k;
  for( k = 1, it = (*hist_x).begin(); it != (*hist_x).end(); it++, k++ ){
    num_x += (*it) * pow( TRACK_ALPHA, (double) k );
    den_x += pow( TRACK_ALPHA, (double) k );
  }
  for( k = 1, it = (*hist_y).begin(); it != (*hist_y).end(); it++, k++ ){
    num_y += (*it) * pow( TRACK_ALPHA, (double) k );
    den_y += pow( TRACK_ALPHA, (double) k );
  }
  for( k = 1, it = (*hist_u).begin(); it != (*hist_u).end(); it++, k++ ){
    num_x += (*it) * pow( TRACK_ALPHA, (double) k );
    den_x += pow( TRACK_ALPHA, (double) k );
  }


  double ax = (den_x != 0) ? (num_x / den_x) : 0;
  double ay = (den_y != 0) ? (num_y / den_y) : 0;
  double au = (den_u != 0) ? (num_u / den_u) : 0;

  (*hist_x).push_front( ( fabs(ax) < MIN_VAL) ? 0 : ax );
  (*hist_y).push_front( ( fabs(ay) < MIN_VAL) ? 0 : ay );
  (*hist_u).push_front( ( fabs(au) < MIN_VAL) ? 0 : au );

  
  // History size limit exceeded
  if( (*hist_x).size() > TRACK_HISTORY_SIZE ){
    (*hist_x).pop_back();
    (*hist_y).pop_back();
    (*hist_u).pop_back();
  }

}


