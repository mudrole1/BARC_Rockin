// FILE: "Navigator01.cpp"
// AUTHOR: Marco Antonio Becerra Pedraza
// > http://www.marcobecerrap.com
// SUMMARY: This program receives the "Pose" (atractor) and the "Laser Scan" (obstacles) msgs. 
// > Then calculates the potential field vectors. Finally obtains & publish the inverse kinematics 
// > needed on the differential steer robot to execute the movement.

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>
#include <laser_geometry/laser_geometry.h>
#include <follow_sim/Pose.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <string>

#define PI 3.1416

#define MIN_DISTANCE_HUMAN 0.7

#define ATRACTION_K     1
#define ATRACTION_ALPHA 1.5
#define REPULSION_K     4
#define REPULSION_ALPHA 0.0075

#define W_MAX  0.7
#define W_MIN -0.4
#define BETA   1
#define RADIUS 1 //|-> Vehicle constants...
#define L      1 //|

using namespace std;

bool publish_on  = false;
bool robot_on    = false;
double v_x = 0;
double w_z = 0;

int g_counter = 0;


double px_atractor = 0;
double py_atractor = 0;
double pr_atractor = 0;
double pt_atractor = 0;
double pu_atractor = 0;
double px_repulsor = 0;
double py_repulsor = 0;
double pr_repulsor = 0;
double pt_repulsor = 0;
double pu_repulsor = 0;

bool ATRACTOR_READY = false;
bool REPULSOR_READY = false;

double Dist2D( double x0, double y0, double x1, double y1 );
double Fa( double pr );
double Fr( double pr );
double atan2_test( double py, double px );
double rad2grad( double rad );



void PoseCallback (const follow_sim::Pose::ConstPtr& msg){
  ATRACTOR_READY = true;
  px_atractor = (*msg).x;
  py_atractor = (*msg).y; 
  pr_atractor = Dist2D( 0, 0, px_atractor, py_atractor );
  pt_atractor = atan2_test( py_atractor, px_atractor );
  pu_atractor = (*msg).uncertainty;
}


void LaserCallback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud PCLmsg;
  sensor_msgs::PointCloud *LaserCloud;

  publish_on = true;
  robot_on = true;
  REPULSOR_READY = true;
  
  // LaserScan To PointCloud
  projector.projectLaser( *msg, PCLmsg );  
  LaserCloud = &PCLmsg;
  //for( unsigned i = 0; i < LaserCloud->points.size(); i++ )
    //cout << i << ") " << LaserCloud->points[i].x << " " << LaserCloud->points[i].y <<  endl;
  //cout << LaserCloud->points.size() << " = " << (*msg).ranges.size() << endl;


  double px, py;
  double r0 = 1000;
  double t0;
  double r;
  double t;
  unsigned idx = 0;
  for( unsigned i = 0; i < LaserCloud->points.size(); i++ ){    
    px = LaserCloud->points[i].x;
    py = LaserCloud->points[i].y;
    r = Dist2D( 0, 0, px, py );
    t = atan2_test( py, px );
    if( r < r0 ){
      idx = i;
      r0 = r;
      t0 = t;
    }
  }

  px_repulsor = LaserCloud->points[ idx ].x;
  py_repulsor = LaserCloud->points[ idx ].y;
  pr_repulsor = Dist2D( 0, 0, px_repulsor, py_repulsor );
  pt_repulsor = atan2_test( py_repulsor, px_repulsor );
  pu_repulsor = 1;

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Navigator");

  ros::NodeHandle n;

  ros::Publisher  node_pub = n.advertise <geometry_msgs::Twist>("robot_1/cmd_vel", 2); // This has to be the HUMAN topic
  ros::Subscriber node_sub1 = n.subscribe("robot_1/base_scan", 2, LaserCallback);
  ros::Subscriber node_sub2 = n.subscribe("track", 2, PoseCallback);

  geometry_msgs::Twist msg; // robot_vel
  
  double Fa_x, Fa_y, Fa_r, Fa_t;
  double Fr_x, Fr_y, Fr_r, Fr_t;
  double F_x, F_y, F_r, F_t;

  double wwr;
  double wwl;
  
  while( ros::ok() ){
    if( ATRACTOR_READY == true && REPULSOR_READY == true ){

      // Obtain Forces (atraction & repulsion)...
      Fa_r = Fa( pr_atractor );
      Fa_t = pt_atractor;
      Fr_r = Fr( pr_repulsor );
      Fr_t = pt_repulsor + PI;
      
      //cout << "Fa_r[" << Fa_r << "] Fa_t[" << Fa_t << "]" << endl;
      //cout << "Fr_r[" << Fr_r << "] Fr_t[" << Fr_t << "]" << endl;
      //cout << "---------------" << endl;

      // Obtain the resultant...
      Fa_x = Fa_r*cos( Fa_t );
      Fa_y = Fa_r*sin( Fa_t );
      Fr_x = Fr_r*cos( Fr_t );
      Fr_y = Fr_r*sin( Fr_t );      
      F_x = Fa_x + Fr_x;
      F_y = Fa_y + Fr_y;
      F_r = Dist2D( 0, 0, F_x, F_y );
      F_t = atan2_test( F_y, F_x );

      // Obtain wheel speeds....
      wwr = F_r * ( ( W_MAX - W_MIN )*exp( (- BETA) * pow( F_t + ( PI / 4 ), 2 ) ) + W_MIN );
      wwl = F_r * ( ( W_MAX - W_MIN )*exp( (- BETA) * pow( F_t - ( PI / 4 ), 2 ) ) + W_MIN );

      // Direct Kinematics
      v_x = RADIUS * ( wwl + wwr ) / 2;
      w_z = RADIUS * ( wwl - wwr ) / L;

      
      // Lost Human
      if( pu_atractor == 0 || pr_atractor < MIN_DISTANCE_HUMAN ){
	v_x = 0;
	w_z = 0;
      }
      
      /* 
      cout << "pr_atractor[" << pr_atractor << "] pt_atractor[" << rad2grad( pt_atractor ) << "]" << endl;
      cout << "pr_repulsor[" << pr_repulsor << "] pt_repulsor[" << rad2grad( pt_repulsor ) << "]" << endl;
      cout << "Fa_r[" << Fa_r << "] Fa_t[" << rad2grad( Fa_t ) << "]" << endl;
      cout << "Fr_r[" << Fr_r << "] Fr_t[" << rad2grad( Fr_t ) << "]" << endl;
      cout << "F_r[" << F_r << "] F_t[" << rad2grad( F_t ) << "]" << endl;
      cout << "wwe[" << wwr << "] wwl[" << wwl << "]" << endl;
      cout << "v_x[" << v_x << "] w_z[" << w_z << "]" << endl;
      cout << "---------------------" << endl;
      */

      //w_z = 0.1;
      //v_x = 0;
      


      msg.linear.x  = v_x;
      msg.angular.z = w_z;
      node_pub.publish( msg );
      publish_on = false;
    }
    ros::spinOnce();
  }
  return 0;
}



// Euclidean distance between two coordinate points
double Dist2D( double x0, double y0, double x1, double y1 ){
  return sqrt( pow( x0 - x1, 2 ) + pow( y0 - y1, 2 ) );
}

double Fa( double pr ){
  return ( ( ( - ATRACTION_K ) * exp( ( - ATRACTION_ALPHA ) * fabs(pr) * 1000 ) ) + 1 );
}

double Fr( double pr ){
  return ( REPULSION_K * exp( ( - REPULSION_ALPHA ) * fabs(pr) * 1000 ) );
}

double atan2_test( double py, double px ){
  if( py == 0 && px == 0 )
    return 0;
  else 
    return atan2( py, px );
}

double rad2grad( double rad ){
  return ( rad * 180 / PI);
}
