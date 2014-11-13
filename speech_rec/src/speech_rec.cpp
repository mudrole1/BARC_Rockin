#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"

bool recIsOn = false;
bool recYNon = false;
int taskID = 0;

std::string person = "";

void startSpeech(const std_msgs::Empty& msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/recognizer2/start");
  std_srvs::Empty srv;
  if (client.call(srv))
  {
    ROS_INFO("The speech recognizer is ready.");
    recIsOn = true;
  }
  else
  {
   ROS_INFO("Somethig has failed, speech recognizer is not ready");
   recIsOn = false;
  }
}

void stopSpeech(const std_msgs::Empty& msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/recognizer2/stop");
  std_srvs::Empty srv;


  if (client.call(srv))
  {
    ROS_INFO("The speech recognizer is stopped.");
    recIsOn = false;
  }
  else
  {
    ROS_INFO("Somethig has failed, speech recognizer is not stopped");
    recIsOn = true;
  }
}

void startYN(const std_msgs::Empty& msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/recognizerYN/start");
  std_srvs::Empty srv;

  if (client.call(srv))
  {
    ROS_INFO("The YN recognizer is ready.");
    recYNon = true;
  }
  else
  {
   ROS_INFO("Somethig has failed, YN recognizer is not ready");
   recYNon = false;
  }
}

void stopYN(const std_msgs::Empty& msg)
{
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/recognizerYN/stop");
  std_srvs::Empty srv;


  if (client.call(srv))
  {
    ROS_INFO("The YN recognizer is stopped.");
    recYNon = false;
  }
  else
  {
    ROS_INFO("Somethig has failed, YN recognizer is not stopped");
    recYNon = true;
  }
}

void getResponse(const std_msgs::String::ConstPtr& msg) //
{
  //std_msgs::String 
  std::string sentence = msg->data;
  bool person_arr[4] = {false, false, false, false}; //doctor, postman, deliman, unkown

  ros::NodeHandle n;
  ros::Publisher speech_pub = n.advertise<std_msgs::String>("/speech_rec/response",1); //TODO buffer?
  std_msgs::String answer;

  ros::Rate loop_rate(10);

  if(recIsOn)
  {

    //doctor
    int doctorInd = (int) sentence.find("doctor");
    int kimbleInd = (int) sentence.find("kimble");

    //postman
    int postmanInd = (int) sentence.find("postman");
    int parcelInd = (int) sentence.find("parcel");
    int letterInd = (int) sentence.find("letter");
  
    //deliman
    int delimanInd = (int) sentence.find("deliman");
    int breakfastInd = (int) sentence.find("breakfast");
    int mealInd = (int) sentence.find("meal");
    int foodInd = (int) sentence.find("food");

    //unknown
    int newspaperInd = (int) sentence.find("newspaper");
    int magazineInd = (int) sentence.find("magazine");    
    int buyInd = (int) sentence.find("buy"); 

    if((doctorInd>=0)&&(kimbleInd>=0))
    {
      person_arr[0] = true;  
    }
    if(((postmanInd>=0)&&(parcelInd>=0))||((postmanInd>=0)&&(letterInd>=0)))
    {
      person_arr[1] = true;  
    }
    if(((delimanInd>=0)&&(breakfastInd>=0))||((delimanInd>=0)&&(mealInd>=0))||((delimanInd>=0)&&(foodInd>=0)))
    {
      person_arr[2] = true;   
    }
    if((newspaperInd >0)||(magazineInd > 0)||(buyInd > 0))
    {
      person_arr[3] = true;
    }

    //if more are active, speech rec got confused
    int amount = 0;

    for(int i=0; i<4; i++)
      if(person_arr[i])
        amount++;

    //speech detected more possibilities or none
    if(amount == 1)
    {
      if(person_arr[0])
      {
        person = "Doctor";
        ROS_INFO("Doctor");
      }
      else if(person_arr[1])
      {
        person = "Postman";
        ROS_INFO("Postman");
      }
      else if(person_arr[2])
      {
        person = "Deliman";
        ROS_INFO("Deliman");
      }
      else if(person_arr[3])
      {
        person = "Unknown";
        ROS_INFO("Unknown");
      }
      std_msgs::Empty e;
      stopSpeech(e);

      answer.data = person;
      while(speech_pub.getNumSubscribers() == 0)
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
      speech_pub.publish(answer);
    }   
  }
}

void getYN(const std_msgs::String::ConstPtr& msg) 
{
  std::string sentence = msg->data;
  
  ros::NodeHandle node;
  ros::Publisher response_pub = node.advertise<std_msgs::Int32>("/speech_rec/confirmation/response", 1);
  std_msgs::Int32 answer;

  ros::Rate loop_rate(10);

  size_t y = sentence.find("yes");
  size_t n = sentence.find("no");

  int numYes = 0;
  int numNo = 0;

  while((int)y >= 0)
  {
    numYes++;
    y = sentence.find("yes",y+1); 
  }

  while((int)n >= 0)
  {
    numNo++;
    n = sentence.find("no",n+1); 
  }

  //yeses only
  if((numYes>0)&&(numNo==0))
  {
    ROS_INFO("yes");
    answer.data = 1;
    while(response_pub.getNumSubscribers() == 0)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    response_pub.publish(answer);
    std_msgs::Empty e;
    stopYN(e);
  }
  //noes only
  else if((numYes==0)&&(numNo>0)) 
  {
    ROS_INFO("no");
    answer.data = 0;
    while(response_pub.getNumSubscribers() == 0)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    response_pub.publish(answer);
    std_msgs::Empty e;
    stopYN(e);
  }
  else //some combination
  {
    ROS_INFO("unknown yes/no");
    answer.data = -1;
    while(response_pub.getNumSubscribers() == 0)
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
    response_pub.publish(answer);
    std_msgs::Empty e;
    stopYN(e);
  }
  
}

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "speech_rec");
  ros::NodeHandle nRecReq, nRecSto, nSpe, nYN, nConReq;

  if(argc < 2)
  {
    ROS_INFO("You need to specify which task is performed 1-3.");
    return -1;
  }

  taskID = strtol(argv[1], NULL, 10);

  //as the recognizer is working by default at the start, we would like to stop it first, but after it is ready to listen
  std_msgs::Empty e;
  stopSpeech(e);
  stopYN(e);


  /*if(taskID == 1)
  {
  }
  else if(taskID == 2)
  {*/
    ROS_INFO("task 2");
    //subsribe to two control topics from the state machine
    ros::Subscriber subRec1 = nRecReq.subscribe("/recognition/request", 10, startSpeech);
    ros::Subscriber subRec2 = nRecSto.subscribe("/recognition/stop", 10, stopSpeech);

    //general commands
    ros::Subscriber subSpe = nSpe.subscribe("/recognizer2/output", 10, getResponse);

    //yes no
    ros::Subscriber subYN = nYN.subscribe("/recognizerYN/output", 10, getYN);

    //confirmation for yes/no
    ros::Subscriber subCon = nConReq.subscribe("/speech_rec/confirmation/request", 10, startYN);
  /*}
  else if(taskID == 3)
  {
  }*/
 
  
  

  ros::spin();

  return 0;
}

// /speech_rec/response - Doctor, Postman, Deliman, Unknown or nothing
// /speech_rec/confirmation/request Empty
// /speech_rec/confirmation/response bool


//TODO add Are you a doctor? Yes - no
