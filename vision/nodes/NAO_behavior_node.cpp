#include "ros/ros.h"
#include "std_msgs/Bool.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "NAO_behavior_node");

  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("take_snaps", 3);
  ros::Rate loop_rate(5);

  int count = 0;
  while (ros::ok())
  { 
    //ROS_INFO("%d",count);
   
    std_msgs::Bool msg;

    if((count%15==0||count%25==0) && count!=0){
      msg.data = true;
      ROS_INFO("MOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWNMOVE THE PAWN");
    }else{
      msg.data = false;
    }
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}