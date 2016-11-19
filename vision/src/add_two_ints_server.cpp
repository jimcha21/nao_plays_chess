/*
#include "ros/ros.h"
#include "vision/AddTwoInts.h"

bool add(vision::AddTwoInts::Request  &req,
         vision::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
*/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points,text;
    points.header.frame_id = text.header.frame_id = "/my_frame";
    points.header.stamp  = text.header.stamp= ros::Time::now();
    points.ns  = text.ns = "points_and_lines";
    points.action  = text.action  = visualization_msgs::Marker::ADD;

    points.pose.orientation.w=text.pose.orientation.w= 1.0;



    points.id =0;
    text.id = 1;


    //for points.. maybe usefull to index the center points of the chess squeres
    //points.type = visualization_msgs::Marker::POINTS;
    points.type = visualization_msgs::Marker::CUBE_LIST;    
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;


    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.9;
    points.scale.y = 0.9;
    points.scale.z = 0.2;

    text.pose.position.x = 10.9;
    text.pose.position.y = 10.9;
    text.scale.z = 0.2;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    text.color.b = 1.0f;
    text.color.a = 1.0;
        text.text = "whaza";

    // Create the vertices for the points and lines
    for (int i = 1; i < 9; ++i)
      {
        for (int j = 1; j < 9; ++j)
        {

        geometry_msgs::Point pi;
        pi.x = i;
        pi.y = j;
        pi.z = 0; // in a relation with the started pose.. init tag pose
        points.points.push_back(pi);
        text.pose.position.x = i;
        text.pose.position.y = j;
        text.scale.z = 2.2;     
        text.lifetime = ros::Duration (10.0);
         marker_pub.publish(text);
       }
    }


    marker_pub.publish(points);
    marker_pub.publish(text);
    r.sleep();

    f += 0.04;
  }
}
