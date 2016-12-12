/**
 * @function cornerDetector_Demo.cpp
 * @brief Demo code for detecting corners using OpenCV built-in functions
 * @author OpenCV team
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "vision/ChessVector.h"
#include "vision/ChessPoint.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
Mat src, src_gray, dst;
bool debug_mode;

std::vector<cv::Point> chess_knob_vector_;
std::vector<cv::Point3d> temp_vector;

int hough_thres=50;

//******************************************************************************************

void chessboardVectorTopic(const vision::ChessVector& data){
  cv::Point pt;
  chess_knob_vector_.clear();
  //ROS_INFO("size %d",data.p_vector.size());
  for(int i=0;i<data.p_vector.size();i++){ //till 81..
    pt.x=data.p_vector[i].x;
    pt.y=data.p_vector[i].y;
    chess_knob_vector_.push_back(pt);
  }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber chess_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
          // Subscrive to input video feed and publish output video feed

          image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::imageCb, this);
          //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);    
          chess_sub = nh_.subscribe("chessboard_knob_coordinates", 10, chessboardVectorTopic); //chessboard subscriber
          //add it
          //houghed_chesspoints = nh_.advertise<vision::ChessVector>("houghed_chessboard_knob_coordinates",0);
        

          //image_pub_ = it_.advertise("/image_converter/output_video", 1);
          cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
          cv::destroyWindow(OPENCV_WINDOW);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

      cv_bridge::CvImagePtr cv_ptr;

/*      for(int i=0;i<chess_knob_vector_.size();i++)
      { ROS_INFO("VECTOR %d %d",chess_knob_vector_[i].x,chess_knob_vector_[i].y);}*/
  
      try
      {
          //The input from the robot is encoded with RGB8.
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }
          
      //src=cv_ptr->image.clone();

      if(!debug_mode){
        src=cv_ptr->image.clone();
      }else{
        src = imread("src/vision/src/data/3.jpg", 1);
        if(src.empty())
        {
          ROS_INFO("can not open the image");// << filename << endl;
          return ;
        }
      }
      // HOUGH
/*      Canny(src, dst, 800, 200, 3);
      cvtColor( dst, src_gray, COLOR_GRAY2BGR ); 
      cv::imshow("canny",dst);
      /// Set some parameters
     
      vector<Vec4i> lines;
      HoughLinesP(dst, lines, 1, CV_PI/180, hough_thres, 40,20);
      ROS_INFO("brike %d grammes",lines.size());

      for( size_t i = 0; i < lines.size(); i++ )
      {
          Vec4i l = lines[i];
          //ROS_INFO("PAEI GIA LINE STO %d %d me %d %d",l[0], l[1], l[2], l[3]);
          ROS_INFO("diafores  %d %d",   (l[3]-l[1]),(l[2]-l[0]) );
          if(l[2]-l[0]==0){
            continue;
          }
          ROS_INFO("syntelesths dieythhsnsh %f",   (float)(l[3]-l[1])/(float)(l[2]-l[0]) );
          //if(l[1]>445 && l[3]>445) //chessboard spatial limiter for hough eytheies lysh
          line( src, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
      }

      line( src, Point(70, 691), Point(290,482), Scalar(0,0,255), 3, LINE_AA);
      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);*/
       imwrite( "easy_13.jpg", src );
      cv::imshow("OPENCV_WINDOW",src);
      //cv::imshow("OPENCV_WINDOW2",src_gray);
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());

      /*chess_featured_points_.clear(); //no need to check for overflow..
      chess_processed_points_.clear();
      temp_vector.clear(); //vector3d*/
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "houghlines_receiver");
  ImageConverter ic;
  ros::spin();
  return 0;
}

