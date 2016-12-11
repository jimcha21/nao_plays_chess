/**
 * @function cornerDetector_Demo.cpp
 * @brief Demo code for detecting corners using OpenCV built-in functions
 * @author OpenCV team
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/Bool.h"
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

// Variables for debugging.. ignore ..
bool view_mode_debug=false;
bool picture_mode_debug=false;

Mat src, src_gray;
Mat snap_one,snap_two;
std::vector<cv::Point> chess_topic_points;
RNG rng(12345);

int frame_received=0;
bool enable_snap=false;
float frameDensity[8][8][3];
float squareDensity[8][8][3]; //0 for black, 1 for white , 2 for und

static const std::string OPENCV_WINDOW = "Source Image";

/// Function headers
int *square_CornPoints(/*std::string str*/int int_letter ,int num);
bool checkifItsInsidetheSquare(Mat,Point,bool);

///////////////////////////////////////////////////////////////////////////

void chessboardVectorTopic(const vision::ChessVector& data){
  cv::Point pt;
  chess_topic_points.clear();
  //ROS_INFO("size %d",data.p_vector.size());
 //ROS_INFO("ta prwta %d %d ",data.p_vector[0].x,data.p_vector[0].y);
  for(int i=0;i<data.p_vector.size();i++){ //till 81..
    pt.x=data.p_vector[i].x;
    pt.y=data.p_vector[i].y;
    chess_topic_points.push_back(pt);
  }
}

/////////////////////////

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber chess_sub;
  ros::Subscriber snapshot_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
          // Subscrive to input video feed and publish output video feed
          chess_sub = nh_.subscribe("mapped_chessboard_knob_coordinates", 10, chessboardVectorTopic); //chessboard subscriber

          if(!view_mode_debug){    
             snapshot_sub = nh_.subscribe("take_snaps", 10, &ImageConverter::Snapshot, this); //chessboard subscriber
          }else{          
            //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::movementDetector, this);       
            image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::movementDetector, this);    
          }

          //image_sub_.shutdown();
          //image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }

  ~ImageConverter()
  {
          cv::destroyWindow(OPENCV_WINDOW);
  }

/////////////////////////

  void Snapshot(const std_msgs::Bool& snap){
    
    ROS_INFO("snapdata: %s", snap.data ? "true" : "false");
    
    if(snap.data){
       image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::movementDetector, this);    
       enable_snap=snap.data;
       //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::movementDetector, this);    
       ROS_INFO("IRRTHE ?");   
    }

    if(frame_received==2){
      cv::imshow("snap one",snap_one);
      cv::imshow("snap two",snap_two);
   

      //sub the two images
      Mat new_image3(snap_one.size(), snap_one.type());
      //new_image3 = new_image2 - new_image1;
      cv::absdiff(snap_one, snap_two, new_image3);
      cv::imshow("image2-das", new_image3);
      inRange(new_image3, cv::Scalar(30,0,0), cv::Scalar(255,255,255 ), new_image3); //BGR Scalar not RGB sequence
      cv::imshow("end", new_image3);
     cv::waitKey(3);
    }



   ROS_INFO("end-");
  }

/////////////////////////

  void movementDetector(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;

      if(chess_topic_points.size()==0){
         ROS_INFO("No ChessPoints received..");
      }

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

      if(!picture_mode_debug){
        src=cv_ptr->image.clone();
      }else{
        src = imread("src/vision/src/data/3.jpg", 1);
        if(src.empty())
        {
          ROS_INFO("can not open the image");// << filename << endl;
          return ;
        }
      }
      //cvtColor( src, src_gray, COLOR_BGR2GRAY ); 

      if(frame_received==0){
        frame_received++;
        snap_one=src.clone();
      }else if(frame_received==1){
        frame_received++;
        snap_two=src.clone();
      } // else..

/*      for( int j = 0; j < chess_topic_points.size(); j++ ){
          circle(src, Point(chess_topic_points[j].x,chess_topic_points[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("she %d -> %d %d",chess_topic_points[j].x,chess_topic_points[j].y);
      }*/
/*      cv::imshow(OPENCV_WINDOW,src);
      cv::waitKey(3);*/

      chess_topic_points.clear();
      if(!view_mode_debug){
        image_sub_.shutdown();
        return ;
      }

  }
};

////////////////////////////////////////////////////////MAIN/////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "movement_detector_node");
  ImageConverter ic;
  ROS_INFO("hello");
  for(int i=0;i<8;i++){
      for(int j=0;j<8;j++){
        squareDensity[i][j][0]=0;
        squareDensity[i][j][1]=0;
        squareDensity[i][j][2]=0;
      }
  }
  ros::spin();
  return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int *square_CornPoints(/*std::string letter,*/int int_letter, int num){
    static int sq_points[4];
    int mul=0;
/*    if(letter.compare("a")==0){
      mul=0;
    }else if(letter.compare("b")==0){
      mul=1;
    }else if(letter.compare("c")==0){
      mul=2;
    }else if(letter.compare("d")==0){
      mul=3;
    }else if(letter.compare("e")==0){
      mul=4;
    }else if(letter.compare("f")==0){
      mul=5;
    }else if(letter.compare("g")==0){
      mul=6;
    }else if(letter.compare("h")==0){
      mul=7;
    }*/

    mul=int_letter-1;

    sq_points[0]=(num-1)+mul*9;
    sq_points[1]=num+mul*9;
    sq_points[2]=(num-1)+mul*9+9;
    sq_points[3]=num+mul*9+9;
    return sq_points;
}

bool checkifItsInsidetheSquare(Mat kati,Point poin,bool chessboard_contour){
  CvPoint left_bottom_p,left_top_p,right_top_p,right_bottom_p;
  if(chessboard_contour){
      left_bottom_p.x=chess_topic_points[0].x-10;
      left_bottom_p.y=chess_topic_points[0].y+10;
      left_top_p.x=chess_topic_points[8].x-10;
      left_top_p.y=chess_topic_points[8].y-10;
      right_bottom_p.x=chess_topic_points[72].x+20;
      right_bottom_p.y=chess_topic_points[72].y+20;
      right_top_p.x=chess_topic_points[80].x+10;
      right_top_p.y=chess_topic_points[80].y-10;
  }
  ROS_INFO("%d %d %d %d %d %d %d",left_bottom_p.x,left_bottom_p.y,left_top_p.x,left_top_p.y,right_bottom_p.x,right_bottom_p.y,right_top_p.x,right_top_p.y);
  line( kati, Point(chess_topic_points[0].x,chess_topic_points[0].y),Point(chess_topic_points[8].x,chess_topic_points[8].y), Scalar(0,0,255), 1, LINE_AA);
  line( kati, Point(chess_topic_points[72].x,chess_topic_points[72].y),Point(chess_topic_points[80].x,chess_topic_points[80].y), Scalar(0,0,255), 1, LINE_AA);
  line( kati, Point(chess_topic_points[0].x,chess_topic_points[0].y),Point(chess_topic_points[72].x,chess_topic_points[72].y), Scalar(0,0,255), 1, LINE_AA);
  line( kati, Point(chess_topic_points[80].x,chess_topic_points[80].y),Point(chess_topic_points[8].x,chess_topic_points[8].y), Scalar(0,0,255), 1, LINE_AA);
  
  line( kati, left_bottom_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
  line( kati, right_bottom_p,right_top_p, Scalar(0,0,255), 1, LINE_AA);
  line( kati, left_bottom_p,right_bottom_p, Scalar(0,0,255), 1, LINE_AA);
  line( kati, right_top_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
  cv::imshow("aaa", kati);


  return false;
}