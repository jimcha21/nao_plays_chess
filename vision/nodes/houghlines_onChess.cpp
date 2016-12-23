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
RNG rng(12345);
static const std::string OPENCV_WINDOW = "Image window";
Mat src, src_gray, dst;
bool debug_mode=false;

std::vector<vision::ChessPoint> chess_knob_vector_;
std::vector<cv::Point3d> temp_vector;

int hough_thres=20;

//******************************************************************************************

void chessboardVectorTopic(const vision::ChessVector& data){
  vision::ChessPoint pt;
  chess_knob_vector_.clear();
  //ROS_INFO("size %d",data.p_vector.size());
  for(int i=0;i<data.p_vector.size();i++){ //till 81..
    pt.x=data.p_vector[i].x;
    pt.y=data.p_vector[i].y;
    pt.state=data.p_vector[i].state;
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
          chess_sub = nh_.subscribe("mapped_chessboard_knob_coordinates", 10, chessboardVectorTopic); //chessboard subscriber
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

/*     for(int i=0;i<chess_knob_vector_.size();i++)
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

     if(chess_knob_vector_.size()>0){// line(src, Point(chess_knob_vector_[72].x, chess_knob_vector_[72].y), Point(chess_knob_vector_[80].x, chess_knob_vector_[80].y), Scalar(0,0,255), 3, LINE_AA);

      Canny(src, dst, 550, 200, 3);
      cvtColor( dst, src_gray, COLOR_GRAY2BGR ); 
      //cv::imshow("canny",dst);
      /// Set some parameters
     
      vector<Vec4i> lines;
      HoughLinesP(dst, lines, 1, CV_PI / 180, hough_thres,(int)sqrt(pow((chess_knob_vector_[0].x-chess_knob_vector_[72].x),2)+pow((chess_knob_vector_[0].y-chess_knob_vector_[72].y),2))-80,40);
      //ROS_INFO("%d lines found",lines.size());
      //cv::imshow("m to canny",dst);

      //ROS_INFO("O syntelesths iytthnshs einai %d",chess_knob_vector_[0].y);


      float chess_vertical_slopes[9]={};
      int column=0;
      for(int j=0;j<9;j++){        
        int start=0+column,end=8+column;
        for(int p=0+column;p<9+column;p++){
          ROS_INFO("p=%d",p);
          if(chess_knob_vector_[p].state.compare("feat_mapped")==0||chess_knob_vector_[p].state.compare("shi_mapped")==0) ROS_INFO("BRIKE STHN %d grammh to %d hehe",linee,p);
        }
        line( src, Point(chess_knob_vector_[start].x,chess_knob_vector_[start].y), Point(chess_knob_vector_[end].x,chess_knob_vector_[end].y),Scalar( 255,0,0 ), 3, LINE_AA); 
        ROS_INFO("termatise ");
       // ROS_INFO("line me %d",line);
        column=column+9;
      }

       float la;
       float lambda_down=(float)(chess_knob_vector_[72].y-chess_knob_vector_[0].y)/(chess_knob_vector_[72].x-chess_knob_vector_[0].x);
       float opa=(float)(chess_knob_vector_[73].y-chess_knob_vector_[1].y)/(chess_knob_vector_[73].x-chess_knob_vector_[1].x);
        float opa1=(float)(chess_knob_vector_[74].y-chess_knob_vector_[2].y)/(chess_knob_vector_[74].x-chess_knob_vector_[2].x);
         float opa2=(float)(chess_knob_vector_[75].y-chess_knob_vector_[3].y)/(chess_knob_vector_[75].x-chess_knob_vector_[3].x);
       //ROS_INFO("O SYNTELESTHS EINAI %f %f %f %f",lambda_down,opa,opa1,opa2);
       float lambda_side=(float)(chess_knob_vector_[76].y-chess_knob_vector_[72].y)/(chess_knob_vector_[76].x-chess_knob_vector_[72].x);
       //ROS_INFO("O syntelesths iytthnshs einai %f sid:%f",lambda_down,lambda_side);
       int num_lines=0;bool end_case=true;
       //ROS_INFO("Ara to distance apo akrhs s akrh einai %f",(float)sqrt(pow((chess_knob_vector_[0].x-chess_knob_vector_[72].x),2)+pow((chess_knob_vector_[0].y-chess_knob_vector_[72].y),2)));
       //ROS_INFO("kai to orizontio %f",(float)sqrt(pow((chess_knob_vector_[80].x-chess_knob_vector_[72].x),2)+pow((chess_knob_vector_[80].y-chess_knob_vector_[72].y),2)));
      for( size_t i = 0; i < lines.size() && end_case; i++ )
      {
          Vec4i l = lines[i];
          //ROS_INFO("PAEI GIA LINE STO %d %d me %d %d",l[0], l[1], l[2], l[3]);
          //ROS_INFO("diafores  %d %d",   (l[3]-l[1]),(l[2]-l[0]) );
          //if(l[2]-l[0]==0) continue;
          //ROS_INFO("syntelesths dieythhsnsh %f",   (float)(l[3]-l[1])/(float)(l[2]-l[0]) );
          //if(l[1]>445 && l[3]>445) //chessboard spatial limiter for hough eytheies lysh
          la=(float)(l[3]-l[1])/(l[2]-l[0]);
          //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), 3, LINE_AA);
          line( src, Point(chess_knob_vector_[54].x,chess_knob_vector_[54].y), Point(chess_knob_vector_[55].x,chess_knob_vector_[55].y),Scalar( 255,0,0 ), 3, LINE_AA);   
          //ROS_INFO("la is %f",la);

          if(abs(la-lambda_down)<=5){
            // ROS_INFO("line with coord %d %d %d %d",l[0],l[2],l[1],l[3]);
            num_lines++;
            if(num_lines==10){
              end_case=true;//!
            }

            int x_mid=(int)(l[0]+l[2])/2;
            int y_mid=(int)(l[1]+l[3])/2;

            float point_distance=9999,point_distance_a=9999,point_distance_b=9999;
            int nearest;
            for(int j=36;j<=44;j++){
              float temp_mid_dist=(float)sqrt(pow((x_mid-chess_knob_vector_[j].x),2)+pow((y_mid-chess_knob_vector_[j].y),2));
              float temp_a_dist=(float)sqrt(pow((l[0]-chess_knob_vector_[j].x),2)+pow((l[1]-chess_knob_vector_[j].y),2));
              float temp_b_dist=(float)sqrt(pow((l[2]-chess_knob_vector_[j].x),2)+pow((l[3]-chess_knob_vector_[j].y),2));
              //ROS_INFO("Distances are %f %f %f",temp_mid_dist,temp_a_dist,temp_b_dist);
              if(temp_mid_dist<point_distance/*&&temp_a_dist<point_distance_a&&temp_b_dist<point_distance_b*/){                  
                //ROS_INFO("min found at %d",j);
                point_distance= temp_mid_dist;
                point_distance_b=temp_b_dist;
                point_distance_a=temp_a_dist;
                nearest=j;
              }

            }
            //ROS_INFO("nearest is %d",nearest);
            /*if(nearest==36){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
              line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,255 ), 3, LINE_AA);            
            }else if(nearest==37){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
              line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,0 ), 3, LINE_AA);            
            }else if(nearest==38){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,0 ), 3, LINE_AA);            
            }else if(nearest==39){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
              line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,255,0 ), 3, LINE_AA);            
            }else if(nearest==40){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                            line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,255 ), 3, LINE_AA);            
            }else if(nearest==41){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                            line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,255 ), 3, LINE_AA);            
            }else if(nearest==42){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                          line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,125 ), 3, LINE_AA);            
            }else if(nearest==43){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                          line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,125,0 ), 3, LINE_AA);            
            }else if(nearest==44){
              //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                          line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 125,0,0 ), 3, LINE_AA);            
            }*/

            //line( src, Point(chess_knob_vector_[36].x,chess_knob_vector_[36].y), Point(chess_knob_vector_[37].x,chess_knob_vector_[37].y),Scalar( 255,0,0 ), 3, LINE_AA);   
          }/*
          else if(abs(la-lambda_side)<=1){
            line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), 3, LINE_AA);
          }*/
      }

    //ROS_INFO("%d filtered lines",num_lines);
}
      //line( src, Point(70, 691), Point(290,482), Scalar(0,0,255), 3, LINE_AA);
      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);*/
       //imwrite( "easy_13.jpg", src );
      //line( src, Point(70, 691), Point(290,482), Scalar(0,0,255), 3, LINE_AA);
      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      //imwrite( "easy_13.jpg", src ); //for storing the snapped images..
      cv::imshow(OPENCV_WINDOW,src);
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

