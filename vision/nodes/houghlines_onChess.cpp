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

float vertical_line_sl,horizontal_line_sl;
int v_x,v_y,h_x,h_y;
int hough_thres=20;
float vert_slope_threshold=0.25;

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
          //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);    o 
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
  ROS_INFO("PHRE %d",chess_knob_vector_.size());
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
      HoughLinesP(dst, lines, 1, CV_PI / 180, hough_thres,100/*(int)sqrt(pow((chess_knob_vector_[0].x-chess_knob_vector_[72].x),2)+pow((chess_knob_vector_[0].y-chess_knob_vector_[72].y),2))-80*/,40);
      ROS_INFO("%d lines found",lines.size());
      //cv::imshow("m to canny",dst);

      float chess_vertical_slopes[9]={};
      int column=0;

      if(chess_knob_vector_.size()==81){

      for(int j=0;j<9;j++){        
          int start=0+column,end=8+column;
          bool logic=false;

          //finding the (ideal-corrected) vertical lines, to extract their slopes..
          int p=0+column;
          bool proceed=false;
          do{
              //ROS_INFO("p=%d kai to chess=%d",p,chess_knob_vector_[p].x);
              if(!logic){
                if(chess_knob_vector_[p].state.compare("estimated")==0){
  /*                if(p==9+column){
                    start=0+column;
                    proceed=false;                //CRASHING FOR SOME REASON?!@
                    end=8+column;
                  }else{*/
                     proceed=true; //this is in case there is/are estimated points in the start of the examination, and we procceed to find the first "corrected one" to initialize the first line point
                /*  }               */
                }else{
                  start=p;
                  logic=true;
                  proceed=false;
                }
              }else{
                end=p;
              }
              p++;
          }while((chess_knob_vector_[p].state.compare("estimated")!=0 || proceed) && p<9+column && p<80/*!!?!?!see that*/);
          //ROS_INFO("paei gia to allo");
          //print line..
          //line( src, Point(chess_knob_vector_[start].x,chess_knob_vector_[start].y), Point(chess_knob_vector_[end].x,chess_knob_vector_[end].y),Scalar( 255,0,0 ), 3, LINE_AA); 
          //ROS_INFO("AFairw ta %d %d",chess_knob_vector_[end].x,chess_knob_vector_[start].x);
          chess_vertical_slopes[j]=(float)(chess_knob_vector_[end].y-chess_knob_vector_[start].y)/(chess_knob_vector_[end].x-chess_knob_vector_[start].x);
          column=column+9;
        }

        float area_limits[9][2]={};

        for(int j=0;j<9;j++){

          float points_distance=abs(chess_vertical_slopes[j]-chess_vertical_slopes[j+1]);
          float temp_thr=vert_slope_threshold;
          if(points_distance>10){
            vert_slope_threshold=0.05;
          }
          if(chess_vertical_slopes[j]<0 && chess_vertical_slopes[j+1]<0){
            if(chess_vertical_slopes[j]>chess_vertical_slopes[j+1]){
              area_limits[j][1]=chess_vertical_slopes[j]-points_distance*vert_slope_threshold;
              area_limits[j+1][0]=chess_vertical_slopes[j+1]+points_distance*vert_slope_threshold;
            }else{
              area_limits[j][1]=chess_vertical_slopes[j]+points_distance*vert_slope_threshold;
              area_limits[j+1][0]=chess_vertical_slopes[j+1]-points_distance*vert_slope_threshold;
            }
          }else if(chess_vertical_slopes[j]>0 && chess_vertical_slopes[j+1]>0){
            if(chess_vertical_slopes[j]>chess_vertical_slopes[j+1]){
              area_limits[j][1]=chess_vertical_slopes[j]+points_distance*vert_slope_threshold;
              area_limits[j+1][0]=chess_vertical_slopes[j+1]-points_distance*vert_slope_threshold;
            }else{
              area_limits[j][1]=chess_vertical_slopes[j]-points_distance*vert_slope_threshold;
              area_limits[j+1][0]=chess_vertical_slopes[j+1]+points_distance*vert_slope_threshold;
            }
          }else if(chess_vertical_slopes[j]<0 && chess_vertical_slopes[j+1]>0){
            points_distance=abs(chess_vertical_slopes[j])+abs(chess_vertical_slopes[j+1]);
            if(points_distance>10){
              vert_slope_threshold=0.05;
            }
            area_limits[j][1]=chess_vertical_slopes[j]-points_distance*vert_slope_threshold;
            area_limits[j+1][0]=chess_vertical_slopes[j+1]-points_distance*vert_slope_threshold;
            
          }else if(chess_vertical_slopes[j]>0 && chess_vertical_slopes[j+1]<0){/*
            points_distance=abs(chess_vertical_slopes[j])+abs(chess_vertical_slopes[j+1]);
            area_limits[j][1]=chess_vertical_slopes[j]+points_distance*vert_slope_threshold;
            area_limits[j+1][0]=chess_vertical_slopes[j+1]-points_distance*vert_slope_threshold;*/
          }//else 0 slope FLAG~
          //side limits

          vert_slope_threshold=temp_thr;
          if(j==0){
            area_limits[j][0]=chess_vertical_slopes[j];
          }else if(j==8){
            area_limits[j][1]=chess_vertical_slopes[j];
          }
        }

/*       for(int j=0;j<9;j++){
        ROS_INFO("slope of %d is %f and limits are %f %f",j+1,chess_vertical_slopes[j],area_limits[j][0],area_limits[j][1]);
       }*/

        float bottom_line_slope=(float)(chess_knob_vector_[72].y-chess_knob_vector_[0].y)/(chess_knob_vector_[72].x-chess_knob_vector_[0].x);
        //float opa=(float)(chess_knob_vector_[73].y-chess_knob_vector_[1].y)/(chess_knob_vector_[73].x-chess_knob_vector_[1].x);
        //float opa1=(float)(chess_knob_vector_[74].y-chess_knob_vector_[2].y)/(chess_knob_vector_[74].x-chess_knob_vector_[2].x);
        //float opa2=(float)(chess_knob_vector_[75].y-chess_knob_vector_[3].y)/(chess_knob_vector_[75].x-chess_knob_vector_[3].x);
        //ROS_INFO("O SYNTELESTHS EINAI %f %f %f %f",bottom_line_slope,opa,opa1,opa2);
        float side_line_slope=(float)(chess_knob_vector_[76].y-chess_knob_vector_[72].y)/(chess_knob_vector_[76].x-chess_knob_vector_[72].x);
        //ROS_INFO("O syntelesths iytthnshs einai %f sid:%f",bottom_line_slope,side_line_slope);
        int num_lines=0;bool end_case=true;
        //ROS_INFO("Ara to distance apo akrhs s akrh einai %f",(float)sqrt(pow((chess_knob_vector_[0].x-chess_knob_vector_[72].x),2)+pow((chess_knob_vector_[0].y-chess_knob_vector_[72].y),2)));
        //ROS_INFO("kai to orizontio %f",(float)sqrt(pow((chess_knob_vector_[80].x-chess_knob_vector_[72].x),2)+pow((chess_knob_vector_[80].y-chess_knob_vector_[72].y),2)));

        for(int e=0;e<chess_knob_vector_.size();e++){   //detectione of estimated points ~.~

          if(chess_knob_vector_[e].state.compare("estimated")==0){
            div_t divresult;
            divresult = div (e,9);
            ROS_INFO("thelw na vrw to estimated sth thesh %d, dld sthn grammh %d kai sthlh %d",e,divresult.quot,divresult.rem);
            divresult.quot=6;
            divresult.rem=0;
            for( size_t i = 0; i < lines.size() && end_case; i++ )
            {
              Vec4i l = lines[i];              
              float line_slope=(float)(l[3]-l[1])/(l[2]-l[0]);
              //if(l[2]-l[0]==0) continue;
              //ROS_INFO("syntelesths dieythhsnsh %f",   (float)(l[3]-l[1])/(float)(l[2]-l[0]) );

              //F0r the Vertical lines ~~

              if(area_limits[divresult.rem][0]>area_limits[divresult.rem][1]){
                if(line_slope<=area_limits[divresult.rem][0]&&line_slope>=area_limits[divresult.rem][1]){
                  //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,0 ), 3, LINE_AA);
                  v_x=l[0];
                  v_y=l[1];
                  vertical_line_sl=line_slope;  
                }
              }else{
                if(line_slope>=area_limits[divresult.rem][0]&&line_slope<=area_limits[divresult.rem][1]){
                  //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,0  ), 3, LINE_AA);
                  v_x=l[0];
                  v_y=l[1];
                  vertical_line_sl=line_slope;  
                }
              }

              //F0r the Horizontal lines ~~
              int nearest;
              if((abs(line_slope-bottom_line_slope)<=5)&&((line_slope>=0&&bottom_line_slope>=0)||(line_slope<=0&&bottom_line_slope<=0))){
                // ROS_INFO("line with coord %d %d %d %d",l[0],l[2],l[1],l[3]);
                num_lines++;
                if(num_lines==10){
                  end_case=true;//!
                }

                int x_mid=(int)(l[0]+l[2])/2;
                int y_mid=(int)(l[1]+l[3])/2;

                float point_distance=9999,point_distance_a=9999,point_distance_b=9999;
                
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
              }

              if(nearest==36+divresult.quot){
                //line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                //line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,255 ), 3, LINE_AA);            
                //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,255 ), 3, LINE_AA);
                h_x=l[0];
                h_y=l[1];
                horizontal_line_sl=line_slope;

                //vertical_line_sl is the λ' (line 2)
                int x_value=(int)( (1/(vertical_line_sl-horizontal_line_sl)* (-horizontal_line_sl*h_x+h_y+vertical_line_sl*v_x-v_y)));
                int y_value=(int)( (1/(vertical_line_sl-horizontal_line_sl)* (-vertical_line_sl*horizontal_line_sl*h_x + vertical_line_sl*h_y + horizontal_line_sl*vertical_line_sl*v_x - horizontal_line_sl*v_y))); 
                ROS_INFO("VRIKE SHMEIEO? %d %d",x_value,y_value);
                circle(src, Point(x_value,y_value), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );      
              }

            }
          }else continue;




      for( size_t i = 0; i < 0 && end_case; i++ )
      {
          Vec4i l = lines[i];
          //if(l[2]-l[0]==0) continue;
          //ROS_INFO("syntelesths dieythhsnsh %f",   (float)(l[3]-l[1])/(float)(l[2]-l[0]) );
          float line_slope=(float)(l[3]-l[1])/(l[2]-l[0]);
          //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), 3, LINE_AA); 
          
/*         if(area_limits[2][0]>area_limits[2][1]){
            if(line_slope<=area_limits[2][0]&&line_slope>=area_limits[2][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,255 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[2][0]&&line_slope<=area_limits[2][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,255 ), 3, LINE_AA);
            }
          }

          if(area_limits[3][0]>area_limits[3][1]){
            if(line_slope<=area_limits[3][0]&&line_slope>=area_limits[3][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,0 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[3][0]&&line_slope<=area_limits[3][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,0 ), 3, LINE_AA);
            }
          }

          if(area_limits[1][0]>area_limits[1][1]){
            if(line_slope<=area_limits[1][0]&&line_slope>=area_limits[1][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,255 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[1][0]&&line_slope<=area_limits[1][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,255 ), 3, LINE_AA);
            }
          }

          if(area_limits[4][0]>area_limits[4][1]){
            if(line_slope<=area_limits[4][0]&&line_slope>=area_limits[4][1]){
              //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,0 ), 3, LINE_AA);
              v_x=l[0];
              v_y=l[1];
              vertical_line_sl=line_slope;  
            }
          }else{
            if(line_slope>=area_limits[4][0]&&line_slope<=area_limits[4][1]){
              //line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,0 ), 3, LINE_AA);
              v_x=l[0];
              v_y=l[1];
              vertical_line_sl=line_slope;  
            }
          }

          if(area_limits[5][0]>area_limits[5][1]){
            if(line_slope<=area_limits[5][0]&&line_slope>=area_limits[5][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,255,0 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[5][0]&&line_slope<=area_limits[5][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,255,0 ), 3, LINE_AA);
            }
          }

          if(area_limits[6][0]>area_limits[6][1]){
            if(line_slope<=area_limits[6][0]&&line_slope>=area_limits[6][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,255,255 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[6][0]&&line_slope<=area_limits[6][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,255,255), 3, LINE_AA);
            }
          }

          if(area_limits[7][0]>area_limits[7][1]){
            if(line_slope<=area_limits[7][0]&&line_slope>=area_limits[7][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 125,0,0 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[7][0]&&line_slope<=area_limits[7][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 125,0,0 ), 3, LINE_AA);
            }
          }

          if(area_limits[8][0]>area_limits[8][1]){
            if(line_slope<=area_limits[8][0]&&line_slope>=area_limits[8][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,125,0 ), 3, LINE_AA);
            }
          }else{
            if(line_slope>=area_limits[8][0]&&line_slope<=area_limits[8][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,125,0  ), 3, LINE_AA);
            }
          }*/

          if(area_limits[0][0]>area_limits[0][1]){
            if(line_slope<=area_limits[0][0]&&line_slope>=area_limits[0][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,0 ), 3, LINE_AA);
                            v_x=l[0];
              v_y=l[1];
              vertical_line_sl=line_slope;  
            }
          }else{
            if(line_slope>=area_limits[0][0]&&line_slope<=area_limits[0][1]){
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,0,0  ), 3, LINE_AA);
                            v_x=l[0];
              v_y=l[1];
              vertical_line_sl=line_slope;  
            }
          }



          if((abs(line_slope-bottom_line_slope)<=5)&&((line_slope>=0&&bottom_line_slope>=0)||(line_slope<=0&&bottom_line_slope<=0))){
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
       /*     if(nearest==36){
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
              //line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 

//              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,255 ), 3, LINE_AA);
              h_x=l[0];
              h_y=l[1];
              horizontal_line_sl=line_slope;    
                    //vertical_line_sl is the λ' (line 2)
      int x_value=(int)( (1/(vertical_line_sl-horizontal_line_sl)* (-horizontal_line_sl*h_x+h_y+vertical_line_sl*v_x-v_y)));
      int y_value=(int)( (1/(vertical_line_sl-horizontal_line_sl)* (-vertical_line_sl*horizontal_line_sl*h_x + vertical_line_sl*h_y + horizontal_line_sl*vertical_line_sl*v_x - horizontal_line_sl*v_y))); 
      ROS_INFO("VRIKE SHMEIEO? %d %d",x_value,y_value);
      circle(src, Point(x_value,y_value), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );           
            }else */if(nearest==41){
              line( src, Point(l[0],l[1]), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
                            //line( src, Point(x_mid,y_mid), Point( chess_knob_vector_[nearest].x,chess_knob_vector_[nearest].y),Scalar(255,0,0), 3, LINE_AA); 
            
              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 255,0,255 ), 3, LINE_AA);            
              //              line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( 0,255,255 ), 3, LINE_AA);
              h_x=l[0];
              h_y=l[1];
              horizontal_line_sl=line_slope;    
                    //vertical_line_sl is the λ' (line 2)
      int x_value=(int)( (1/(vertical_line_sl-horizontal_line_sl)* (-horizontal_line_sl*h_x+h_y+vertical_line_sl*v_x-v_y)));
      int y_value=(int)( (1/(vertical_line_sl-horizontal_line_sl)* (-vertical_line_sl*horizontal_line_sl*h_x + vertical_line_sl*h_y + horizontal_line_sl*vertical_line_sl*v_x - horizontal_line_sl*v_y))); 
      ROS_INFO("VRIKE SHMEIEO? %d %d",x_value,y_value);
      circle(src, Point(x_value,y_value), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );      
            }/*else if(nearest==42){
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
          else if(abs(line_slope-side_line_slope)<=1){
            line( src, Point(l[0], l[1]), Point(l[2], l[3]),Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), 3, LINE_AA);
          }*/
      }

    //ROS_INFO("%d filtered lines",num_lines);
}
}
}

      //ROS_INFO("the result is %f",(1/(hor)))
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

