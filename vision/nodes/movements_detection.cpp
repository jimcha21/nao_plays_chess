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

Mat src, src_gray;;
std::vector<cv::Point> chess_topic_points;
RNG rng(12345);

int frames_num=0;
bool debug_mode=false;
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
          snapshot_sub = nh_.subscribe("take_snaps", 10, &ImageConverter::Snapshot, this); //chessboard subscriber
          image_sub_.shutdown();
          //image_pub_ = it_.advertise("/image_converter/output_video", 1);
          cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
          cv::destroyWindow(OPENCV_WINDOW);
  }

/////////////////////////

  void Snapshot(const std_msgs::Bool& snap){
    
    ROS_INFO("snapdata: %s", snap.data ? "true" : "false");
    if(snap.data){
       //image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::movementDetector, this);    
       image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::movementDetector, this);    
       ROS_INFO("IRRTHE ?");   
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
      //cvtColor( src, src_gray, COLOR_BGR2GRAY ); 
      
 if(chess_topic_points.size()>0&&snap.data){ 

      
      ROS_INFO("les go..");

      image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::imageCb, this);
          

      for(int ch_line=1;ch_line<=2;ch_line++){
          for(int ch_column=1;ch_column<=8;ch_column++){
          //ROS_INFO("ara ta prohgoumena einai %f %f %f",squareDensity[ch_line-1][ch_column-1][0],squareDensity[ch_line-1][ch_column-1][1],squareDensity[ch_line-1][ch_column-1][2]);

          //choosing a chess square ..
          int *sq_points;
          sq_points=square_CornPoints(/*"a",*/ch_column,ch_line); //added decimal (int,int) square identification.. // 
         //ROS_INFO("points %d %d %d %d",sq_points[0],sq_points[1],sq_points[2],sq_points[3]);


    /*      int diste[4][2];
          diste[0][0]=(int)sqrt(pow(chess_topic_points[sq_points[0]].x,2)+pow(chess_topic_points[sq_points[0]].y,2));
          diste[0][1]=sq_points[0];
          diste[1][0]=(int)sqrt(pow(chess_topic_points[sq_points[1]].x,2)+pow(chess_topic_points[sq_points[1]].y,2));
          diste[1][1]=sq_points[1];
          diste[2][0]=(int)sqrt(pow(chess_topic_points[sq_points[2]].x,2)+pow(chess_topic_points[sq_points[2]].y,2));
          diste[2][1]=sq_points[2];
          diste[3][0]=(int)sqrt(pow(chess_topic_points[sq_points[3]].x,2)+pow(chess_topic_points[sq_points[3]].y,2));
          diste[3][1]=sq_points[3];
          ROS_INFO("distance %d %d - %d %d - %d %d - %d %d",diste[0][1],diste[0][0],diste[1][1],diste[1][0],diste[2][1],diste[2][0],diste[3][1],diste[3][0]);

          //bubblesorting the point distances
          int swap;
          for(intc=0;c<3;c++){
            for(int d=0;d<4-c-1;d++){
              if (diste[d][0] > diste[d+1][0]) // For decreasing order use <
              {
                swap       = diste[d][0];
                diste[d][0]   = diste[d+1][0];
                diste[d+1][0] = swap;
                swap       = diste[d][1];
                diste[d][1]   = diste[d+1][1];
                diste[d+1][1] = swap;
              }
            }
          }
          ROS_INFO("SORTED distance %d %d - %d %d - %d %d - %d %d",diste[0][1],diste[0][0],diste[1][1],diste[1][0],diste[2][1],diste[2][0],diste[3][1],diste[3][0]);
       */

          CvPoint left_bottom_p,left_top_p,right_top_p,right_bottom_p;
          std::vector<CvPoint> v;
          left_bottom_p.x=chess_topic_points[sq_points[0]].x;
          left_bottom_p.y=chess_topic_points[sq_points[0]].y;
          v.push_back(left_bottom_p);
          left_top_p.x=chess_topic_points[sq_points[1]].x;
          left_top_p.y=chess_topic_points[sq_points[1]].y;
          v.push_back(left_top_p);
          right_top_p.x=chess_topic_points[sq_points[3]].x;
          right_top_p.y=chess_topic_points[sq_points[3]].y;
          v.push_back(right_top_p);
          right_bottom_p.x=chess_topic_points[sq_points[2]].x;
          right_bottom_p.y=chess_topic_points[sq_points[2]].y;
          v.push_back(right_bottom_p);

          //int a=(float)(right_top_p.y-left_top_p.y)/(float)(right_top_p.x-left_top_p.x);

          //calculation of 4 lines' slope parameters , for line contruction.. 
          float lamda1 =(float)(left_bottom_p.y-left_top_p.y)/(float)(left_bottom_p.x-left_top_p.x);
          float lamda2 =(float)(right_bottom_p.y-right_top_p.y)/(float)(right_bottom_p.x-right_top_p.x);
          float lamda3 =(float)(right_bottom_p.y-left_bottom_p.y)/(float)(right_bottom_p.x-left_bottom_p.x);
          float lamda4 =(float)(right_top_p.y-left_top_p.y)/(float)(right_top_p.x-left_top_p.x);
          //ROS_INFO("shmeia %f %f %f %f",lamda1,lamda2,lamda3,lamda4);
         
    //!include pls extreme case of total 90 degree rotation-> in this situation you must change the point arrangement 

          //estimation of the examination rectangle, which will be filtered by the 4 line equations below..
          int min_x=9999,max_x=0,min_y=9999,max_y=0;
          for(int i=0;i<v.size();i++){
              if(v[i].x<min_x){
                min_x=v[i].x;
              }
              if(v[i].y<min_y){
                min_y=v[i].y;
              }
              if(v[i].x>max_x){
                max_x=v[i].x;
              }
              if(v[i].y>max_y){
                max_y=v[i].y;
              }
          }//ROS_INFO("max x %d max y %d min x %d min y %d",max_x,max_y,min_x,min_y);

          int areaIn_pixels=0;
          int whites=0;
          int blacks=0;
          int unenti=0;


          for(int x=min_x;x<=max_x;x++){
              if(x>src.cols){
                continue;
              }
            for(int y=min_y;y<=max_y;y++){
              if(y>src.rows){
                continue;
              }
              if(/*(y>=(int)(lamda1*(x-left_bottom_p.x)+left_bottom_p.y)) &&*/ (x>=(int)(y-left_bottom_p.y+lamda1*left_bottom_p.x)/lamda1)){
                if(/*(y<=(int)(lamda2*(x-right_bottom_p.x)+right_bottom_p.y))&&*/(x<=(int)(y-right_bottom_p.y+lamda2*right_bottom_p.x)/lamda2)){
                  if((y<=(int)(lamda3*(x-right_bottom_p.x)+right_bottom_p.y))/*&&(x>=(int)(y-right_bottom_p.y+lamda3*right_bottom_p.x)/lamda3)*/){
                    if((y>=(int)(lamda4*(x-right_top_p.x)+right_top_p.y))/*&&(x<=(int)(y-right_top_p.y+lamda4*right_top_p.x)/lamda4)*/){
                      areaIn_pixels++;
                      //THIS IS THE AREA THAT WILL BE EXAMINED
                      if(src.at<cv::Vec3b>(y,x)[0]<=60&&src.at<cv::Vec3b>(y,x)[1]<=60&&src.at<cv::Vec3b>(y,x)[2]<=60){
                        blacks++;
                      }else if(src.at<cv::Vec3b>(y,x)[0]>=190&&src.at<cv::Vec3b>(y,x)[1]>=190&&src.at<cv::Vec3b>(y,x)[2]>=190){
                        whites++;
                      }else{
                        unenti++;
                      }

                      //for checking area inspection..
                      src.at<cv::Vec3b>(y,x)[0] = 255;
                      src.at<cv::Vec3b>(y,x)[1] = 255;
                      src.at<cv::Vec3b>(y,x)[2] = 255;
              }}}}
            }
          }
          if(areaIn_pixels!=0){
            //ROS_INFO("EXAMINED AN AREA OF %d PIXELS -> BLACKS-> %f WHITES-> %f unenti-> %f",areaIn_pixels,(float)blacks/areaIn_pixels,(float)whites/areaIn_pixels,(float)unenti/areaIn_pixels);
           // if(!(squareDensity[0][0][0]==0&&squareDensity[0][0][1]==0&&squareDensity[0][0][2]==0)){
            float black_diff=0;
            float white_diff=0;
            float undi_diff=0;
            black_diff = (float)abs(squareDensity[ch_line-1][ch_column-1][0]-(float)blacks/areaIn_pixels);
            white_diff = (float)abs(squareDensity[ch_line-1][ch_column-1][1]-(float)whites/areaIn_pixels);
            undi_diff = (float)abs(squareDensity[ch_line-1][ch_column-1][2]-(float)unenti/areaIn_pixels);
            ROS_INFO("b=%f w=%f u=%f",black_diff,white_diff,undi_diff);
            if(black_diff>0.60){
              ROS_INFO("yparxei diafora sta black! sto %d %d = %f ", ch_line, ch_column,abs(squareDensity[ch_line-1][ch_column-1][0]-(float)blacks/areaIn_pixels));
            }
            if(white_diff>0.60){
              ROS_INFO("yparxei diafora sta white! sto %d %d = %f ", ch_line, ch_column,abs(squareDensity[ch_line-1][ch_column-1][1]-(float)whites/areaIn_pixels));
            }
            if(undi_diff>0.60){
              ROS_INFO("yparxei diafora sta unid! sto %d %d = %f ", ch_line, ch_column,abs(squareDensity[ch_line-1][ch_column-1][2]-(float)unenti/areaIn_pixels));
            }
            //}
            squareDensity[ch_line-1][ch_column-1][0]=(float)blacks/areaIn_pixels;
            squareDensity[ch_line-1][ch_column-1][1]=(float)whites/areaIn_pixels;
            squareDensity[ch_line-1][ch_column-1][2]=(float)unenti/areaIn_pixels;        
          }
          v.clear();
        
      }}
    }
      
/*      for( int j = 0; j < chess_topic_points.size(); j++ ){
          circle(src, Point(chess_topic_points[j].x,chess_topic_points[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("she %d -> %d %d",chess_topic_points[j].x,chess_topic_points[j].y);
      }*/
      cv::imshow(OPENCV_WINDOW,src);
      cv::waitKey(3);

      chess_topic_points.clear();
      image_sub_.shutdown();
      return ;
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