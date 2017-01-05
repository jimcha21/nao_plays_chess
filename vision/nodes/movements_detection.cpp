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
#include "vision/ChessboardSquare.h"
#include "vision/ChessBoard.h"
#include "vision/ChessPiece.h"
#include "vision/ChessPiecesVector.h"


using namespace cv;
using namespace std;

// Variables for debugging.. ignore ..
bool view_mode_debug=false;
bool picture_mode_debug=false;

Mat src, src_gray;
Mat snap_one,snap_two;

std::vector<std::string> visited_v;

vision::ChessBoard game_;
//std::vector<cv::Point> chess_topic_points;
vision::ChessVector chess_topic_points;
vision::ChessPiecesVector pieces_topic_points;

RNG rng(12345);

int frame_received=0;
bool enable_snap=false;
float frameDensity[8][8][3];
float squareDensity[8][8][3]; //0 for black, 1 for white , 2 for und

static const std::string OPENCV_WINDOW = "wpa";
std::vector<int> first_team;

/// Function headers
int *square_CornPoints(/*std::string str*/int int_letter ,int num);
bool checkifItsInsidetheSquare(CvPoint,int,int);
void colorMovement(Mat img,CvPoint start_pos,CvPoint end_pos,bool seq);
void recursive_flagCheck(int index,std::vector<vision::ChessPoint> flag_square,float scan_area);

///////////////////////////////////////////////////////////////////////////

void chessboardVectorTopic(const vision::ChessVector& data){
  //cv::Point pt;
  chess_topic_points.p_vector.clear();
  for(int i=0;i<data.p_vector.size();i++){ //till 81..
    chess_topic_points.p_vector.push_back(data.p_vector[i]);
  }
}

void gameTopic(const vision::ChessBoard& data){
  //vision::ChessboardSquare sq;

  ROS_INFO("received game");
  game_.chessSquare.clear();
  for(int i=0;i<data.chessSquare.size();i++){ //till 64..
    game_.chessSquare.push_back(data.chessSquare[i]);
  }
}

void piecesVectorTopic(const vision::ChessPiecesVector& data){
  //vision::ChessboardSquare sq;
  //ROS_INFO("received pieces kai data ssize %d",data.p_vector.size());
  pieces_topic_points.p_vector.clear();
  for(int i=0;i<data.p_vector.size();i++){ //till 64..
    pieces_topic_points.p_vector.push_back(data.p_vector[i]);
  }
}

/////////////////////////

class ImageConverter
{
  ros::NodeHandle nh_;
  ros::Subscriber chess_sub;  
  ros::Subscriber pieces_sub;  
  ros::Subscriber game_sub;
  ros::Subscriber snapshot_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
          // Subscrive to input video feed and publish output video feed
          game_sub = nh_.subscribe("/Game_Menu_node/chessboard_state", 10, gameTopic); //game status subscriber
          chess_sub = nh_.subscribe("hough_mapped_chessboard_knob_coordinates", 10, chessboardVectorTopic); //chessboard subscriber
          pieces_sub = nh_.subscribe("/chessboard_estimated_areas_ofPieces", 10, piecesVectorTopic); //pieces areas subscriber


          if(!view_mode_debug){    
             snapshot_sub = nh_.subscribe("take_snaps", 10, &ImageConverter::Snapshot, this); //chessboard subscriber
          }else{          
            //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::movementDetector, this);       
            //image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::movementDetector, this);   
             image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::imagePreview, this); 
          }

          //image_sub_.shutdown();
          //image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }

  ~ImageConverter()
  {
          cv::destroyWindow(OPENCV_WINDOW);
  }

  void imagePreview(const sensor_msgs::ImageConstPtr& msg)
  {

      cv_bridge::CvImagePtr cv_ptr;
      //ROS_INFO("recevied ?%d",chess_topic_points.p_vector.size());
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

      //cvtColor( cv_ptr->image, src_gray, COLOR_BGR2GRAY );      
      
      if(true){
        src=cv_ptr->image.clone();
      }else{
        src = imread("src/vision/src/data/3.jpg", 1);
        if(src.empty())
        {
          ROS_INFO("can not open the image");// << filename << endl;
          return ;
        }
      }
      if(chess_topic_points.p_vector.size()>0) 
      {
        for(int linee=0;linee<8;linee++){
          for(int column=0;column<8;column++){
          int  hello=8*column+linee;
          if(pieces_topic_points.p_vector[hello].category.compare("empty")!=0){

            for (int i = 0; i < 8; i++)
            {
              for (int j = 0; j < 8; j++)
              {
                if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[hello].e.x,pieces_topic_points.p_vector[hello].e.y),i+1,j+1)||(i==7&&j==7)){
                   ROS_INFO("geia to %d BRIKE TH SKIA TOU STO SHMEIO %d %d!",hello,i,j);
                   ROS_INFO("kai prepei na paei sto shmeio %d %d!",linee,column);
                   bool end=false;
                   int start_x=i,start_y=j;
                   do{
                      ROS_INFO("HEHE GEIA TO %d %d!",start_x,start_y);
                      if(start_x!=linee){//not same row
                        //aproaching
                        if(start_x>linee){
                          start_x--;
                        }//else start_x++; //never gonna happen
                      }
                      if(start_y!=column){
                        if(start_y>column){
                          start_y--;
                        }else{
                          start_y++;
                        }
                      }
                      if(start_y==column&&start_x==linee){
                        end=true;
                      }
                   }while(!end);
                }
              }


              }
      //checkifItsInsidetheSquare(CvPoint(92,283),1,1);
        } }}}
      //cvtColor( src, src_gray, COLOR_BGR2GRAY ); 


      
      for( int j = 0; j < chess_topic_points.p_vector.size(); j++ ){
        circle(src, Point(chess_topic_points.p_vector[j].x,chess_topic_points.p_vector[j].y), 4, Scalar( 0,255,0 ), -1, 8, 0 );    
        //ROS_INFO("she %d -> %d %d",chess_featured_points_[j].x,chess_featured_points_[j].y);
      }  

      //ROS_INFO("game size %d pieces size %d",game_.chessSquare.size(),pieces_topic_points.p_vector.size());
      for( int j = 0; j < game_.chessSquare.size(); j++ ){
        if(pieces_topic_points.p_vector[j].category.compare("empty")!=0){          
          line(src,Point(pieces_topic_points.p_vector[j].a.x,pieces_topic_points.p_vector[j].a.y),Point(pieces_topic_points.p_vector[j].b.x,pieces_topic_points.p_vector[j].b.y), Scalar(255,0,0),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].a.x,pieces_topic_points.p_vector[j].a.y),Point(pieces_topic_points.p_vector[j].c.x,pieces_topic_points.p_vector[j].c.y), Scalar(0,255,0),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].c.x,pieces_topic_points.p_vector[j].c.y),Point(pieces_topic_points.p_vector[j].d.x,pieces_topic_points.p_vector[j].d.y), Scalar(0,0,255),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].b.x,pieces_topic_points.p_vector[j].b.y),Point(pieces_topic_points.p_vector[j].d.x,pieces_topic_points.p_vector[j].d.y), Scalar(255,0,255),1, LINE_AA);

          line(src,Point(pieces_topic_points.p_vector[j].e.x,pieces_topic_points.p_vector[j].e.y),Point(pieces_topic_points.p_vector[j].f.x,pieces_topic_points.p_vector[j].f.y), Scalar(255,0,0),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].e.x,pieces_topic_points.p_vector[j].e.y),Point(pieces_topic_points.p_vector[j].g.x,pieces_topic_points.p_vector[j].g.y), Scalar(0,255,0),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].f.x,pieces_topic_points.p_vector[j].f.y),Point(pieces_topic_points.p_vector[j].h.x,pieces_topic_points.p_vector[j].h.y), Scalar(0,0,255),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].g.x,pieces_topic_points.p_vector[j].g.y),Point(pieces_topic_points.p_vector[j].h.x,pieces_topic_points.p_vector[j].h.y), Scalar(255,0,25),1, LINE_AA);

          line(src,Point(pieces_topic_points.p_vector[j].a.x,pieces_topic_points.p_vector[j].a.y),Point(pieces_topic_points.p_vector[j].e.x,pieces_topic_points.p_vector[j].e.y), Scalar(255,0,0),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].b.x,pieces_topic_points.p_vector[j].b.y),Point(pieces_topic_points.p_vector[j].f.x,pieces_topic_points.p_vector[j].f.y), Scalar(0,255,0),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].c.x,pieces_topic_points.p_vector[j].c.y),Point(pieces_topic_points.p_vector[j].g.x,pieces_topic_points.p_vector[j].g.y), Scalar(0,0,255),1, LINE_AA);
          line(src,Point(pieces_topic_points.p_vector[j].d.x,pieces_topic_points.p_vector[j].d.y),Point(pieces_topic_points.p_vector[j].h.x,pieces_topic_points.p_vector[j].h.y), Scalar(255,0,255),1, LINE_AA);
        }
      }

      
      cv::imshow("movementDetector final image for processing", src);
     cv::waitKey(3);
}


/////////////////////////

  void Snapshot(const std_msgs::Bool& snap){
    
    ROS_INFO("snapdata: %s", snap.data ? "true" : "false");
    //ROS_INFO("received %d pieces with the last one a.x,a.y : %d %d",pieces_topic_points.p_vector.size(),pieces_topic_points.p_vector[63].a.x,pieces_topic_points.p_vector[63].a.y);

    if(snap.data){
       image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::movementDetector, this);    
       enable_snap=snap.data;
       //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::movementDetector, this);    
    }

    if(frame_received==2){
      cv::imshow("snap one",snap_one);
      cv::imshow("snap two",snap_two);
/*   

      //sub the two images
      Mat new_image3(snap_one.size(), snap_one.type());
      //new_image3 = new_image2 - new_image1;
      cv::absdiff(snap_one, snap_two, new_image3);
      cv::imshow("image2-das", new_image3);
      inRange(new_image3, cv::Scalar(30,0,0), cv::Scalar(255,255,255 ), new_image3); //BGR Scalar not RGB sequence
      cv::imshow("end", new_image3);*/


     cv::waitKey(3);
    }
  }

/////////////////////////

  void movementDetector(const sensor_msgs::ImageConstPtr& msg)
  {
      cv_bridge::CvImagePtr cv_ptr;

      if(chess_topic_points.p_vector.size()==0){
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
        Mat new_image(src.size(), src.type());
        //new_image3 = new_image2 - new_image1;
        cv::absdiff(snap_one, snap_two, new_image);
        cv::imshow("difference", new_image);

        Mat ba(src.size(), src.type());
        ba = snap_one - snap_two;
        cv::imshow("difference ba", ba);
        inRange(ba, cv::Scalar(10,10,10), cv::Scalar(255,255,255 ), ba); //BGR Scalar not RGB sequence
        cv::imshow("range ba", ba);

        Mat ab(src.size(), src.type());
        ab =snap_two-snap_one;
        cv::imshow("difference ab", ab);
        inRange(ab, cv::Scalar(10,10,10), cv::Scalar(255,255,255 ), ab); //BGR Scalar not RGB sequence
        cv::imshow("range ab", ab);

        Mat range_img,new_image3;
        inRange(new_image, cv::Scalar(10,10,10), cv::Scalar(255,255,255 ), range_img); //BGR Scalar not RGB sequence

        cv::imshow("ranged", range_img);
        medianBlur ( range_img, new_image3, 3 );
        cv::imshow("color filtered in range", new_image3);

        //cvtColor( src, src_gray, COLOR_BGR2GRAY );

       
        std::vector<vision::ChessPoint> flag_square; // this vector will used as flagged index , pointing the spotted differences that are detected on any of the 64 chessboard squares. -detection part follows- 
      
       if(chess_topic_points.p_vector.size()>0){ 

            //image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::imageCb, this);
                

            for(int ch_line=1;ch_line<=8;ch_line++){
                for(int ch_column=1;ch_column<=8;ch_column++){
                //ROS_INFO("ara ta prohgoumena einai %f %f %f",squareDensity[ch_line-1][ch_column-1][0],squareDensity[ch_line-1][ch_column-1][1],squareDensity[ch_line-1][ch_column-1][2]);

                //choosing a chess square ..
                int *sq_points;
                sq_points=square_CornPoints(/*"a",*/ch_column,ch_line); //added decimal (int,int) square identification.. // 
               //ROS_INFO("points %d %d %d %d",sq_points[0],sq_points[1],sq_points[2],sq_points[3]);


          /*      int diste[4][2];
                diste[0][0]=(int)sqrt(pow(chess_topic_points.p_vector[sq_points[0]].x,2)+pow(chess_topic_points.p_vector[sq_points[0]].y,2));
                diste[0][1]=sq_points[0];
                diste[1][0]=(int)sqrt(pow(chess_topic_points.p_vector[sq_points[1]].x,2)+pow(chess_topic_points.p_vector[sq_points[1]].y,2));
                diste[1][1]=sq_points[1];
                diste[2][0]=(int)sqrt(pow(chess_topic_points.p_vector[sq_points[2]].x,2)+pow(chess_topic_points.p_vector[sq_points[2]].y,2));
                diste[2][1]=sq_points[2];
                diste[3][0]=(int)sqrt(pow(chess_topic_points.p_vector[sq_points[3]].x,2)+pow(chess_topic_points.p_vector[sq_points[3]].y,2));
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
                left_bottom_p.x=chess_topic_points.p_vector[sq_points[0]].x;
                left_bottom_p.y=chess_topic_points.p_vector[sq_points[0]].y;
                v.push_back(left_bottom_p);
                left_top_p.x=chess_topic_points.p_vector[sq_points[1]].x;
                left_top_p.y=chess_topic_points.p_vector[sq_points[1]].y;
                v.push_back(left_top_p);
                right_top_p.x=chess_topic_points.p_vector[sq_points[3]].x;
                right_top_p.y=chess_topic_points.p_vector[sq_points[3]].y;
                v.push_back(right_top_p);
                right_bottom_p.x=chess_topic_points.p_vector[sq_points[2]].x;
                right_bottom_p.y=chess_topic_points.p_vector[sq_points[2]].y;
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
                    if(x>new_image3.cols){
                      continue;
                    }
                    for(int y=min_y;y<=max_y;y++){
                      if(y>new_image3.rows){
                        continue;
                      }
                      if(/*(y>=(int)(lamda1*(x-left_bottom_p.x)+left_bottom_p.y)) &&*/ (x>=(int)(y-left_bottom_p.y+lamda1*left_bottom_p.x)/lamda1)){
                        if(/*(y<=(int)(lamda2*(x-right_bottom_p.x)+right_bottom_p.y))&&*/(x<=(int)(y-right_bottom_p.y+lamda2*right_bottom_p.x)/lamda2)){
                          if((y<=(int)(lamda3*(x-right_bottom_p.x)+right_bottom_p.y))/*&&(x>=(int)(y-right_bottom_p.y+lamda3*right_bottom_p.x)/lamda3)*/){
                            if((y>=(int)(lamda4*(x-right_top_p.x)+right_top_p.y))/*&&(x<=(int)(y-right_top_p.y+lamda4*right_top_p.x)/lamda4)*/){
                              areaIn_pixels++;
                              
                              //FOR CV_32F IMAGE (COLOR IMAGE LIKE SOURCE)
                             /*//THIS IS THE AREA THAT WILL BE EXAMINED
                              if(new_image3.at<cv::Vec3b>(y,x)[0]<=60&&new_image3.at<cv::Vec3b>(y,x)[1]<=60&&new_image3.at<cv::Vec3b>(y,x)[2]<=60){
                                blacks++;
                              }else if(new_image3.at<cv::Vec3b>(y,x)[0]>=190&&new_image3.at<cv::Vec3b>(y,x)[1]>=190&&new_image3.at<cv::Vec3b>(y,x)[2]>=190){
                                whites++;
                              }else{
                                unenti++;
                              }

                              //for checking area inspection..
                            new_image3.at<cv::Vec3b>(y,x)[0] = 255;
                              new_image3.at<cv::Vec3b>(y,x)[1] = 255;
                              new_image3.at<cv::Vec3b>(y,x)[2] = 255;*/

                              
                              //FOR CV_8U IMAGE (BINARY LIKE IMAGE - RESULT FROM IMRANGE - NO RGB[][][] ONLY ONE VALUE PER PIXEL=0 OR 255)
                              //THIS IS THE AREA THAT WILL BE EXAMINED
                              if(new_image3.at<uchar>(y, x)==0){
                                blacks++;
                              }else if(new_image3.at<uchar>(y, x)==255){
                                whites++;
                              }

                              //for checking area inspection..
                              //new_image3.at<uchar>(y, x)=255;
                            }
                          }
                        }
                      }
                    }
                }

                if(areaIn_pixels!=0){
                  //ROS_INFO("EXAMINED AN AREA OF %d PIXELS -> BLACKS-> %f WHITES-> %f unenti-> %f",areaIn_pixels,(float)blacks/areaIn_pixels,(float)whites/areaIn_pixels,(float)unenti/areaIn_pixels);
        

// this for comparing two in a row snaps in specific colour choices.. new_image3 <> src
      /*            if(frame_received>1){
                      ROS_INFO("-----------------------------------------------");
                      float black_diff=0;
                      float white_diff=0;
                      float undi_diff=0;
                      black_diff = (float)abs(squareDensity[ch_line-1][ch_column-1][0]-(float)blacks/areaIn_pixels);
                      white_diff = (float)abs(squareDensity[ch_line-1][ch_column-1][1]-(float)whites/areaIn_pixels);
                      undi_diff = (float)abs(squareDensity[ch_line-1][ch_column-1][2]-(float)unenti/areaIn_pixels);
                      ROS_INFO("b=%f w=%f u=%f",black_diff,white_diff,undi_diff);
                      if(black_diff>0.60){
                        ROS_INFO("yparxei diafora sta black! sto %d %d = %f ", ch_line, ch_column,abs(squareDensity[ch_line-1][ch_column-1][0]-(float)blacks/areaIn_pixels));
                      }else{
                        ROS_INFO("no diff in black.");
                      }
                      if(white_diff>0.60){
                        ROS_INFO("yparxei diafora sta white! sto %d %d = %f ", ch_line, ch_column,abs(squareDensity[ch_line-1][ch_column-1][1]-(float)whites/areaIn_pixels));
                      }else{
                        ROS_INFO("no diff in white.");
                      }
                      if(undi_diff>0.60){
                        ROS_INFO("yparxei diafora sta unid! sto %d %d = %f ", ch_line, ch_column,abs(squareDensity[ch_line-1][ch_column-1][2]-(float)unenti/areaIn_pixels));
                      }else{
                        ROS_INFO("no diff in unint.");
                      }
                      ROS_INFO("-----------------------------------------------");
                  }*/

                  squareDensity[ch_line-1][ch_column-1][0]=(float)blacks/areaIn_pixels;
                  squareDensity[ch_line-1][ch_column-1][1]=(float)whites/areaIn_pixels;
                  //squareDensity[ch_line-1][ch_column-1][2]=(float)unenti/areaIn_pixels;    
                  ROS_INFO("%d-%d:Colour amounts %f %f %f ",ch_line,ch_column,squareDensity[ch_line-1][ch_column-1][0],squareDensity[ch_line-1][ch_column-1][1],squareDensity[ch_line-1][ch_column-1][2]);    
                  
                  vision::ChessPoint cp;
                  if(squareDensity[ch_line-1][ch_column-1][0]<0.75){  //red flag (really positive of a movement)
                    ROS_INFO("\033[1;31mDETECTED!\033[0m\n");
                    cp.x=ch_line-1; cp.y=ch_column-1; cp.state="confident";
                    flag_square.push_back(cp);
                  }
                  else if(squareDensity[ch_line-1][ch_column-1][0]<0.85){ //yellow flag (really positive of a movement)
                    ROS_INFO("\033[1;33mSOMETHING HERE?!\033[0m\n");
                    cp.x=ch_line-1; cp.y=ch_column-1; cp.state="cautious";
                    flag_square.push_back(cp);
                  }

                  ROS_INFO("-----------------------------------------------");
                }
                v.clear();
                //cv::imshow(OPENCV_WINDOW,new_image3);
                cv::waitKey(3);
            }
          }
          
          for (int i = 0; i < flag_square.size(); i++)
          {
            ROS_INFO("\033[1;35mFLAG ARAY %d %d %s!\033[0m\n",flag_square[i].x+1,flag_square[i].y+1,flag_square[i].state.c_str());
            visited_v.push_back("n"); //n for no, y for yes ... initialization
          }

          
          first_team.clear();
          first_team.push_back(0);
          recursive_flagCheck(0,flag_square,sqrt(2)); //int scan_area=sqrt(2)
          visited_v.clear();

          if(first_team.size()==1){ //make a bigger scan
            for (int i = 0; i < flag_square.size(); i++)
            {
              visited_v.push_back("n"); //n for no, y for yes ... initialization
            }
            recursive_flagCheck(0,flag_square,2*sqrt(2)); //int scan_area=2*sqrt(2)
          }

          visited_v.clear();
          std::vector<int> second_team;
          for (int i = 0; i < flag_square.size(); ++i) //to find the second 'team' points..
          {
            
            bool belongs=false; 
            for (int j = 0; j < first_team.size(); j++)
            {
              if(i==first_team[j]){
                belongs=true;
                break;
              }
            }

            if(!belongs){
              second_team.push_back(i);
              ROS_INFO("h omada 2 exei %d stoixeia kai to ena einai to %d",second_team.size(),i);
            }
          }

          for (int i = 0; i < first_team.size(); ++i)
          {
            ROS_INFO("h omada exei %d stoixeia kai to ena einai to %d",first_team.size(),first_team[i]);
          }


          //checking the flagged squares..

/*          int min=7,frontline_idx=7; //init max line
          for (int i = 0; i < flag_square.size(); ++i) 
          {
            if(flag_square[i].x<min){
              min=flag_square[i].x;
              frontline_idx=i;
            }
            ROS_INFO("found the %d %d with state %s",flag_square[i].x+1,flag_square[i].y+1,flag_square[i].state.c_str());
          }
          if(flag_square.size()>0){
            ROS_INFO("%s piece",pieces_topic_points.p_vector[flag_square[frontline_idx].x+8*flag_square[frontline_idx].y].category.c_str());
            ROS_INFO("min is %d on vector pos %d and it is going to check on square %d",min,frontline_idx,(flag_square[frontline_idx].x+8*flag_square[frontline_idx].y));
            if(pieces_topic_points.p_vector[flag_square[frontline_idx].x+8*flag_square[frontline_idx].y].category.compare("empty")!=0){
              ROS_INFO("brike me sigouria to minimun %s poy vriskotan sth thesh %d %d",pieces_topic_points.p_vector[flag_square[frontline_idx].x+8*flag_square[frontline_idx].y].category.c_str(),flag_square[frontline_idx].x+1,flag_square[frontline_idx].y+1);

            }
          }*/

          int square_num=56;
          int column=7,linee=0;
          bool found_first=false;
          bool found_second=false;
          bool yeap=false;
          CvPoint first,second;
          do{
            square_num=column*8+linee;
            for (int i = 0; i < flag_square.size(); i++){
              if(flag_square[i].x==linee&&flag_square[i].y==column&&!found_first){
                found_first=true;
                ROS_INFO("found the first piece at %d %d",linee+1,column+1);
                first.x=linee;
                first.y=column;
                if(pieces_topic_points.p_vector[square_num].category.compare("empty")==0){
                  ROS_INFO("\033[1;31mFOUND LAST PIECE POSITION!\033[0m\n");
                  yeap=true;
                }else{
                  ROS_INFO("\033[1;31mFOUND ONE OF THE TWO POSITIONS AND I KNOW THAT HERE WAS A %s\033[0m\n",pieces_topic_points.p_vector[square_num].category.c_str());
                  
                  for (int ii = 0; ii < 8; ii++)
                  {
                    for (int jj = 0; jj < 8; jj++)
                    {
                      //for e
                      if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].e.x,pieces_topic_points.p_vector[square_num].e.y),ii+1,jj+1)){
                         ROS_INFO("gia to %d BRIKE TH SKIA TOU STO SHMEIO %d %d!",square_num,ii,jj);

                         //ROS_INFO("kai prepei na paei sto shmeio %d %d!",linee,column);
                         bool end=false;
                         int start_x=ii,start_y=jj;
                         do{
                            ROS_INFO("HEHE GEIA TO %d %d!",start_x+1,start_y+1);
                            for (int f = 0; f < flag_square.size(); f++){
                              if(start_x==flag_square[f].x&&start_y==flag_square[f].y){
                                flag_square[f].state="ignore"; //propably it's a piece's shadow..
                                ROS_INFO("\033[1;32mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[f].x+1,flag_square[f].y+1);
                              }
                            }
                            if(start_x!=linee){//not same row
                              //aproaching
                              if(start_x>linee){
                                start_x--;
                              }//else start_x++; //never gonna happen
                            }
                            if(start_y!=column){
                              if(start_y>column){
                                start_y--;
                              }else{
                                start_y++;
                              }
                            }
                            if(start_y==column&&start_x==linee){
                              end=true;
                            }
                         }while(!end);
                      }
                      //for h
                      if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].h.x,pieces_topic_points.p_vector[square_num].h.y),ii+1,jj+1)){
                         ROS_INFO("gia to %d BRIKE TH SKIA TOU STO SHMEIO %d %d!",square_num,ii,jj);

                         //ROS_INFO("kai prepei na paei sto shmeio %d %d!",linee,column);
                         bool end=false;
                         int start_x=ii,start_y=jj;
                         do{
                            ROS_INFO("HEHE GEIA TO %d %d!",start_x+1,start_y+1);
                            for (int f = 0; f < flag_square.size(); f++){
                              if(start_x==flag_square[f].x&&start_y==flag_square[f].y&&flag_square[f].state.compare("ignore")!=0){
                                flag_square[f].state="ignore"; //propably it's a piece's shadow..
                                ROS_INFO("\033[1;32mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[f].x+1,flag_square[f].y+1);
                              }
                            }
                            if(start_x!=linee){//not same row
                              //aproaching
                              if(start_x>linee){
                                start_x--;
                              }//else start_x++; //never gonna happen
                            }
                            if(start_y!=column){
                              if(start_y>column){
                                start_y--;
                              }else{
                                start_y++;
                              }
                            }
                            if(start_y==column&&start_x==linee){
                              end=true;
                            }
                         }while(!end);
                      }
                                            //for h
                      if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].f.x,pieces_topic_points.p_vector[square_num].f.y),ii+1,jj+1)){
                         ROS_INFO("gia to %d BRIKE TH SKIA TOU STO SHMEIO %d %d!",square_num,ii,jj);

                         //ROS_INFO("kai prepei na paei sto shmeio %d %d!",linee,column);
                         bool end=false;
                         int start_x=ii,start_y=jj;
                         do{
                            ROS_INFO("HEHE GEIA TO %d %d!",start_x+1,start_y+1);
                            for (int f = 0; f < flag_square.size(); f++){
                              if(start_x==flag_square[f].x&&start_y==flag_square[f].y&&flag_square[f].state.compare("ignore")!=0){
                                flag_square[f].state="ignore"; //propably it's a piece's shadow..
                                ROS_INFO("\033[1;32mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[f].x+1,flag_square[f].y+1);
                              }
                            }
                            if(start_x!=linee){//not same row
                              //aproaching
                              if(start_x>linee){
                                start_x--;
                              }//else start_x++; //never gonna happen
                            }
                            if(start_y!=column){
                              if(start_y>column){
                                start_y--;
                              }else{
                                start_y++;
                              }
                            }
                            if(start_y==column&&start_x==linee){
                              end=true;
                            }
                         }while(!end);
                      }
                                            //for h
                      if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].g.x,pieces_topic_points.p_vector[square_num].g.y),ii+1,jj+1)){
                         ROS_INFO("gia to %d BRIKE TH SKIA TOU STO SHMEIO %d %d!",square_num,ii,jj);

                         //ROS_INFO("kai prepei na paei sto shmeio %d %d!",linee,column);
                         bool end=false;
                         int start_x=ii,start_y=jj;
                         do{
                            ROS_INFO("HEHE GEIA TO %d %d!",start_x+1,start_y+1);
                            for (int f = 0; f < flag_square.size(); f++){
                              if(start_x==flag_square[f].x&&start_y==flag_square[f].y&&flag_square[f].state.compare("ignore")!=0){
                                flag_square[f].state="ignore"; //propably it's a piece's shadow..
                                ROS_INFO("\033[1;32mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[f].x+1,flag_square[f].y+1);
                              }
                            }
                            if(start_x!=linee){//not same row
                              //aproaching
                              if(start_x>linee){
                                start_x--;
                              }//else start_x++; //never gonna happen
                            }
                            if(start_y!=column){
                              if(start_y>column){
                                start_y--;
                              }else{
                                start_y++;
                              }
                            }
                            if(start_y==column&&start_x==linee){
                              end=true;
                            }
                         }while(!end);
                      }
                    }
                  }


              /*for (int j = 0; j < flag_square.size(); j++){

                  ROS_INFO("eimaste sto %d kai elegxoume to shmeio %d %d an anhkei mesa sto kouti %d %d",square_num , pieces_topic_points.p_vector[square_num].e.x , pieces_topic_points.p_vector[square_num].e.y,flag_square[j].x+1,flag_square[j].y+1);
                    if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].e.x,pieces_topic_points.p_vector[square_num].e.y),flag_square[j].x+1,flag_square[j].y+1)){
                        flag_square[j].state="ignore"; //propably it's a piece's shadow..
                         ROS_INFO("\033[1;31mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[j].x+1,flag_square[j].y+1);
                    }
                 ROS_INFO("eimaste sto %d kai elegxoume to shmeio %d %d an anhkei mesa sto kouti %d %d",square_num , pieces_topic_points.p_vector[square_num].f.x , pieces_topic_points.p_vector[square_num].f.y,flag_square[j].x+1,flag_square[j].y+1);
                 
                    if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].f.x,pieces_topic_points.p_vector[square_num].f.y),flag_square[j].x+1,flag_square[j].y+1)){
                        flag_square[j].state="ignore"; //propably it's a piece's shadow..
                        ROS_INFO("\033[1;31mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[j].x+1,flag_square[j].y+1);
                    }
                 ROS_INFO("eimaste sto %d kai elegxoume to shmeio %d %d an anhkei mesa sto kouti %d %d",square_num , pieces_topic_points.p_vector[square_num].g.x , pieces_topic_points.p_vector[square_num].g.y,flag_square[j].x+1,flag_square[j].y+1);
                 
                    if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].g.x,pieces_topic_points.p_vector[square_num].g.y),flag_square[j].x+1,flag_square[j].y+1)){
                        flag_square[j].state="ignore"; //propably it's a piece's shadow..
                        ROS_INFO("\033[1;31mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[j].x+1,flag_square[j].y+1);
                    }
                 ROS_INFO("eimaste sto %d kai elegxoume to shmeio %d %d an anhkei mesa sto kouti %d %d",square_num , pieces_topic_points.p_vector[square_num].h.x , pieces_topic_points.p_vector[square_num].h.y,flag_square[j].x+1,flag_square[j].y+1);
                 
                    if(checkifItsInsidetheSquare(CvPoint(pieces_topic_points.p_vector[square_num].h.x,pieces_topic_points.p_vector[square_num].h.y),flag_square[j].x+1,flag_square[j].y+1)){
                        flag_square[j].state="ignore"; //propably it's a piece's shadow..
                        ROS_INFO("\033[1;31mAPOKLEIEI VASH TOU PRWTOU P BRIKE TH SKIA TOU STO SHMEIO %d %d!\033[0m\n",flag_square[j].x+1,flag_square[j].y+1);
                    }
                }*/
                }
              }else if(flag_square[i].x==linee&&flag_square[i].y==column&&found_first&&sqrt(pow(first.x-linee,2)+pow(first.y-column,2))>1.5&&flag_square[i].state.compare("ignore")!=0){
                if(!found_second&&flag_square[i].state.compare("cautious")!=0){
                  ROS_INFO("\033[1;33mMWRE LES NA KSEKINAEI APO DW %d %d!\033[0m\n",linee+1,column+1);
                  second.x=linee;
                  second.y=column;
                  found_second=true;
                }  
                ROS_INFO("\033[1;33mAN ANOTHEEEEEEEER ONE AT %d %d!\033[0m\n",linee+1,column+1);
              }
            } 



            linee++;
            if(linee>=8){
              linee=0;
              column--;
            }
          }while(column>=0/*&&!(found_second&&found_first)*/);


          if(found_second&&found_first){
           ROS_INFO("cvpoints start %d %d  end %d %d",first.x,first.y,second.x,second.y);
           colorMovement(snap_one,first,second,yeap);
         }
        }
        flag_square.clear();
      } // else frames>=2..


/*      for( int j = 0; j < chess_topic_points.p_vector.size(); j++ ){
          circle(src, Point(chess_topic_points.p_vector[j].x,chess_topic_points.p_vector[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("she %d -> %d %d",chess_topic_points.p_vector[j].x,chess_topic_points.p_vector[j].y);
      }*/
/*      cv::imshow(OPENCV_WINDOW,new_image3);
      cv::waitKey(3);*/

      
      chess_topic_points.p_vector.clear();
      pieces_topic_points.p_vector.clear();
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


int *square_CornPoints(/*std::string letter,*/int int_letter, int num){  //! first column and then line as inputs
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

bool checkifItsInsidetheSquare(CvPoint point,int linee,int column){

  int *sq_points=square_CornPoints(column,linee);
  Point left_bottom_p,left_top_p,right_top_p,right_bottom_p;
  std::vector<CvPoint> v;
  left_bottom_p.x=chess_topic_points.p_vector[sq_points[0]].x;
  left_bottom_p.y=chess_topic_points.p_vector[sq_points[0]].y;
  v.push_back(left_bottom_p);
  left_top_p.x=chess_topic_points.p_vector[sq_points[1]].x;
  left_top_p.y=chess_topic_points.p_vector[sq_points[1]].y;
  v.push_back(left_top_p);
  right_top_p.x=chess_topic_points.p_vector[sq_points[3]].x;
  right_top_p.y=chess_topic_points.p_vector[sq_points[3]].y;
  v.push_back(right_top_p);
  right_bottom_p.x=chess_topic_points.p_vector[sq_points[2]].x;
  right_bottom_p.y=chess_topic_points.p_vector[sq_points[2]].y;
  v.push_back(right_bottom_p);

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

  if(point.x>=(int)(point.y-left_bottom_p.y+lamda1*left_bottom_p.x)/lamda1){
    if(point.x<=(int)(point.y-right_bottom_p.y+lamda2*right_bottom_p.x)/lamda2){
      if(point.y<=(int)(lamda3*(point.x-right_bottom_p.x)+right_bottom_p.y)){
        if(point.y>=(int)(lamda4*(point.x-right_top_p.x)+right_top_p.y)){
            //ROS_INFO("yes, it belongs to the square.");
            return true;
        }
      }
    }
  }
/*
  cv::line( img, left_bottom_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
  cv::line( img, right_bottom_p,right_top_p, Scalar(0,0,255), 1, LINE_AA);
  cv::line( img, left_bottom_p,right_bottom_p, Scalar(0,0,255), 1, LINE_AA);
  cv::line( img, right_top_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
  cv::imshow("checkifItsInsidetheSquare", img);*/


  return false;
}

void colorMovement(Mat img,CvPoint start_pos,CvPoint end_pos,bool seq){

  int *sq_points=square_CornPoints(start_pos.y+1,start_pos.x+1);
  Point left_bottom_p,left_top_p,right_top_p,right_bottom_p;
  std::vector<CvPoint> v;
  left_bottom_p.x=chess_topic_points.p_vector[sq_points[0]].x;
  left_bottom_p.y=chess_topic_points.p_vector[sq_points[0]].y;
  v.push_back(left_bottom_p);
  left_top_p.x=chess_topic_points.p_vector[sq_points[1]].x;
  left_top_p.y=chess_topic_points.p_vector[sq_points[1]].y;
  v.push_back(left_top_p);
  right_top_p.x=chess_topic_points.p_vector[sq_points[3]].x;
  right_top_p.y=chess_topic_points.p_vector[sq_points[3]].y;
  v.push_back(right_top_p);
  right_bottom_p.x=chess_topic_points.p_vector[sq_points[2]].x;
  right_bottom_p.y=chess_topic_points.p_vector[sq_points[2]].y;
  v.push_back(right_bottom_p);

  if(seq){    
    cv::line( img, left_bottom_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
    cv::line( img, right_bottom_p,right_top_p, Scalar(0,0,255), 1, LINE_AA);
    cv::line( img, left_bottom_p,right_bottom_p, Scalar(0,0,255), 1, LINE_AA);
    cv::line( img, right_top_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
  }else{
    cv::line( img, left_bottom_p,left_top_p, Scalar(0,255,0), 1, LINE_AA);
    cv::line( img, right_bottom_p,right_top_p, Scalar(0,255,0), 1, LINE_AA);
    cv::line( img, left_bottom_p,right_bottom_p, Scalar(0,255,0), 1, LINE_AA);
    cv::line( img, right_top_p,left_top_p, Scalar(0,255,0), 1, LINE_AA);
  }
  v.clear();
  sq_points=square_CornPoints(end_pos.y+1,end_pos.x+1);
  //Point left_bottom_p,left_top_p,right_top_p,right_bottom_p;
  left_bottom_p.x=chess_topic_points.p_vector[sq_points[0]].x;
  left_bottom_p.y=chess_topic_points.p_vector[sq_points[0]].y;
  v.push_back(left_bottom_p);
  left_top_p.x=chess_topic_points.p_vector[sq_points[1]].x;
  left_top_p.y=chess_topic_points.p_vector[sq_points[1]].y;
  v.push_back(left_top_p);
  right_top_p.x=chess_topic_points.p_vector[sq_points[3]].x;
  right_top_p.y=chess_topic_points.p_vector[sq_points[3]].y;
  v.push_back(right_top_p);
  right_bottom_p.x=chess_topic_points.p_vector[sq_points[2]].x;
  right_bottom_p.y=chess_topic_points.p_vector[sq_points[2]].y;
  v.push_back(right_bottom_p);

  if(!seq){    
    cv::line( img, left_bottom_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
    cv::line( img, right_bottom_p,right_top_p, Scalar(0,0,255), 1, LINE_AA);
    cv::line( img, left_bottom_p,right_bottom_p, Scalar(0,0,255), 1, LINE_AA);
    cv::line( img, right_top_p,left_top_p, Scalar(0,0,255), 1, LINE_AA);
  }else{
    cv::line( img, left_bottom_p,left_top_p, Scalar(0,255,0), 1, LINE_AA);
    cv::line( img, right_bottom_p,right_top_p, Scalar(0,255,0), 1, LINE_AA);
    cv::line( img, left_bottom_p,right_bottom_p, Scalar(0,255,0), 1, LINE_AA);
    cv::line( img, right_top_p,left_top_p, Scalar(0,255,0), 1, LINE_AA);
  }
  cv::imshow("end", img);

}

void recursive_flagCheck(int index,std::vector<vision::ChessPoint> flag_square,float scan_area){
  
  //ROS_INFO("\nnew iteration with index=%d",index);
  if(visited_v[index].compare("y")==0){
    //ROS_INFO("inside return");
    return;
  }
  visited_v[index]="y";
  for (int i = 0; i < flag_square.size(); ++i) 
  {
    //ROS_INFO("comparing the %d %d",index,i);
    if(i==index){
      //ROS_INFO("passing..");
      continue; // to avoid the check with itself in any iteration..
    } 
    float dist=sqrt(pow(flag_square[index].x-flag_square[i].x,2)+pow(flag_square[index].y-flag_square[i].y,2));
    if(dist<=scan_area){
      //ROS_INFO("is %d visited?=%s",i,visited_v[i].c_str());
      if(visited_v[i].compare("y")!=0){
        //ROS_INFO("accepted to  i=%d",i);     
        first_team.push_back(i);
        recursive_flagCheck(i,flag_square,scan_area);
      }//else continue..
    }
  }
  //ROS_INFO("returned final");
}