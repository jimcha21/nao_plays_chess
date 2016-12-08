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
static const std::string OPENCV_WINDOW2 = "Image window2";
Mat src, src_gray;
Mat myHarris_dst, Mc;
Mat myShiTomasi_dst,result_image,result_image2,shi_image,feat_image,final_image;

std::vector<cv::Point> chess_topic_points;
std::vector<cv::Point> chess_processed_points_;
std::vector<cv::Point> chess_featured_points_;
std::vector<cv::Point3d> temp_vector;

bool debug_mode=false;
int confidence_area_pix=10;
int maxCorners = 50; // depends on room brightness .. reduce the value if the room is too bright ..
int thresh = 200;
int max_thresh = 255;
int hough_thres=150;

int myShiTomasi_qualityLevel = 40;
int myHarris_qualityLevel = 50;
int max_qualityLevel = 100;
double myHarris_minVal, myHarris_maxVal;
double myShiTomasi_minVal, myShiTomasi_maxVal;

RNG rng(12345);


/// Function headers
int *square_CornPoints(std::string str ,int num);
void filteredArray();
bool checkifItsInsidetheSquare(Mat,Point,bool);
void myShiTomasi_function( int, void* );
void myHarris_function( int, void* );
void cornerHarris_demo( int, void* );
void best_corners( int, void*,bool useHarrisDetector,int compar);
int findMin(int po[100][3]);

//******************************************************************************************

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

          chess_sub = nh_.subscribe("chessboard_knob_coordinates", 10, chessboardVectorTopic); //chessboard subscriber
          image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::imageCb, this);
          //image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);    
          
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


/* //testing smth   
   int fwt=0;
      ROS_INFO("1 %d",fwt++);
      ROS_INFO("2 %d",fwt);
      ROS_INFO("3 %d",++fwt);
      ROS_INFO("4 %d",fwt);
      fwt+=1;
      ROS_INFO("5 %d",fwt);*/

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
      
      if(!debug_mode){
        src=cv_ptr->image.clone();
      }else{
        src = imread("src/vision/src/3.jpg", 1);
        if(src.empty())
        {
          ROS_INFO("can not open the image");// << filename << endl;
          return ;
        }
      }

      cvtColor( src, src_gray, COLOR_BGR2GRAY ); 
     
      ROS_INFO("received %d points",chess_topic_points.size());
/*      if(chess_topic_points.size()>0){ // received the points from the topic
      //ROS_INFO("VECTOR %d %d",chess_topic_points[0].x,chess_topic_points[0].y);
        checkifItsInsidetheSquare(src,Point(1,2),true);
        /*for(int i=0;i<chess_topic_points.size();i++)
        { 
          ROS_INFO("VECTOR %d %d",chess_topic_points[i].x,chess_topic_points[i].y);
        }*/
    //  }*/


      /// Set some parameters
      int blockSize = 3; int apertureSize = 3, choice=2; //1 for harris, else for shatosshi

      if(choice==1){
          /// My Harris matrix -- Using cornerEigenValsAndVecs
          myHarris_dst = Mat::zeros( src_gray.size(), CV_32FC(6) );
          Mc = Mat::zeros( src_gray.size(), CV_32FC1 );

          cornerEigenValsAndVecs( src_gray, myHarris_dst, blockSize, apertureSize, BORDER_DEFAULT );

          // calculate Mc 
          for( int j = 0; j < src_gray.rows; j++ )
             { for( int i = 0; i < src_gray.cols; i++ )
                  {
                    float lambda_1 = myHarris_dst.at<Vec6f>(j, i)[0];
                    float lambda_2 = myHarris_dst.at<Vec6f>(j, i)[1];
                    Mc.at<float>(j,i) = lambda_1*lambda_2 - 0.04f*pow( ( lambda_1 + lambda_2 ), 2 );
                  }
             }

          minMaxLoc( Mc, &myHarris_minVal, &myHarris_maxVal, 0, 0, Mat() );

          // Create Window and Trackbar 
          namedWindow( "My Harris corner detector", WINDOW_AUTOSIZE );
          createTrackbar( " Quality Level:", "My Harris corner detector", &myHarris_qualityLevel, max_qualityLevel, myHarris_function );
          myHarris_function( 0, 0 );
      }
      else if(choice==2){
          /// My Shi-Tomasi -- Using cornerMinEigenVal
          myShiTomasi_dst = Mat::zeros( src_gray.size(), CV_32FC1 );
          cornerMinEigenVal( src_gray, myShiTomasi_dst, blockSize, apertureSize, BORDER_DEFAULT );

          minMaxLoc( myShiTomasi_dst, &myShiTomasi_minVal, &myShiTomasi_maxVal, 0, 0, Mat() );

          // Create Window and Trackbar 
          namedWindow("My Shi Tomasi corner detector", WINDOW_AUTOSIZE );
          createTrackbar( " Quality Level:", "My Shi Tomasi corner detector", &myShiTomasi_qualityLevel, max_qualityLevel, myShiTomasi_function );
          myShiTomasi_function( 0, 0 );

      }

      //best_corners(0,0,false,2); 
      best_corners(0,0,true,1); //With corner Harris enabled -
     
      //ROS_INFO("found total %d",chess_featured_points_.size());
      //ROS_INFO("found total %d points",temp_vector.size());

      filteredArray();

      //ROS_INFO("reduced the amount of point to %d ",chess_processed_points_.size());
      
      std::vector<cv::Point> chess_final_points;
      cv::Point pt;
      final_image = src.clone(); 
      int min,distance;
      bool min_found,min_found2;

      if(chess_topic_points.size()>0){
        for(int i=0;i<chess_topic_points.size();i++){
          //init point
          pt=cv::Point(chess_topic_points[i].x,chess_topic_points[i].y);
          min=9999;
          min_found=false;
          min_found2=false;
          for(int j=0;j<chess_processed_points_.size();j++){
              distance=sqrt(pow((chess_topic_points[i].x-chess_processed_points_[j].x),2)+pow((chess_topic_points[i].y-chess_processed_points_[j].y),2));
              if(distance<min&&distance<=confidence_area_pix){ //no need for min because the vector is already filtered..
                min=sqrt(pow((chess_topic_points[i].x-chess_processed_points_[j].x),2)+pow((chess_topic_points[i].y-chess_processed_points_[j].y),2));
                //pt=cv::Point((int)(chess_topic_points[i].x+chess_processed_points_[j].x)/2,(int)(chess_topic_points[i].y+chess_processed_points_[j].y)/2);
                pt=cv::Point(chess_processed_points_[j].x,chess_processed_points_[j].y);
                min_found=true;
                //ROS_INFO("min found at %d %d",i,j);
              }
          }

          //Merge the extra featured points that were detected along with the Shitomashi Corner detections.. 
          for(int j=0;j<chess_featured_points_.size();j++){ //80 in total
              distance=sqrt(pow((chess_topic_points[i].x-chess_featured_points_[j].x),2)+pow((chess_topic_points[i].y-chess_featured_points_[j].y),2));
              if(distance<min&&distance<=confidence_area_pix){ //no need for min because the vector is already filtered..
                min=sqrt(pow((chess_topic_points[i].x-chess_featured_points_[j].x),2)+pow((chess_topic_points[i].y-chess_featured_points_[j].y),2));
                pt=cv::Point(chess_featured_points_[j].x,chess_featured_points_[j].y);
                min_found2=true;
                min_found=false;
                //ROS_INFO("MPIKE");
              }
          }

          //chess_final_points.push_back(cv::Point(chess_topic_points[i].x,chess_topic_points[i].y));
          chess_final_points.push_back(pt);
          if(min_found){ //random color on points detected correctly from shitomashi function - red color on points that shitomashi function didn't found and received from the topic ..
            circle(final_image, Point(pt.x,pt.y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          }else if(min_found2){ //random color on points detected correctly from shitomashi function - red color on points that shitomashi function didn't found and received from the topic ..
            circle(final_image, Point(pt.x,pt.y), 4, Scalar( 0,255,0 ), -1, 8, 0 );    
          }
          else{
            circle(final_image, Point(pt.x,pt.y), 4, Scalar( 0,0,255 ), -1, 8, 0 );    
          }
           
        }
      }

      shi_image = src.clone(); 

      for( int j = 0; j < chess_processed_points_.size(); j++ ){
          circle(shi_image, Point(chess_processed_points_[j].x,chess_processed_points_[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("he %d -> %d %d",chess_processed_points_[j].x,chess_processed_points_[j].y);
      }

      feat_image = src.clone(); 
      for( int j = 0; j < chess_featured_points_.size(); j++ ){
          circle(feat_image, Point(chess_processed_points_[j].x,chess_processed_points_[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("she %d -> %d %d",chess_featured_points_[j].x,chess_featured_points_[j].y);
      }

      for( int j = 0; j < chess_topic_points.size(); j++ ){
          circle(src, Point(chess_topic_points[j].x,chess_topic_points[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("she %d -> %d %d",chess_topic_points[j].x,chess_topic_points[j].y);
      }

      //choosing a chess square ..
      int *sq_points;
      sq_points=square_CornPoints("e",1);
      //ROS_INFO("shmeia %d %d %d %d",sq_points[0],sq_points[1],sq_points[2],sq_points[3]);
      //
 if(chess_topic_points.size()>0){


/*      int diste[4][2];
      diste[0][0]=(int)sqrt(pow(chess_final_points[sq_points[0]].x,2)+pow(chess_final_points[sq_points[0]].y,2));
      diste[0][1]=sq_points[0];
      diste[1][0]=(int)sqrt(pow(chess_final_points[sq_points[1]].x,2)+pow(chess_final_points[sq_points[1]].y,2));
      diste[1][1]=sq_points[1];
      diste[2][0]=(int)sqrt(pow(chess_final_points[sq_points[2]].x,2)+pow(chess_final_points[sq_points[2]].y,2));
      diste[2][1]=sq_points[2];
      diste[3][0]=(int)sqrt(pow(chess_final_points[sq_points[3]].x,2)+pow(chess_final_points[sq_points[3]].y,2));
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
      left_bottom_p.x=chess_final_points[sq_points[0]].x;
      left_bottom_p.y=chess_final_points[sq_points[0]].y;
      v.push_back(left_bottom_p);
      left_top_p.x=chess_final_points[sq_points[1]].x;
      left_top_p.y=chess_final_points[sq_points[1]].y;
      v.push_back(left_top_p);
      right_top_p.x=chess_final_points[sq_points[3]].x;
      right_top_p.y=chess_final_points[sq_points[3]].y;
      v.push_back(right_top_p);
      right_bottom_p.x=chess_final_points[sq_points[2]].x;
      right_bottom_p.y=chess_final_points[sq_points[2]].y;
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

      for(int x=min_x;x<=max_x;x++){
        for(int y=min_y;y<=max_y;y++){
          if(/*(y>=(int)(lamda1*(x-left_bottom_p.x)+left_bottom_p.y)) &&*/ (x>=(int)(y-left_bottom_p.y+lamda1*left_bottom_p.x)/lamda1)){
            if(/*(y<=(int)(lamda2*(x-right_bottom_p.x)+right_bottom_p.y))&&*/(x<=(int)(y-right_bottom_p.y+lamda2*right_bottom_p.x)/lamda2)){
              if((y<=(int)(lamda3*(x-right_bottom_p.x)+right_bottom_p.y))/*&&(x>=(int)(y-right_bottom_p.y+lamda3*right_bottom_p.x)/lamda3)*/){
                if((y>=(int)(lamda4*(x-right_top_p.x)+right_top_p.y))/*&&(x<=(int)(y-right_top_p.y+lamda4*right_top_p.x)/lamda4)*/){
                  
                  //THIS IS THE AREA THAT WILL BE EXAMINED
                  final_image.at<cv::Vec3b>(y,x)[0] = 255;
                  final_image.at<cv::Vec3b>(y,x)[1] = 255;
                  final_image.at<cv::Vec3b>(y,x)[2] = 255;
            
          }}}}
        }
      }
      v.clear();
    
}

      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //  cv::imshow("OPENCV_WINDOW",shi_image);
    //  cv::imshow("GROUNDTRUTH",src);
    //  cv::imshow("FEATURED",feat_image);
      cv::imshow("FINAL",final_image);
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());

  
      chess_final_points.clear();
      chess_featured_points_.clear(); //no need to check for overflow..
      chess_processed_points_.clear();
      temp_vector.clear(); //vector3d
  }
};



////////////////////////????/////////////////////////////////////////////////////????/////////////////////////////////////////////////////????/////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reveiver");
  ImageConverter ic;
  ros::spin();
  return 0;
}

////////////////////////????/////////////////////////////////////////////////////????/////////////////////////////////////////////////////????/////////////////////////////

int *square_CornPoints(std::string str ,int num){
    static int sq_points[4];
    int mul=0;
    if(str.compare("a")==0){
      mul=0;
    }else if(str.compare("b")==0){
      mul=1;
    }else if(str.compare("c")==0){
      mul=2;
    }else if(str.compare("d")==0){
      mul=3;
    }else if(str.compare("e")==0){
      mul=4;
    }else if(str.compare("f")==0){
      mul=5;
    }else if(str.compare("g")==0){
      mul=6;
    }else if(str.compare("h")==0){
      mul=7;
    }
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

void filteredArray() {

  cv::Point pt;

  for (int j = 0; j < temp_vector.size(); j++) {
    if (temp_vector[j].z == 0) {
      temp_vector[j].z = 1;
      for (int i = 0; i < temp_vector.size(); i++) {

        if (abs(temp_vector[j].x -temp_vector[i].x) <= 5 && abs(temp_vector[j].y - temp_vector[i].y)<= 5) {
          pt= cv::Point(temp_vector[j].x,temp_vector[j].y);
          //newpoint[k][2] = 0;
          temp_vector[i].z = 1;
        }
      }
      chess_processed_points_.push_back(pt);
    }
  }

}

/**
 * @function myShiTomasi_function
 */
void myShiTomasi_function( int, void* )
{
  result_image = src.clone();

  if( myShiTomasi_qualityLevel < 1 ) { myShiTomasi_qualityLevel = 1; }

  for( int j = 0; j < src_gray.rows; j++ )
     { for( int i = 0; i < src_gray.cols; i++ )
          {
            if( myShiTomasi_dst.at<float>(j,i) > myShiTomasi_minVal + ( myShiTomasi_maxVal - myShiTomasi_minVal )*myShiTomasi_qualityLevel/max_qualityLevel )
              {
                  circle( result_image, Point(i,j), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );
                  temp_vector.push_back(cv::Point3d(i,j,0));
              }
          }
     }
  //cv::imshow( "My Shi Tomasi corner detector", result_image );
}

/**
 * @function myHarris_function
 */
void myHarris_function( int, void* )
{
  result_image = src.clone();  

  if( myHarris_qualityLevel < 1 ) { myHarris_qualityLevel = 1; }

  for( int j = 0; j < src_gray.rows; j++ )
     { for( int i = 0; i < src_gray.cols; i++ )
          {
            if( Mc.at<float>(j,i) > myHarris_minVal + ( myHarris_maxVal - myHarris_minVal )*myHarris_qualityLevel/max_qualityLevel )
              { 
                circle( result_image, Point(i,j), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );
                temp_vector.push_back(cv::Point3d(i,j,0));
              }
          }
     }
  //cv::imshow( "My Harris corner detector", result_image );
}
//classic corner harris
void cornerHarris_demo( int, void* )
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  //cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
               temp_vector.push_back(cv::Point3d(i,j,0));
              }
          }
     }
  /// Showing the result
  namedWindow( "Corners detected", CV_WINDOW_AUTOSIZE );
  imshow( "Corners detected", dst_norm_scaled );
}

void best_corners( int, void*,bool useHarrisDetector,int compar)
{        /*for(int i=0;i<chess_topic_points.size();i++)
        { 
          ROS_INFO("VECTOR %d %d",chess_topic_points[i].x,chess_topic_points[i].y);
        }*/
     
    std::vector< cv::Point2f > corners;

    // maxCorners – The maximum number of corners to return. If there are more corners
    // than that will be found, the strongest of them will be returned
    
    //int maxCorners = 40; 

    // qualityLevel – Characterizes the minimal accepted quality of image corners;
    // the value of the parameter is multiplied by the by the best corner quality
    // measure (which is the min eigenvalue, see cornerMinEigenVal() ,
    // or the Harris function response, see cornerHarris() ).
    // The corners, which quality measure is less than the product, will be rejected.
    // For example, if the best corner has the quality measure = 1500,
    // and the qualityLevel=0.01 , then all the corners which quality measure is
    // less than 15 will be rejected.
    double qualityLevel = 0.01;

    // minDistance – The minimum possible Euclidean distance between the returned corners
    double minDistance = 20.;

    // mask – The optional region of interest. If the image is not empty (then it
    // needs to have the type CV_8UC1 and the same size as image ), it will specify
    // the region in which the corners are detected
    cv::Mat mask; //!!

    // blockSize – Size of the averaging block for computing derivative covariation
    // matrix over each pixel neighborhood, see cornerEigenValsAndVecs()
    int blockSize = 3;

    // useHarrisDetector – Indicates, whether to use operator or cornerMinEigenVal()
    //bool useHarrisDetector = true;

    // k – Free parameter of Harris detector
    double k = 0.04;

    cv::goodFeaturesToTrack( src_gray, corners, maxCorners, qualityLevel, minDistance, mask, blockSize, useHarrisDetector, k );

    for( size_t i = 0; i < corners.size(); i++ )
    {
        cv::circle( src_gray, corners[i], 5, cv::Scalar( 255. ), -1 );
        chess_featured_points_.push_back(cv::Point(corners[i].x,corners[i].y));
    }

    //cv::namedWindow( "argv[1]", CV_WINDOW_NORMAL );
      
    if(compar==1){
      cv::imshow( "with corner harris", src_gray );
    }else{
      cv::imshow( "without harrris", src_gray );
    }
    //result_image=src_gray.clone();
    //cv::waitKey(0);

}

//function find minimum set of x,y
int findMin(int po[100][3])
{
  int j;
  int min = 100, minx = 10000, miny = 10000;
  for (j = 0; j < 100; j++) {
    if ((po[j][0] < minx) && (po[j][1] < miny) && (po[j][0] != 0) && (po[j][1]) != 0) {
      min = j;
      minx = po[j][0];
      miny = po[j][1];
    }
  }
  return min;
}