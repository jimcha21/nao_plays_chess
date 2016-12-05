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
Mat myShiTomasi_dst,result_image,result_image2;

std::vector<cv::Point> chess_knob_vector_;
std::vector<cv::Point> chess_processed_points_;
std::vector<cv::Point> chess_featured_points_;
std::vector<cv::Point3d> temp_vector;

int thresh = 200;
int max_thresh = 255;
int hough_thres=150;

int myShiTomasi_qualityLevel = 80;
int myHarris_qualityLevel = 50;
int max_qualityLevel = 100;
double myHarris_minVal, myHarris_maxVal;
double myShiTomasi_minVal, myShiTomasi_maxVal;

RNG rng(12345);


/// Function headers
void filteredArray();
void myShiTomasi_function( int, void* );
void myHarris_function( int, void* );
void cornerHarris_demo( int, void* );
void best_corners( int, void*,bool useHarrisDetector,int compar);
int findMin(int po[100][3]);

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

          //image_sub_ = it_.subscribe("/naoqi_driver_node/camera/bottom/image_raw", 10, &ImageConverter::imageCb, this);
          image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);    
          chess_sub = nh_.subscribe("chess_points", 10, chessboardVectorTopic); //chessboard subscriber

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
      ROS_INFO("received %d = 81 ?",chess_knob_vector_.size());
      for(int i=0;i<chess_knob_vector_.size();i++)
      { ROS_INFO("VECTOR %d %d",chess_knob_vector_[i].x,chess_knob_vector_[i].y);}
  
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
      //src=cv_ptr->image.clone();

      src = imread("src/vision/src/3.jpg", 1);
      if(src.empty())
      {
        ROS_INFO("can not open the image");// << filename << endl;
        return ;
      }

      cvtColor( src, src_gray, COLOR_BGR2GRAY ); 
     
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

      best_corners(0,0,true,1); //With corner Harris enabled -
      //best_corners(0,0,false,2); 
      
      
      //ROS_INFO("vrike %d",chess_featured_points_.size());
      //ROS_INFO("found total %d points",temp_vector.size());

      //filtered array of points
      filteredArray();
      //ROS_INFO("reduced the amount of point to %d ",chess_processed_points_.size());

/*      result_image2 = src.clone(); 
      for( int j = 0; j < chess_processed_points_.size(); j++ ){
          //circle(result_image2, Point(chess_processed_points_[j].x,chess_processed_points_[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          ROS_INFO("he %d -> %d %d",chess_processed_points_[j].x,chess_processed_points_[j].y);
      }

      for( int j = 0; j < chess_featured_points_.size(); j++ ){
          //circle(result_image2, Point(chess_processed_points_[j].x,chess_processed_points_[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          ROS_INFO("she %d -> %d %d",chess_featured_points_[j].x,chess_featured_points_[j].y);
      }

      for( int j = 0; j < chess_knob_vector_.size(); j++ ){
          circle(src, Point(chess_knob_vector_[j].x,chess_knob_vector_[j].y), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 );    
          //ROS_INFO("she %d -> %d %d",chess_knob_vector_[j].x,chess_knob_vector_[j].y);
      }
*/

/*PROCESS HERE WITH PROCESSED POINTS AND GROUNDTRUTH POINTS - MIDPOINT?REJECTION?*/
      /*AND THEN PUBLISH THEM*/

      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      //cv::imshow("OPENCV_WINDOW",result_image);
      cv::imshow("OPENCV_WINDOW",src);
      //cv::imshow("OPENCV_WINDOW2",result_image2);
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());

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
{
     
    std::vector< cv::Point2f > corners;

    // maxCorners – The maximum number of corners to return. If there are more corners
    // than that will be found, the strongest of them will be returned
    int maxCorners = 5; 

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
      cv::imshow( "argv[1]", src_gray );
    }else{
      cv::imshow( "without harrris", src_gray );
    }
    result_image=src_gray.clone();
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