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


using namespace cv;
using namespace std;


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
/// Global variables
Mat src, src_gray;
Mat myHarris_dst;Mat Mc;
Mat myShiTomasi_dst; Mat result_image;
int thresh = 200;
int max_thresh = 255;

char* source_window = "Source image";
char* corners_window = "Corners detected";

int myShiTomasi_qualityLevel = 50;
int myHarris_qualityLevel = 50;
int max_qualityLevel = 100;

double myHarris_minVal; double myHarris_maxVal;
double myShiTomasi_minVal; double myShiTomasi_maxVal;

RNG rng(12345);

const char* myHarris_window = "My Harris corner detector";
const char* myShiTomasi_window = "My Shi Tomasi corner detector";

/// Function headers
void myShiTomasi_function( int, void* );
void myHarris_function( int, void* );
void cornerHarris_demo( int, void* );
void best_corners( int, void*,bool useHarrisDetector,int compar);

/**
 * @function main
 */

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    //image_sub_ = it_.subscribe("/naoqi_driver_node/camera/front/image_raw", 10, &ImageConverter::imageCb, this);
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageCb, this);    
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

      cvtColor( cv_ptr->image, src_gray, COLOR_BGR2GRAY );
      src=cv_ptr->image.clone();

      /// Set some parameters
      int blockSize = 3; int apertureSize = 3, choice=3; //1 for harris, else for shatosshi

      if(choice==1){
          /// My Harris matrix -- Using cornerEigenValsAndVecs
          myHarris_dst = Mat::zeros( src_gray.size(), CV_32FC(6) );
          Mc = Mat::zeros( src_gray.size(), CV_32FC1 );

          cornerEigenValsAndVecs( src_gray, myHarris_dst, blockSize, apertureSize, BORDER_DEFAULT );

          /* calculate Mc */
          for( int j = 0; j < src_gray.rows; j++ )
             { for( int i = 0; i < src_gray.cols; i++ )
                  {
                    float lambda_1 = myHarris_dst.at<Vec6f>(j, i)[0];
                    float lambda_2 = myHarris_dst.at<Vec6f>(j, i)[1];
                    Mc.at<float>(j,i) = lambda_1*lambda_2 - 0.04f*pow( ( lambda_1 + lambda_2 ), 2 );
                  }
             }

          minMaxLoc( Mc, &myHarris_minVal, &myHarris_maxVal, 0, 0, Mat() );

          /* Create Window and Trackbar */
          namedWindow( myHarris_window, WINDOW_AUTOSIZE );
          createTrackbar( " Quality Level:", myHarris_window, &myHarris_qualityLevel, max_qualityLevel, myHarris_function );
          myHarris_function( 0, 0 );
      }
      else if(choice==2){
          /// My Shi-Tomasi -- Using cornerMinEigenVal
          myShiTomasi_dst = Mat::zeros( src_gray.size(), CV_32FC1 );
          cornerMinEigenVal( src_gray, myShiTomasi_dst, blockSize, apertureSize, BORDER_DEFAULT );

          minMaxLoc( myShiTomasi_dst, &myShiTomasi_minVal, &myShiTomasi_maxVal, 0, 0, Mat() );

          /* Create Window and Trackbar */
          namedWindow( myShiTomasi_window, WINDOW_AUTOSIZE );
          createTrackbar( " Quality Level:", myShiTomasi_window, &myShiTomasi_qualityLevel, max_qualityLevel, myShiTomasi_function );
          myShiTomasi_function( 0, 0 );
      }else{
        best_corners(0,0,true,1);
        best_corners(0,0,false,2);
        
      }
      
      //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      //cv::imshow(OPENCV_WINDOW,result_image);
      cv::waitKey(3);

      // Output modified video stream
      //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
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
              { circle( result_image, Point(i,j), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 ); }
          }
     }
  //cv::imshow( myShiTomasi_window, result_image );
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
              { circle( result_image, Point(i,j), 4, Scalar( rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255) ), -1, 8, 0 ); }
          }
     }
  //cv::imshow( myHarris_window, result_image );
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
              }
          }
     }
  /// Showing the result
  namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}

void best_corners( int, void*,bool useHarrisDetector,int compar)
{
     
    std::vector< cv::Point2f > corners;

    // maxCorners – The maximum number of corners to return. If there are more corners
    // than that will be found, the strongest of them will be returned
    int maxCorners = 30;

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
    }

    //cv::namedWindow( "argv[1]", CV_WINDOW_NORMAL );
    if(compar==1){
      cv::imshow( "argv[1]", src_gray );
    }else{
      cv::imshow( "without harrris", src_gray );
    }
    
    //cv::waitKey(0);

}