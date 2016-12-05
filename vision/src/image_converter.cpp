
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

#include "vision/ChessVector.h"
#include "vision/ChessPoint.h"

#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>


/*
      For popular image encodings, CvBridge will optionally do color or pixel depth conversions as necessary. To use this feature, specify the encoding to be one of the following strings:

            mono8: CV_8UC1, grayscale image

            mono16: CV_16UC1, 16-bit grayscale image

            bgr8: CV_8UC3, color image with blue-green-red color order

            rgb8: CV_8UC3, color image with red-green-blue color order

            bgra8: CV_8UC4, BGR color image with an alpha channel

            rgba8: CV_8UC4, RGB color image with an alpha channel 

      Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.

      Finally, CvBridge will recognize Bayer pattern encodings as having OpenCV type 8UC1 (8-bit unsigned, one channel). It will not perform conversions to or from Bayer pattern; in a typical ROS system, this is done instead by image_proc. CvBridge recognizes the following Bayer encodings:
      
      But you must convert the RGB8 input to the specific one you want...
            bayer_rggb8

            bayer_bggr8

            bayer_gbrg8

            bayer_grbg8
      */

using namespace std;
using namespace cv;
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
cv::Size checkerboardSize = cv::Size(8, 8);

vision::ChessPoint chess_point;
vision::ChessVector chess_vector;
ros::Publisher whaza;

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
      
    whaza = nh_.advertise <vision::ChessVector>("chess_points",0);


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

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //cv::circle((cv_ptr->image), cv::Point(50, 50), 10, CV_RGB(255,0,0));
/*
    //CHESSSSS
    // check for chessboard corners
    vector<cv::Point2f> corner_current;
    bool found_current;

    // convert image to gray-> function findChesboardCorners needs gray image
    cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
    found_current = findChessboardCorners(cv_ptr->image, checkerboardSize, corner_current,
                                          cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found_current)
    {
      // set current image as src1
      //src1 = cv_ptr->image;
      cv::cornerSubPix(cv_ptr->image, corner_current, checkerboardSize, cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.1));
      // convert image to color
      cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2RGB);
      cv::drawChessboardCorners(cv_ptr->image, checkerboardSize, corner_current, found_current);

    }

    //

    // Update GUI Window
    */
 /*   Mat dst, cdst,src_gray;

    Canny(cv_ptr->image, dst, 50, 200, 3);
    cvtColor(dst, cdst, COLOR_GRAY2BGR);

    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
    for( size_t i = 0; i < lines.size(); i++ )
    {
        Vec4i l = lines[i];
        line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
    }
    imshow("detected lines", cdst);

    //cv::imshow(OPENCV_WINDOW, cv_ptr->image); //the robot's source image..*/

CvPoint chessknob_point;
chessknob_point.x=10;
chessknob_point.y=10;
cv::circle((cv_ptr->image), chessknob_point, 10, CV_RGB(255,0,0));


chess_point.x=220;
chess_point.y=646;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=185;
chess_point.y=690;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=300;
chess_point.y=688;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=318;
chess_point.y=646;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=424;
chess_point.y=642;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=411;
chess_point.y=688;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=727;
chess_point.y=644;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=749;
chess_point.y=688;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=864;
chess_point.y=688;
chess_vector.p_vector.push_back(chess_point);
chess_point.x=832;
chess_point.y=646;
chess_vector.p_vector.push_back(chess_point);



whaza.publish(chess_vector);
chess_vector.p_vector.clear();
   // cvCircle(cv_ptr->image,chessknob_point,1, CV_RGB(0, 255,0),5,8,0);
    //cvCircle(image,cvPoint((int)visualize2d_points[8][0], (int)visualize2d_points[8][1]),1,color,5,8,0);
    //ROS_INFO("1red eytheia ? me kentro ? %d %d",(int)visualize2d_points[8][0], (int)visualize2d_points[8][1]);



    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
   // image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;

  //ImageConverter2 ic2;
  ros::spin();
  return 0;
}
