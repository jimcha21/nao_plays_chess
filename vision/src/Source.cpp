#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>


using namespace cv;
using namespace std;


/// Global variables
Mat src, src_gray, temp_image;
Mat myShiTomasi_dst; Mat result_image, image;
int thresh = 120;
int myShiTomasi_qualityLevel = 50;
int max_thresh = 255;
int max_qualityLevel = 100;

double myShiTomasi_minVal; double myShiTomasi_maxVal;

//point of the corners
int point[100][3];
int newpoint[100][3];
int m = 0, n = 0;

RNG rng(12345);
char* source_window = "Source image";
char* corners_window = "Corners detected";

/// Function header
void cornerHarris_demo(int, void*);
void myShiTomasi_function(int, void*);

// find minimum
int findMin(int po[100][3]);
//filtered array with pointers
void filteredArray();
//find the nearest neabor
void nearestNeabor(int thesi, int po[100][3], int o, Mat image);
//calculate the euclidean distance
int distanceCalculate(int x1, int y1, int x2, int y2);
//filter shitomashi
void filter_Shitomasi(int po[100][3]);

//photo
Mat dst, dst_norm_scaled;


/** @function main */
int main(int argc, char** argv)
{
	/// Load source image and convert it to gray

	src = imread("data/pic11.jpg", 1);

	namedWindow(source_window, CV_WINDOW_AUTOSIZE);
	imshow(source_window, src);
	cvtColor(src, src_gray, COLOR_BGR2GRAY);

	/// Set some parameters
	int blockSize = 3; int apertureSize = 3;
	/// My Shi-Tomasi -- Using cornerMinEigenVal
	myShiTomasi_dst = Mat::zeros(src_gray.size(), CV_32FC1);
	cornerMinEigenVal(src_gray, myShiTomasi_dst, blockSize, apertureSize, BORDER_DEFAULT);
	minMaxLoc(myShiTomasi_dst, &myShiTomasi_minVal, &myShiTomasi_maxVal, 0, 0, Mat());

	/* Create Window and Trackbar */
	namedWindow("result", WINDOW_AUTOSIZE);
	createTrackbar(" Quality Level:", "result", &myShiTomasi_qualityLevel, max_qualityLevel, myShiTomasi_function);
	myShiTomasi_function(0, 0);

	//cornerHarris_demo(0, 0);

	

	//filtered array of points
	filteredArray();

	// output each array element's value
	for (int i = 0; i < 100; i++) {
		if ((point[i][0] != 0) && (point[i][1] != 0)) {
			cout << "The corner's points are: " << point[i][0] << " ," << point[i][1] << "," <<point[i][2]<<"\n";
		}
	}
	for (int i = 0; i < 100; i++) {
		if ((newpoint[i][0] != 0) && (newpoint[i][1] != 0)) {
			cout << "The new array of corner's points are: " << newpoint[i][0] << " ," << newpoint[i][1] << "," << newpoint[i][2] << "\n";
		}
	}
	
	

	//find the minimun set of x,y of the newpoint array
	int i = findMin(newpoint);
	//cout << "OUT the min row is:" << i << "\n";

	//find the nearest neabor
	int o = 0;
	image = src.clone();
	//nearestNeabor(i, newpoint, o, image);

	//show the lines that nearestNeabor create

	imshow("result", image);
	//filtered shitomashi
	filter_Shitomasi(newpoint);

	waitKey(0);
	return(0);
}

//------------------------------------------------------------------------------------
void filteredArray() {
	int l = 0, k = 0;
	for (int j = 0; j < 100; j++) {
		if (point[j][2] == 0) {
			point[j][2] = 1;
			for (int i = 0; i < 100; i++) {

				if (((point[j][0] - point[i][0] <= 5) && (point[j][0] - point[i][0] >= -5)) && ((point[j][1] - point[i][1] <= 5) && (point[j][1] - point[i][1] >= -5))) {

					//cout << "1.TA I KAI J EINAI SXEDON IDIA. [j,0]= " << point[j][0] << " ,[i,0]=" << point[i ][0] << " ,[j,1]=" << point[j][1] << " ,[i,1]=" << point[i][1] << "me j="<<j<<"kai i="<<i<<"\n";
					newpoint[k][0] = point[j][0];
					newpoint[k][1] = point[j][1];
					//newpoint[k][2] = 0;
					point[i][2] = 1;
				}
			}
			k++;
		}
	}

}

void nearestNeabor(int thesi, int po[100][3], int o, Mat image) {


	cout << "the starting point is" << thesi << "\n";
	int min = 1000, min_thesi = thesi, D = 0;
	for (int j = 0; j < 100; j++) {
		//cout << "auto einai to j=" << j << "kai auto i thesi="<<thesi<< "kai auto to po="<< po[j][2]<< "\n";
		if ((j != thesi) && po[j][2] == 0) {
			if (po[j][0] != 0 && po[j][1] != 0) {

				D = distanceCalculate(po[thesi][0], po[thesi][1], po[j][0], po[j][1]);
				//cout << "this is the distance, D=" << D << "in thesi = " << j << "\n";
				if (D < min) {
					min = D;
					min_thesi = j;
				}
			}
			else {
				j = 100;
			}
		}
	}
	cout << "this is the smallest distance, D=" << min << "in thesi = " << min_thesi << "\n";
	if (o < 50) {
		line(image, Point(po[thesi][0],po[thesi][1]), Point(po[min_thesi][0],po[min_thesi][1]), Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 1, 8, 0);
		o++;
		po[thesi][3] = 1;
		nearestNeabor(min_thesi, newpoint, o, image);
	}
	else {
		//line(image, Point(po[thesi][0], po[thesi][1]), Point(po[min_thesi][0], po[min_thesi][1]), Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 4, 8, 0);
		o = 50;
	}

}

//fuction calculate the Euclidean distance
int distanceCalculate(int x1, int y1, int x2, int y2)
{
	int x = x1 - x2;
	int y = y1 - y2;
	int dist;

	dist = pow(x, 2) + pow(y, 2);           //calculating distance by euclidean formula
	dist = sqrt(dist);                  //sqrt is function in math.h

	return dist;
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


//fuction for circling only the filtered points
void filter_Shitomasi(int po[100][3]) {
	result_image = src.clone();
	int m = 1;
	//circle only the final points
	for (int k = 0; k < 100; k++) {
		for (int j = 0; j < src_gray.rows; j++) {
			for (int i = 0; i < src_gray.cols; i++) {
				if ((newpoint[k][0] == i && newpoint[k][1] == j) && (newpoint[k][0] != 0 && newpoint[k][1] != 0)) {
					circle(result_image, Point(i,j), 4, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8, 0);
					/*cout << "mpika gia m = " << m << " fora.\n";
					cout << "mpika gia i = " << newpoint[k][0] << " gia j =" << newpoint[k][1] << "i ="<<i<<" j= "<<j<< " .\n";
					m++;*/
				}
			}
		}
	}
	imshow("result", result_image);
}


//-----------------------------------------------------------------------------------
/** @function cornerHarris_demo */
void cornerHarris_demo(int, void*)
{

	Mat dst, dst_norm, dst_norm_scaled;
	dst = Mat::zeros(src.size(), CV_32FC1);

	/// Detector parameters
	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;

	/// Detecting corners
	cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);

	/// Normalizing
	normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(dst_norm, dst_norm_scaled);

	/// Drawing a circle around corners

	for (int j = 0; j < dst_norm.rows; j++)
	{
		for (int i = 0; i < dst_norm.cols; i++)
		{
			if ((int)dst_norm.at<float>(j, i) > thresh)
			{
				//cout << "Value of i is : " << i << "and the value of j is : " << j << endl;
				point[m][n] = { i };
				n++;
				point[m][n] = { j };
				n = 0;
				m++;

				//circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
			}
		}
	}
	/// Showing the result
	namedWindow(corners_window, CV_WINDOW_AUTOSIZE);
	imshow(corners_window, dst_norm_scaled);
}


void myShiTomasi_function(int, void*)
{
	result_image = src.clone();
	int m = 0, n = 0;
	if (myShiTomasi_qualityLevel < 1) { myShiTomasi_qualityLevel = 1; }

	for (int j = 0; j < src_gray.rows; j++)
	{
		for (int i = 0; i < src_gray.cols; i++)
		{
			if (myShiTomasi_dst.at<float>(j, i) > myShiTomasi_minVal + (myShiTomasi_maxVal - myShiTomasi_minVal)*myShiTomasi_qualityLevel / max_qualityLevel)
			{
				point[m][n] = { i };
				n++;
				point[m][n] = { j };
				n = 0;
				m++;
				circle(result_image, Point(i, j), 4, Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), -1, 8, 0);
			}
		}
	}
}