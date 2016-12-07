#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;

using namespace cv;


Mat K_means_func(Mat image);


int main(int argc, char** argv)
{
	//This first loop reorders the data from the image from a (rows,cols,3) matrix to an (rows*cols,3) matrix (one row per pixel).
	Mat image1 = imread("data/7.jpg", 1);
	Mat image2 = imread("data/8.jpg", 1);
	//the k-means result image
	Mat new_image1(image1.size(), image1.type());
	Mat new_image2(image2.size(), image2.type());

	//find the k-means image for the two images
	new_image1 = K_means_func(image1);
	new_image2 = K_means_func(image2);

	//show the result of the two images
	imshow("Clustered Image1", new_image1);
	imshow("Clustered Image2", new_image2);

	//sub the two images
	Mat new_image3(image1.size(), image1.type());
	//new_image3 = new_image2 - new_image1;
	cv::subtract(new_image2, new_image1, new_image3);
	imshow("image2-image1", new_image3);

	Mat new_image4(image1.size(), image1.type());
	//new_image3 = new_image1 - new_image2;
	cv::subtract(new_image1, new_image2, new_image4);
	imshow("image1-image2", new_image4);

	Mat new_image5(image1.size(), image1.type());

	cv::add(new_image3, new_image4, new_image5);
	imshow("add", new_image5);

	cout << "this is the size of image1 =" << image1.size() << "and image2 = " << image2.size() << "\n";

	/*
	for (int y = 0; y < rows; y++)
		for (int x = 0; x < cols; x++)
	*/
	/////////////////////convert apo 0-1 range se 0-255(dld apo 32F se 8U) 
	//Mat final_image;
	// Change image type from   32F to 8U
	//new_image.convertTo(final_image, CV_8U, 1.5, 10); 
	//cvtColor(final_image, final_image, CV_BGR2Luv);
	//imshow("Clustered Image", final_image);

	waitKey(0);
}


Mat K_means_func(Mat src) {

	Mat samples(src.rows * src.cols, 3, CV_32F);
	for (int y = 0; y < src.rows; y++)
		for (int x = 0; x < src.cols; x++)
			for (int z = 0; z < 3; z++)
				samples.at<float>(y + x*src.rows, z) = src.at<Vec3b>(y, x)[z];

	int clusterCount = 5;//edw einai o ari8mos twn xrwmatwn ths eikonas!!
	Mat labels;
	int attempts = 5;//Flag to specify the number of times the algorithm is executed using different initial labellings. The algorithm returns the labels that yield the best compactness
	Mat centers;
	kmeans(samples, clusterCount, labels, TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10000, 0.0001), attempts, KMEANS_PP_CENTERS, centers);


	Mat new_image(src.size(), src.type());
	//The loop at the end replaces each pixel in the image with the corresponding cluster center for visualization
	for (int y = 0; y < src.rows; y++)
		for (int x = 0; x < src.cols; x++)
		{
			int cluster_idx = labels.at<int>(y + x*src.rows, 0);
			new_image.at<Vec3b>(y, x)[0] = (uchar)centers.at<float>(cluster_idx, 0);
			new_image.at<Vec3b>(y, x)[1] = (uchar)centers.at<float>(cluster_idx, 1);
			new_image.at<Vec3b>(y, x)[2] = (uchar)centers.at<float>(cluster_idx, 2);
		}


	//imshow("Clustered Image", new_image);
	return new_image;
}