#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

int main()
{
	Mat img_src, img_mul_color, img_blur, img_can,img_gray;
	Mat channels[3];

	img_src = imread("test1.jpg");

	namedWindow("Source", CV_WINDOW_AUTOSIZE);
	namedWindow("Multiplied Color", CV_WINDOW_AUTOSIZE);
	namedWindow("Blurred", CV_WINDOW_AUTOSIZE);
	namedWindow("Canny", CV_WINDOW_AUTOSIZE);
	namedWindow("Gray",CV_WINDOW_AUTOSIZE);

	split(img_src, channels);
	multiply(channels[1], channels[2], img_mul_color, (1.0F / 255));
	multiply(img_mul_color, channels[0], img_mul_color, (1.0F / 255));
	GaussianBlur(img_mul_color, img_blur, Size(15, 15), 0, 0);
	Canny(img_blur, img_can, 50, 100, 3);

	cvtColor(img_src, img_gray, CV_BGR2GRAY);
	Mat binThresh;
	threshold(img_blur, binThresh, 96, 255, 0);
	
	//threshold(img_gray, binThresh, 127, 255, 0);


	for (;;)
	{
		
		imshow("Source", img_src);
		imshow("Gray", img_gray);
		imshow("Multiplied Color", img_mul_color);
		imshow("Blurred", img_blur);
		imshow("Canny", binThresh);

		// Exit key
		if (waitKey(30) >= 0) break;
	}

	return 0;
}