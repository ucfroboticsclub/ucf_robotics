//
// Created by kenneth on 5/24/15.
//
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::Mat original_image = cv::imread("/home/kenneth/Pictures/Selection_002.png", CV_LOAD_IMAGE_COLOR);
    cv::imshow("Original", original_image);

    std::vector<cv::Mat> results;
    cv::split(original_image, results);

    cv::imshow("Green", results[1]);

    cv::Mat_<uchar> clone1 = results[1].clone();
    cv::Mat_<uchar> clone2 = results[1].clone();

    double min, max;
    cv::minMaxLoc(clone1, &min, &max);

    std::cout << min << " " << max << std::endl;

    uchar thresh = (uchar)((max - min)/2.0);

    cv::MatIterator_<uchar> it;
    for (it = clone1.begin(); it != clone1.end(); ++it)
    {
        (*it) = thresh;
    }

    cv::compare(results[1], clone1, clone2, cv::CMP_GE);

    cv::imshow("wtf", clone2);
    results[1] = results[1] - clone2;
    cv::imshow("wtf2", results[1]);
    cv::waitKey(0);
}
