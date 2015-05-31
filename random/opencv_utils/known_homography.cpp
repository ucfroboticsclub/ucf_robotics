//
// Created by kenneth on 5/27/15.
//

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::namedWindow("Original");
    cv::namedWindow("Bird's Eye");

    cv::Mat original;
    cv::Mat warped;
    cv::Mat homography_matrix;

    original = cv::imread("/home/kenneth/Pictures/checkerboard.png");

    cv::FileStorage fs("/home/kenneth/temp/front_homography.xml", cv::FileStorage::READ);
    fs["homography-matrix"] >> homography_matrix;
    fs.release();

    warped = cv::Mat::zeros(original.size(), CV_8UC3);
    cv::Size warped_size(original.cols, original.rows);

    cv::warpPerspective(original, warped, homography_matrix, warped_size,
        CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

    cv::imshow("Original", original);
    cv::imshow("Bird's Eye", warped);

    while(cv::waitKey(0) != 27)
        continue;

    return 0;
}