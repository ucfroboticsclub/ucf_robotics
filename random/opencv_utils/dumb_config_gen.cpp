//
// Created by kenneth on 5/29/15.
//

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    cv::Scalar orange_thresh_low(0, 125, 125);
    cv::Scalar orange_thresh_high(80, 255, 255);
    cv::Scalar white_thresh_low(0, 0, 210);
    cv::Scalar white_thresh_high(255, 20, 255);
    int grid_cols = 6;
    int grid_rows = 6;
    int contour_area_threshold = 50;
    int morph_elem = 2;
    int morph_size = 14;
    int morph_operator = 2;

    cv::FileStorage fs("/home/kenneth/temp/front_configuration.xml", cv::FileStorage::WRITE);
    fs << "white-thresh-low" << white_thresh_low;
    fs << "white-thresh-high" << white_thresh_high;
    fs << "orange_thresh-low" << orange_thresh_low;
    fs << "orange_thresh-high" << orange_thresh_high;
    fs << "grid-cols" << grid_cols;
    fs << "grid-rows" << grid_rows;
    fs << "contour-area-threshold" << contour_area_threshold;
    fs << "morph-elem" << morph_elem;
    fs << "morph-size" << morph_size;
    fs << "morph-operator" << morph_operator;
    fs.release();

    return 0;
}