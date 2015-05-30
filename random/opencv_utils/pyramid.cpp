#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    cv::Mat matrix = cv::Mat::zeros(210, 210, CV_8UC1);
    cv::imshow("Window", matrix);

    std::cout << matrix.rows << std::endl;
    std::cout << matrix.cols << std::endl;

    int val = 0;
    for (int padding = 0; padding < 105; padding += 10)
    {
        cv::Mat_<uchar> sub_rect = matrix(cv::Range(padding, 210 - padding), cv::Range(padding, 210 - padding));

        std::cout << sub_rect.rows << std::endl;
        std::cout << sub_rect.cols << std::endl;

        cv::MatIterator_<uchar> it;
        for (it = sub_rect.begin(); it != sub_rect.end(); it++)
            (*it) = val;

        val += 20;
    }

    cv::imshow("Pyramid", matrix);
    cv::waitKey(0);

    return 0;
}