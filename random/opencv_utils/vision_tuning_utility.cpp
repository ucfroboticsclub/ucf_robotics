//
// Created by kenneth on 5/29/15.
//

#include <opencv2/opencv.hpp>

cv::Mat src, filtered, warped, homography_matrix, debug;

// Default settings.
int morph_operator = 0;
int morph_elem = 0;
int morph_size = 14;
int grid_cols = 6;
int grid_rows = 6;
int contour_area_threshold = 50;
cv::Scalar orange_thresh_low(0, 125, 125);
cv::Scalar orange_thresh_high(80, 255, 255);
cv::Scalar white_thresh_low(0, 0, 210);
cv::Scalar white_thresh_high(255, 20, 255);

// Flags to indicate first time we adjust white or orange values.
// Silly workaround to avoid using integers for each value.
bool white_adjusted = false;
bool orange_adjusted = false;

void whiteHSVChange(int, void*)
{
    if (!white_adjusted)
    {
        cv::setTrackbarPos("White H Low", "White Threshold", (int)white_thresh_low[0]);
        cv::setTrackbarPos("White S Low", "White Threshold", (int)white_thresh_low[1]);
        cv::setTrackbarPos("White V Low", "White Threshold", (int)white_thresh_low[2]);

        cv::setTrackbarPos("White H High", "White Threshold", (int)white_thresh_high[0]);
        cv::setTrackbarPos("White S High", "White Threshold", (int)white_thresh_high[1]);
        cv::setTrackbarPos("White V High", "White Threshold", (int)white_thresh_high[2]);

        white_adjusted = true;
    }
    else
    {
        white_thresh_low[0] = cv::getTrackbarPos("White H Low", "White Threshold");
        white_thresh_low[1] = cv::getTrackbarPos("White S Low", "White Threshold");
        white_thresh_low[2] = cv::getTrackbarPos("White V Low", "White Threshold");

        white_thresh_high[0] = cv::getTrackbarPos("White H High", "White Threshold");
        white_thresh_high[1] = cv::getTrackbarPos("White S High", "White Threshold");
        white_thresh_high[2] = cv::getTrackbarPos("White V High", "White Threshold");
    }
}

void orangeHSVChange(int, void*)
{
    if (!orange_adjusted)
    {
        cv::setTrackbarPos("Orange H Low", "Orange Threshold", (int)orange_thresh_low[0]);
        cv::setTrackbarPos("Orange S Low", "Orange Threshold", (int)orange_thresh_low[1]);
        cv::setTrackbarPos("Orange V Low", "Orange Threshold", (int)orange_thresh_low[2]);

        cv::setTrackbarPos("Orange H High", "Orange Threshold", (int)orange_thresh_high[0]);
        cv::setTrackbarPos("Orange S High", "Orange Threshold", (int)orange_thresh_high[1]);
        cv::setTrackbarPos("Orange V High", "Orange Threshold", (int)orange_thresh_high[2]);

        orange_adjusted = true;
    }
    else
    {
        orange_thresh_low[0] = cv::getTrackbarPos("Orange H Low", "Orange Threshold");
        orange_thresh_low[1] = cv::getTrackbarPos("Orange S Low", "Orange Threshold");
        orange_thresh_low[2] = cv::getTrackbarPos("Orange V Low", "Orange Threshold");

        orange_thresh_high[0] = cv::getTrackbarPos("Orange H High", "Orange Threshold");
        orange_thresh_high[1] = cv::getTrackbarPos("Orange S High", "Orange Threshold");
        orange_thresh_high[2] = cv::getTrackbarPos("Orange V High", "Orange Threshold");
    }
}



void filterImage()
{
    // TODO: Work on improving filtering and line fitting.

    // Since MORPH_X options are int valued 2, 3, 4, 5, and 6
    int operation = morph_operator + 2;
    cv::Mat element = cv::getStructuringElement(morph_elem,
                                                cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                                cv::Point(morph_size, morph_size));

    // Get HSV encoded image for thresholding.
    cv::Mat src_HSV;
    cv::cvtColor(src, src_HSV, CV_BGR2HSV);

    // Find orange.
    cv::Mat threshed_orange;
    cv::inRange(src_HSV, orange_thresh_low, orange_thresh_high,
                threshed_orange);
    cv::imshow("Orange Threshold", threshed_orange);

    // Blur the HSV image.
    cv::Mat src_blur;
    cv::GaussianBlur(src_HSV, src_blur, cv::Size(7, 7), 0, 0);

    // Find white in a different image.
    cv::Mat threshed_white;
    cv::inRange(src_blur, white_thresh_low, white_thresh_high,
                threshed_white);
    cv::imshow("White Threshold", threshed_white);

    // Do Morphology on orange image to blob over white portions of obstacles.
    cv::morphologyEx(threshed_orange, filtered, operation, element);
    cv::imshow("Orange Morphed", filtered);

    // "Subtract" orange detection from white detection.
    cv::bitwise_not(filtered, filtered);
    cv::bitwise_and(threshed_white, filtered, filtered);
    cv::imshow("Color Filter Result", filtered);

    // Grid based fit line stuff
    // TODO: Understand this

    int block_width = filtered.size().width / grid_cols;
    int block_height = filtered.size().height / grid_rows;

    
    cv::Mat fitline;
    fitline = cv::Mat::zeros(filtered.size(), CV_8U);

    for (int i = 0; i < grid_cols; i++)
    {
        for (int j = 0; j < grid_rows; j++)
        {
            std::vector< std::vector< cv::Point > > contours;
            cv::Vec4f lines;

            cv::Rect grid(block_width * i, block_height * j, block_width, block_height);
            cv::Mat roi = filtered(grid);
            cv::Mat fitline_roi = fitline(grid);
            cv::findContours(roi, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> points;

            std::vector< std::vector < cv::Point > >::const_iterator iter;
            for (iter = contours.begin(); iter != contours.end(); iter++)
            {
                if (cv::contourArea((*iter)) < contour_area_threshold)
                    continue;

                std::vector<cv::Point>::const_iterator iter2;
                for (iter2 = (*iter).begin(); iter2 != (*iter).end(); iter2++)
                {
                    points.push_back(*iter2);
                }
            }

            if (points.size() > 0)
            {
                cv::fitLine(points, lines, CV_DIST_HUBER, 0, 0.01, 0.01);


                cv::Point2f low_left(lines[2] - lines[0] * 1000,
                                     lines[3] - lines[1] * 1000);
                cv::Point2f up_right(lines[2] + lines[0] * 1000,
                                     lines[3] + lines[1] * 1000);

                cv::line(fitline_roi, low_left, up_right, cv::Scalar(255, 0, 0), 5);
            }
        }
    }

   fitline.copyTo(filtered);
}

// Warp the filtered and fitlined image to bird's eye view.
void warpFiltered()
{
    warped = cv::Mat::zeros(filtered.size(), CV_8UC3);
    cv::Size warpedsize(filtered.cols, filtered.rows);

    cv::warpPerspective(filtered, warped, homography_matrix, warpedsize,
                        CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
}

// Warp the original image to bird's eye view.  For comparison purposes.
void warpOriginal()
{
    debug = cv::Mat::zeros(src.size(), CV_8UC3);
    cv::Size warpedsize(src.cols, src.rows);

    cv::warpPerspective(src, debug, homography_matrix, warpedsize,
                        CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
}

void writeOut(std::string filepath)
{
    cv::FileStorage fs(filepath, cv::FileStorage::WRITE);

    if (fs.isOpened())
    {
        std::cout << "Saving configuration to " << filepath << std::endl;

        fs << "white-thresh-low" << white_thresh_low;
        fs << "white-thresh-high" << white_thresh_high;
        fs << "orange-thresh-low" << orange_thresh_low;
        fs << "orange-thresh-high" << orange_thresh_high;
        fs << "grid-cols" << grid_cols;
        fs << "grid-rows" << grid_rows;
        fs << "contour-area-threshold" << contour_area_threshold;
        fs << "morph-elem" << morph_elem;
        fs << "morph-size" << morph_size;
        fs << "morph-operator" << morph_operator;
        fs.release();
    }
    else
    {
        std::cout << "Cannot write to " << filepath << std::endl;
    }
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cout << "Usage: executable homography_filepath output_filepath video_device_number" << std::endl;
        return -1;
    }

    // Pull in command line arguments.
    std::string homography_path = argv[1];
    std::string output_path = argv[2];
    int video_device = atoi(argv[3]);

    // Get homography matrix
    cv::FileStorage fs;
    fs.open(homography_path, cv::FileStorage::READ);
    fs["homography-matrix"] >> homography_matrix;
    fs.release();

    // Verify homography file is opened.
    if (homography_matrix.cols != 3 || homography_matrix.rows != 3)
    {
        std::cout << "Failed to read homography matrix at " << homography_path << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Found homography matrix at " << homography_path << std::endl;
    }

    // Check if the output file already exists so we can load its current settings.
    fs.open(output_path, cv::FileStorage::READ);
    if (fs.isOpened())
    {
        std::cout << "File already exists, loading settings..." << std::endl;

        fs["white-thresh-low"] >> white_thresh_low;
        fs["white-thresh-high"] >> white_thresh_high;
        fs["orange-thresh-low"] >> orange_thresh_low;
        fs["orange-thresh-high"] >> orange_thresh_high;
        fs["grid-cols"] >> grid_cols;
        fs["grid-rows"] >> grid_rows;
        fs["contour-area-threshold"] >> contour_area_threshold;
        fs["morph-elem"] >> morph_elem;
        fs["morph-size"] >> morph_size;
        fs["morph-operator"] >> morph_operator;

        fs.release();
    }

    cv::VideoCapture camera_cap(video_device);

    cv::namedWindow("Original");
    cv::namedWindow("Warped");
    cv::namedWindow("White Threshold");
    cv::namedWindow("Orange Threshold");
    cv::namedWindow("Orange Morphed");
    cv::namedWindow("Color Filter Result");
    cv::namedWindow("Filtered");
    cv::namedWindow("Warped and Filtered");
    cv::startWindowThread();

    // White Threshold window trackbars.
    cv::createTrackbar("White H Low", "White Threshold", 0, 255, whiteHSVChange);
    cv::createTrackbar("White S Low", "White Threshold", 0, 255, whiteHSVChange);
    cv::createTrackbar("White V Low", "White Threshold", 0, 255, whiteHSVChange);
    cv::createTrackbar("White H High", "White Threshold", 0, 255, whiteHSVChange);
    cv::createTrackbar("White S High", "White Threshold", 0, 255, whiteHSVChange);
    cv::createTrackbar("White V High", "White Threshold", 0, 255, whiteHSVChange);

    // Orange Threshold window trackbars.
    cv::createTrackbar("Orange H Low", "Orange Threshold", 0, 255, orangeHSVChange);
    cv::createTrackbar("Orange S Low", "Orange Threshold", 0, 255, orangeHSVChange);
    cv::createTrackbar("Orange V Low", "Orange Threshold", 0, 255, orangeHSVChange);
    cv::createTrackbar("Orange H High", "Orange Threshold", 0, 255, orangeHSVChange);
    cv::createTrackbar("Orange S High", "Orange Threshold", 0, 255, orangeHSVChange);
    cv::createTrackbar("Orange V High", "Orange Threshold", 0, 255, orangeHSVChange);

    // Morph operators
    //  0: Open, 1: Close, 2: Gradient, 3: Tophat, 4: Blackhat
    cv::createTrackbar("Morph Operation (- 2)", "Orange Morphed", &morph_operator, 4);
    // 0: Rectangle, 1: Cross, 2: Ellipse
    cv::createTrackbar("Morph Element", "Orange Morphed", &morph_elem, 2);
    // Kernel Size
    cv::createTrackbar("Morph Size (2n+1)", "Orange Morphed", &morph_size, 60);

    // Fitline Parameters
    cv::createTrackbar("Contour Area Threshold", "Filtered", &contour_area_threshold, 150);
    cv::createTrackbar("Grid Columns", "Filtered", &grid_cols, 16);
    cv::createTrackbar("Grid Rows", "Filtered", &grid_rows, 16);

    if (camera_cap.isOpened())
    {
        while (true)
        {
            camera_cap >> src;
            cv::imshow("Original", src);
            warpOriginal();
            cv::imshow("Warped", debug);
            filterImage();
            cv::imshow("Filtered", filtered);
            warpFiltered();
            cv::imshow("Warped and Filtered", warped);

            // Check for user input.  ESC to exit, 's' to save configuration
            int key = cv::waitKey(100);
            if (key % 256 == 27)
                break;
            if (key % 256 == 115)
                writeOut(output_path);
        }
    }
    else
    {
        std::cout << "Video device could not be opened" << std::endl;
        return -1;
    }

    camera_cap.release();

    return 0;
}