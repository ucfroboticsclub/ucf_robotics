//
// Created by kenneth on 5/26/15.
//

// Really terrible script for obtaining homography matrix for a given camera
//  configuration.  Can save the output to an xml file for reuse.
// Maps chessboard in bird's eye image such that 1 pixel ~= 1 cm (I think).
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    int board_w = 13;
    int board_h = 10;
    cv::Size board_sz = cv::Size_<int>(board_w, board_h);
    cv::namedWindow("Original");
    cv::namedWindow("Gray");
    cv::namedWindow("Bird's Eye");

    cv::Mat original;
    cv::Mat gray_original;
    cv::Mat original_copy;
    cv::Mat warped;

    // Get original BGR encoded image.
    original = cv::imread("/home/kenneth/Pictures/checkerboard.png");
    original_copy = original.clone();
    // Convert the BGR image to grayscale before finding corners.
    cv::cvtColor(original, gray_original, CV_BGR2GRAY);

    // Show all the images up to this point.
    cv::imshow("Original", original);
    cv::imshow("Gray", gray_original);

    std::vector<cv::Point2f> corners;
    bool found_corners = cv::findChessboardCorners(gray_original, board_sz, corners,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if (found_corners)
    {
        std::cout << "Found corners" << std::endl;

        cv::cornerSubPix(gray_original, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        // Get the image and object points.
        cv::Point2f obj_points[4], img_points[4];

        // Conversions so that 1 pixel is equivalent to 1 cm???
        double width_cm = 5.5 * (board_w - 1);
        double height_cm = 5.5 * (board_h - 1);

        // 4 Corners of physical chessboard
        // These corners are where we want the corners of the chessboard
        //  to appear in the warped image.
        // NOTE: These have to be specified in the same order
        //  as the points in the other array after this.
        obj_points[0].x = original_copy.cols/2 - width_cm/2;
        obj_points[0].y = original_copy.rows - height_cm;
        obj_points[1].x = original_copy.cols/2 + width_cm/2;
        obj_points[1].y = original_copy.rows - height_cm;
        obj_points[2].x = original_copy.cols/2 - width_cm/2;
        obj_points[2].y = original_copy.rows;
        obj_points[3].x = original_copy.cols/2 + width_cm/2;
        obj_points[3].y = original_copy.rows;

        // 4 Corners of chessboard found in image given by (x,y) pixel location
        img_points[0] = corners[0];
        img_points[1] = corners[board_w - 1];
        img_points[2] = corners[(board_h - 1)*board_w];
        img_points[3] = corners[corners.size() - 1];

        // Draw the points in order: B, G, R, YELLOW
        cv::circle(original_copy, cvPointFrom32f(img_points[0]), 9, CV_RGB(0,0,255), 3);
        cv::circle(original_copy, cvPointFrom32f(img_points[1]), 9, CV_RGB(0,255,0), 3);
        cv::circle(original_copy, cvPointFrom32f(img_points[2]), 9, CV_RGB(255,0,0), 3);
        cv::circle(original_copy, cvPointFrom32f(img_points[3]), 9, CV_RGB(255,255,0), 3);

        // Find the homography.
        cv::Mat h = cv::getPerspectiveTransform(obj_points, img_points);
        // Adjust zoom if desired, doesn't seem necessary.
        //float Z = 1.0f;
        //h.at<double>(2, 2) = Z;

        cv::FileStorage fs("/home/kenneth/temp/front_homography.xml", cv::FileStorage::WRITE);
        fs << "homography-matrix" << h;
        fs.release();

        warped = cv::Mat::zeros(original_copy.size(), CV_8UC3);
        cv::Size dstSize(original_copy.cols, original_copy.rows);

        cv::warpPerspective(original_copy, warped, h, dstSize,
            CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS);

        cv::imshow("Bird's Eye", warped);
    }

    while(true)
    {
        int key = cv::waitKey();
        if (key % 256 == 27)
            break;
    }

    return 0;
}