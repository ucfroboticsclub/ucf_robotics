//
// Created by kenneth on 5/26/15.
//

// Really terrible script for obtaining homography matrix for a given camera
//  configuration.  Can save the output to an xml file for reuse.
// Maps chessboard in bird's eye image such that 1 pixel ~= 1 cm (I think).
#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: executable homography_output_path video_device" << std::endl;
        return -1;
    }

    // Get runtime configuration.
    std::string output_path = argv[1];
    int video_device = atoi(argv[2]);

    int board_w = 13;
    int board_h = 10;
    cv::Size board_sz = cv::Size_<int>(board_w, board_h);
    cv::namedWindow("Original");
    cv::namedWindow("Gray");
    cv::namedWindow("Bird's Eye");
    cv::startWindowThread();

    cv::Mat original;
    cv::Mat gray_original;
    cv::Mat original_copy;
    cv::Mat warped;
    cv::Mat homography;

    // Open video device and set properties.
    cv::VideoCapture cap(video_device);
    if (cap.isOpened())
    {
        std::cout << "Camera opened" << std::endl;
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    }
    else
    {
        std::cout << "Could not open camera" << std::endl;
        return -1;
    }

    int key_pressed = 0;
    while(key_pressed % 256 != 27)
    {
        // Get the current frame and make a copy.
        cap >> original;
        original_copy = original.clone();
        std::cout << "Image width,height:" << original.cols << "," << original.rows << std::endl;

        // Convert the BGR image to grayscale before finding corners.
        cv::cvtColor(original, gray_original, CV_BGR2GRAY);

        // Show all the images up to this point.
        cv::imshow("Original", original);
        cv::imshow("Gray", gray_original);

        // Attempt to find interior corners of the chessboard.
        std::vector<cv::Point2f> corners;
        bool found_corners = cv::findChessboardCorners(gray_original, board_sz, corners,
                                                       CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if (found_corners)
        {
            cv::cornerSubPix(gray_original, corners, cv::Size(11, 11),
                             cv::Size(-1, -1),
                             cv::TermCriteria(
                                     CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30,
                                     0.1));

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
            obj_points[0].x = original_copy.cols / 2 - width_cm / 2;
            obj_points[0].y = original_copy.rows - height_cm;
            obj_points[1].x = original_copy.cols / 2 + width_cm / 2;
            obj_points[1].y = original_copy.rows - height_cm;
            obj_points[2].x = original_copy.cols / 2 - width_cm / 2;
            obj_points[2].y = original_copy.rows;
            obj_points[3].x = original_copy.cols / 2 + width_cm / 2;
            obj_points[3].y = original_copy.rows;

            // 4 Corners of chessboard found in image given by (x,y) pixel location
            img_points[0] = corners[0];
            img_points[1] = corners[board_w - 1];
            img_points[2] = corners[(board_h - 1) * board_w];
            img_points[3] = corners[corners.size() - 1];

            // Draw the points in order: B, G, R, YELLOW
            cv::circle(original_copy, cvPointFrom32f(img_points[0]), 9,
                       CV_RGB(0, 0, 255), 3);
            cv::circle(original_copy, cvPointFrom32f(img_points[1]), 9,
                       CV_RGB(0, 255, 0), 3);
            cv::circle(original_copy, cvPointFrom32f(img_points[2]), 9,
                       CV_RGB(255, 0, 0), 3);
            cv::circle(original_copy, cvPointFrom32f(img_points[3]), 9,
                       CV_RGB(255, 255, 0), 3);

            // Find the homography.
            homography = cv::getPerspectiveTransform(obj_points, img_points);

            // Warp the image to bird's eye view.
            warped = cv::Mat::zeros(original_copy.size(), CV_8UC3);
            cv::Size dstSize(original_copy.cols, original_copy.rows);
            cv::warpPerspective(original_copy, warped, homography, dstSize,
                                CV_INTER_LINEAR | CV_WARP_INVERSE_MAP |
                                CV_WARP_FILL_OUTLIERS);

            // Display results.
            cv::imshow("Bird's Eye", warped);
        }

        // Check user input.
        key_pressed = cv::waitKey(50);
        // If user pressed 's', save the homography file and inform the user.
        if (key_pressed % 256 == 115)
        {
            cv::FileStorage fs(output_path, cv::FileStorage::WRITE);
            fs << "homography-matrix" << homography;
            fs.release();

            std::cout << "Wrote homography matrix to " << output_path << std::endl;
        }
    }

    // Release the camera connection.
    cap.release();

    return 0;
}