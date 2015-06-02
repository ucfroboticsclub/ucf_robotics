//
// Created by kenneth on 5/26/15.
//

#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

/*cv::Mat src, filtered, warped, homography_matrix;

image_transport::Publisher filter_pub;
ros::Publisher laser_pub;*/

// TODO: Clean all of this stuff up
// TODO: Separate this thing into a class with members.
// TODO: Clearly mark and separate debugging code like image viewing

class LaneDetector
{
public:

   LaneDetector(ros::NodeHandle& nh)
    : nh_(nh), it_(nh)
    {
        initializeParams();

        // Start up ROS connections to the outside world.
        ros_image_sub_ = it_.subscribe(source_image_topic_, 1, &LaneDetector::imageCallback, this);
        laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(fake_scan_topic_, 1);

        // Algorithm value parsing.
        cv::FileStorage fs;

        // Initialize homography matrix from input file.
        fs.open(homography_file_, cv::FileStorage::READ);
        fs["homography-matrix"] >> homography_matrix_;
        fs.release();

        // Initialize algorithm values
        fs.open(configuration_file_, cv::FileStorage::READ);
        fs["white-thresh-low"] >> white_thresh_low_;
        fs["white-thresh-high"] >> white_thresh_high_;
        fs["orange-thresh-low"] >> orange_thresh_low_;
        fs["orange-thresh-high"] >> orange_thresh_high_;
        fs["grid-cols"] >> grid_cols_;
        fs["grid-rows"] >> grid_rows_;
        fs["contour-area-threshold"] >> contour_area_threshold_;
        fs["morph-elem"] >> morph_elem_;
        fs["morph-size"] >> morph_size_;
        fs["morph-operator"] >> morph_operator_;
        fs.release();
    }

    ~LaneDetector()
    {

    }

    // Loads parameters for everything that is configurable at runtime.
    void initializeParams()
    {
        if(!ros::param::get("~homography_file", homography_file_))
        {
            ROS_WARN("Could not find parameter homography_file, using /home/auvsi/temp/homography.xml");
            homography_file_ = "/home/auvsi/temp/homography.xml";
        }

        if(!ros::param::get("~configuration_file", configuration_file_))
        {
            ROS_WARN("Could not find parameter configuration_file, using /home/auvsi/temp/configuration.xml");
            configuration_file_ = "/home/auvsi/temp/configuration.xml";
        }

        if(!ros::param::get("~source_image_topic", source_image_topic_))
        {
            ROS_WARN("Could not find parameter source_image_topic, using /camera/image_raw");
            source_image_topic_ = "/camera/image_raw";
        }

        if(!ros::param::get("~fake_scan_frame_id", fake_scan_frame_id_))
        {
            ROS_WARN("Could not find parameter fake_scan_frame_id, using fake_laser");
            fake_scan_frame_id_ = "fake_laser";
        }

        if(!ros::param::get("~fake_scan_topic", fake_scan_topic_))
        {
            ROS_WARN("Could not find parameter fake_scan_topic, using fake_scan");
            fake_scan_topic_ = "fake_scan";
        }
    }

    void filterImage()
    {
        // TODO: Work on improving filtering and line fitting.

        // Since MORPH_X options are int valued 2, 3, 4, 5, and 6
        int operation = morph_operator_ + 2;
        cv::Mat element = cv::getStructuringElement(morph_elem_,
                                                    cv::Size(2 * morph_size_ + 1, 2 * morph_size_ + 1),
                                                    cv::Point(morph_size_, morph_size_));

        // Get HSV encoded image for thresholding.
        cv::Mat src_HSV;
        cv::cvtColor(src_, src_HSV, CV_BGR2HSV);

        // Find orange.
        cv::Mat threshed_orange;
        cv::inRange(src_HSV, orange_thresh_low_, orange_thresh_high_,
                    threshed_orange);

        // Blur the HSV image.
        cv::Mat src_blur;
        cv::GaussianBlur(src_HSV, src_blur, cv::Size(5, 5), 0, 0);

        // Find white in a different image.
        cv::Mat threshed_white;
        cv::inRange(src_HSV, white_thresh_low_, white_thresh_high_,
                    threshed_white);

        // Do Morphology on orange image to blob over white portions of obstacles.
        cv::morphologyEx(threshed_orange, filtered_, operation, element);

        // "Subtract" orange detection from white detection.
        cv::bitwise_not(filtered_, filtered_);
        cv::bitwise_and(threshed_white, filtered_, filtered_);

        // Grid based fit line stuff
        // TODO: Understand this

        int block_width = filtered_.size().width / grid_cols_;
        int block_height = filtered_.size().height / grid_rows_;

        cv::Mat fitline(filtered_.rows, filtered_.cols, CV_8U);

        for (int i = 0; i < grid_cols_; i++)
        {
            for (int j = 0; j < grid_rows_; j++)
            {
                std::vector< std::vector< cv::Point > > contours;
                cv::Vec4f lines;

                cv::Rect grid(block_width * i, block_height * j, block_width, block_height);
                cv::Mat roi = filtered_(grid);
                cv::Mat fitline_roi = fitline(grid);
                cv::findContours(roi, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

                std::vector<cv::Point> points;

                std::vector< std::vector < cv::Point > >::const_iterator iter;
                for (iter = contours.begin(); iter != contours.end(); iter++)
                {
                    if (cv::contourArea((*iter)) < contour_area_threshold_)
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
                }
                else
                {
                    continue;
                }

                cv::Point2f low_left(lines[2] - lines[0] * 1000,
                                     lines[3] - lines[1] * 1000);
                cv::Point2f up_right(lines[2] + lines[0] * 1000,
                                     lines[3] + lines[1] * 1000);

                cv::line(fitline_roi, low_left, up_right, cv::Scalar(255, 0, 0), 5);
            }
        }
        filtered_ = fitline;
    }

    void warpFiltered()
    {
        warped_ = cv::Mat::zeros(filtered_.size(), CV_8UC3);
        cv::Size warped_size(filtered_.cols, filtered_.rows);

        cv::warpPerspective(filtered_, warped_, homography_matrix_, warped_size,
                            CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);
    }

    void populateScan()
    {
        // Initialize one point to track every column.
        std::vector<int> detected_points(warped_.cols, 0);
        uchar current_pixel;
        // Start and end columns so we can adjust whether we even use the full scan width.
        int start_col = 0;
        int end_col = detected_points.size() - start_col;
        int image_center = warped_.cols/2;

        // Find anywhere we hit a white pixel in the warped image.
        // Essentially passing a 'curtain' up the image from the bottom.
        for (int row = warped_.rows - 1; row > warped_.rows/2; row--)
        {
            for (int current_col = start_col; current_col < end_col; current_col++)
            {
                current_pixel = warped_.at<uchar>(row, current_col);
                if (current_pixel == 255 && detected_points[current_col] == 0)
                {
                    // current_col - warped_.cols/2 can give us x coordinate.
                    // row being stored at that position gives us the y coordinate paired to x = current_col
                    detected_points[current_col] = row;
                }
            }
        }

        // Vectors to hold the ranges and angles the points are at from bottom center
        //  of the image.
        std::vector<double> ranges(detected_points.size(), 0);
        std::vector<double> angles(detected_points.size(), 0);

        // x_distance is distance from center, y_distance is distance from bottom of warped image frame
        int x_distance, y_distance;

        for (int i = start_col; i < end_col; i++)
        {
            // X distance from center of image and distance in pixels from bottom of image.
            x_distance = i - image_center;
            y_distance = warped_.rows - detected_points[i];

            // If we detected a point along that column.
            if (detected_points[i] != 0)
            {
                // Use Pythagorean theorem to get range
                ranges[i] = cvSqrt(x_distance*x_distance + y_distance*y_distance);
                // Arctan with offset so that 0 degrees is the vertical axis in the image
                //  Positive angles are CCW from this axis
                angles[i] = cvFastArctan(y_distance, x_distance) - 90;
            }
        }

        // Set up the "scan"
        sensor_msgs::LaserScan scan;
        scan.header.stamp.sec = ros::Time::now().sec;
        scan.header.stamp.nsec = ros::Time::now().nsec;
        scan.header.frame_id = fake_scan_frame_id_;
        // Scan angle min and max in radians
        // 1.57 radians is around 90 degrees
        scan.angle_min = -1.57f;
        scan.angle_max = 1.57f;
        // Angle increment of approximately 0.33 degree in radians.
        scan.angle_increment = .00576f;
        scan.range_min = .01;
        scan.range_max = 60;
        scan.time_increment = 0.00001;
        scan.scan_time = 0.05;

        float angle_range = scan.angle_max - scan.angle_min;

        scan.ranges = std::vector<float>((unsigned long)(angle_range/scan.angle_increment));

        for (int i = 0; i < detected_points.size(); i++)
        {
            if (ranges[i] != 0)
            {
                double radians = angles[i] * CV_PI / 180.0;
                int index = (int)((radians - scan.angle_min) / scan.angle_increment);
                // Constrain index
                // TODO: This can be done without this stupid hack work on it later.
                if (index < 0) index = 0;
                if (index > scan.ranges.size() - 1) index = scan.ranges.size() - 1;

                // If the range is smaller than the current value stored at that index
                //  or if this is the first nonzero value we've found at that index
                //  then replace the scan range value for this corresponding angle.
                if (scan.ranges[index] > ranges[i] || scan.ranges[i] == 0)
                    scan.ranges[index] = ranges[i] / 100.0;
            }
        }

        laser_pub_.publish(scan);
    }

private:
    // Values obtained via parameter server.
    std::string homography_file_;
    std::string configuration_file_;
    std::string source_image_topic_;
    std::string fake_scan_frame_id_;
    std::string fake_scan_topic_;

    // ROS message handling
    ros::NodeHandle nh_;
    ros::Publisher laser_pub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber ros_image_sub_;

    // OpenCV images
    cv::Mat src_;
    cv::Mat filtered_;
    cv::Mat warped_;
    cv::Mat debug_;

    // Homography matrix
    cv::Mat homography_matrix_;

    // Threshold scalars
    cv::Scalar white_thresh_low_;
    cv::Scalar white_thresh_high_;
    cv::Scalar orange_thresh_low_;
    cv::Scalar orange_thresh_high_;

    // Morphology parameters
    int morph_elem_;
    int morph_size_;
    int morph_operator_;

    // Fitline grid values
    int grid_cols_;
    int grid_rows_;

    // Contour area limit
    int contour_area_threshold_;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            src_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            filterImage();
            warpFiltered();
            populateScan();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from %s to bgr8", msg->encoding.c_str());
        }
    }
};




/*
int morph_elem = 2;
int morph_size = 14;
int morph_operator = 2;

void initialize()
{
    // Open homography matrix storage.
    cv::FileStorage fs("/home/kenneth/temp/front_homography.xml",
        cv::FileStorage::READ);
    fs["front-homography"] >> homography_matrix;
    fs.release();
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
    cv::inRange(src_HSV, cv::Scalar(0, 125, 125), cv::Scalar(80, 255, 255),
                threshed_orange);
    cv::imshow("Orange", threshed_orange);

    // Blur the HSV image.
    cv::Mat src_blur;
    cv::GaussianBlur(src_HSV, src_blur, cv::Size(5, 5), 0, 0);

    // Find white in a different image.
    cv::Mat threshed_white;
    cv::inRange(src_HSV, cv::Scalar(0, 0, 210), cv::Scalar(255, 20, 255),
                threshed_white);
    cv::imshow("White", threshed_white);

    // Do Morphology on orange image to blob over white portions of obstacles.
    cv::morphologyEx(threshed_orange, filtered, operation, element);

    // "Subtract" orange detection from white detection.
    cv::bitwise_not(filtered, filtered);
    cv::bitwise_and(threshed_white, filtered, filtered);

    // Grid based fit line stuff
    // TODO: Understand this

    int blocks_cols = 6;
    int blocks_rows = 6;
    int block_width = filtered.size().width / blocks_cols;
    int block_height = filtered.size().height / blocks_rows;

    for (int i = 0; i < blocks_cols; i++)
    {
        for (int j = 0; j < blocks_rows; j++)
        {
            std::vector< std::vector< cv::Point > > contours;
            cv::Vec4f lines;

            cv::Rect grid(block_width * i, block_height * j, block_width, block_height);
            cv::Mat roi = filtered(grid);
            cv::findContours(roi, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

            std::vector<cv::Point> points;

            std::vector< std::vector < cv::Point > >::const_iterator iter;
            for (iter = contours.begin(); iter != contours.end(); iter++)
            {
                if (cv::contourArea((*iter)) < 50)
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
            }
            else
            {
                continue;
            }

            cv::Point2f low_left(lines[2] - lines[0] * 1000,
                                 lines[3] - lines[1] * 1000);
            cv::Point2f up_right(lines[2] + lines[0] * 1000,
                                 lines[3] + lines[1] * 1000);

            cv::line(roi, low_left, up_right, cv::Scalar(255, 0, 0), 5);
        }
    }
}

void warpFiltered()
{
    warped = cv::Mat::zeros(filtered.size(), CV_8UC3);
    cv::Size warped_size(filtered.cols, filtered.rows);

    cv::warpPerspective(filtered, warped, homography_matrix, warped_size,
                        CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

}

void warpOriginal()
{
    debug = cv::Mat::zeros(src.size(), CV_8UC3);
    cv::Size warped_size(src.cols, src.rows);

    cv::warpPerspective(src, debug, homography_matrix, warped_size,
                        CV_WARP_INVERSE_MAP | CV_INTER_LINEAR | CV_WARP_FILL_OUTLIERS);

}

void populateScan()
{
    // Initialize one point to track every column.
    std::vector<int> detected_points(warped.cols, 0);
    uchar current_pixel;
    // Start and end columns so we can adjust whether we even use the full scan width.
    int start_col = 0;
    int end_col = detected_points.size() - start_col;
    int image_center = warped.cols/2;

    // Find anywhere we hit a white pixel in the warped image.
    // Essentially passing a 'curtain' up the image from the bottom.
    for (int row = warped.rows - 1; row > warped.rows/2; row--)
    {
        for (int current_col = start_col; current_col < end_col; current_col++)
        {
            current_pixel = warped.at<uchar>(row, current_col);
            if (current_pixel == 255 && detected_points[current_col] == 0)
            {
                // current_col - warped.cols/2 can give us x coordinate.
                // row being stored at that position gives us the y coordinate paired to x = current_col
                detected_points[current_col] = row;
                // Draw a connecting line and show on the debug image.
                cv::line(debug, cv::Point(current_col, warped.rows - 1),
                    cv::Point(current_col, row), cv::Scalar(0, 255, 0), 1);
                cv::imshow("Point Finding", debug);
            }
        }
    }

    // Vectors to hold the ranges and angles the points are at from bottom center
    //  of the image.
    std::vector<double> ranges(detected_points.size(), 0);
    std::vector<double> angles(detected_points.size(), 0);

    // x_distance is distance from center, y_distance is distance from bottom of warped image frame
    int x_distance, y_distance;

    for (int i = start_col; i < end_col; i++)
    {
        // X distance from center of image and distance in pixels from bottom of image.
        x_distance = i - image_center;
        y_distance = warped.rows - detected_points[i];

        // If we detected a point along that column.
        if (detected_points[i] != 0)
        {
            // Use Pythagorean theorem to get range
            ranges[i] = cvSqrt(x_distance*x_distance + y_distance*y_distance);
            // Arctan with offset so that 0 degrees is the vertical axis in the image
            //  Positive angles are CCW from this axis
            angles[i] = cvFastArctan(y_distance, x_distance) - 90;
            // Print debugging info.
            std::cout << "Hit found at range = " << ranges[i] << ", angle = " << angles[i] << std::endl;
            std::cout << "Position x: = " << x_distance <<", y: = " << y_distance << std::endl;
        }
    }

    // Set up the "scan"
    sensor_msgs::LaserScan scan;
    scan.header.stamp.sec = ros::Time::now().sec;
    scan.header.stamp.nsec = ros::Time::now().nsec;
    scan.header.frame_id = "front_fake_laser";
    // Scan angle min and max in radians
    // .698 is approximately 40 degrees.
    scan.angle_min = -1.57f;
    scan.angle_max = 1.57f;
    // Angle increment of approximately 0.33 degree in radians.
    scan.angle_increment = .00576f;
    scan.range_min = .01;
    scan.range_max = 60;

    float angle_range = scan.angle_max - scan.angle_min;

    scan.ranges = std::vector<float>((unsigned long)(angle_range/scan.angle_increment));

    for (int i = 0; i < detected_points.size(); i++)
    {
        if (ranges[i] != 0)
        {
            double radians = angles[i] * CV_PI / 180.0;
            int index = (int)((radians - scan.angle_min) / scan.angle_increment);
            // Constrain index
            // TODO: This can be done without this stupid hack work on it later.
            if (index < 0) index = 0;
            if (index > scan.ranges.size() - 1) index = scan.ranges.size() - 1;

            if (scan.ranges[index] > ranges[i] || ranges[i] == 0)
                scan.ranges[index] = ranges[i] / 100.0;
        }
    }

    laser_pub.publish(scan);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        src = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::imshow("Original", src);
        warpOriginal();
        cv::imshow("Warped", debug);
        filterImage();
        cv::imshow("Filtered", filtered);
        warpFiltered();
        cv::imshow("Warped and Filtered", warped);
        populateScan();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from %s to bgr8", msg->encoding.c_str());
    }
}
*/



int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_detection");
    ros::NodeHandle nh("~");

    LaneDetector detector(nh);

    ros::spin();
}
