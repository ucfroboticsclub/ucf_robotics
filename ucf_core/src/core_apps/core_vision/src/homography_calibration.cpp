#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <sensor_msgs/image_encodings.h>

class Homography {
  private:
  public:

    static const double box_width = 2.54;
    static const double box_height = 2.54;

    static cv::Mat process(cv::Mat& rgba_image)
    {

      cv::Mat original_;
      cv::Mat img_copy_;
      cv::Mat img_warp_;
      cv::Mat gray_img_;

      std::vector<cv::Point2f> corners;
      img_warp_ = cv::Mat(rgba_image.rows, rgba_image.cols, CV_8UC4);
      gray_img_ = cv::Mat(rgba_image.rows, rgba_image.cols, CV_8UC4);

      int chess_rows = 6;
      int chess_cols = 9;

      img_copy_ = rgba_image.clone();

      cv::Size pattern_size = cv::Size(chess_rows, chess_cols);
      cv::cvtColor(img_copy_, gray_img_, cv::COLOR_BGR2GRAY);
      bool found_corners = cv::findChessboardCorners(gray_img_, pattern_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

      if(found_corners)
      {
        cv::cornerSubPix(gray_img_, corners, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, .1));

        cv::Mat obj_pts = cv::Mat();
        cv::Mat img_pts = cv::Mat();

        std::vector<cv::Point> src_pts;
        std::vector<cv::Point> dst_pts;
        double width = box_width*(pattern_size.width + 1); // centimeters
        double height = box_height*(pattern_size.height + 1); // centimeters

        // 4 Corners of physical chessboard image.
        src_pts.push_back(cv::Point(img_copy_.cols/2 - width/2, img_copy_.rows - height));
        src_pts.push_back(cv::Point(img_copy_.cols/2 + width/2, img_copy_.rows - height));
        src_pts.push_back(cv::Point(img_copy_.cols/2 - width/2, img_copy_.rows));
        src_pts.push_back(cv::Point(img_copy_.cols/2 + width/2, img_copy_.rows));
        // 4 Corners of chessboard found in image.
        dst_pts.push_back(corners.at(0));
        dst_pts.push_back(corners.at((int)(pattern_size.width) - 1));
        dst_pts.push_back(corners.at((int)(pattern_size.width) * ((int)(pattern_size.height) - 1)));
        dst_pts.push_back(corners.at((int)(corners.size()) - 1));

        obj_pts = cv::Mat(src_pts);
        img_pts = cv::Mat(dst_pts);
        obj_pts.convertTo(obj_pts, CV_32F);
        img_pts.convertTo(img_pts, CV_32F);

        cv::Mat h = cv::getPerspectiveTransform(obj_pts, img_pts);

        cv::FileStorage file("homMatrix.xml", cv::FileStorage::WRITE);
        file << "homography" << h;

        cv::circle( img_copy_, dst_pts.at(0), 9, cv::Scalar(0,0,255), 3);
        cv::circle( img_copy_, dst_pts.at(1), 9, cv::Scalar(0,255,0), 3);
        cv::circle( img_copy_, dst_pts.at(2), 9, cv::Scalar(255,0,0), 3);
        cv::circle( img_copy_, dst_pts.at(3), 9, cv::Scalar(255,255,0), 3);
        img_warp_ = cv::Mat::zeros(img_copy_.size(), CV_8UC4);

        cv::warpPerspective(img_copy_,img_warp_,h,img_copy_.size(), cv::INTER_LINEAR | CV_WARP_FILL_OUTLIERS | CV_WARP_INVERSE_MAP);

        return img_warp_;
      }

        return gray_img_;


    }



};



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{


  cv_bridge::CvImagePtr orig_ptr;

  try
  {
    orig_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8 );
    
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    return;
  }


  cv::imshow("original", orig_ptr->image);
  cv::imshow("homography", Homography::process(orig_ptr->image));


}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "homography_calibration");
  ros::NodeHandle nh;

  cv::namedWindow("original");
  cv::namedWindow("homography");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("front_camera/image_raw", 1, imageCallback);
  
  for(;;)
  {
    ros::spinOnce();
    if(cv::waitKey(30) >= 0) break;
  }

  cv::destroyWindow("view");
}