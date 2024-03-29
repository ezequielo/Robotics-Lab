
/*
*
*	EPD3 C2 / Depth information and feature extraction and tracking
*
*/

#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "student.h"

  cv::Mat template_image;

    
  void imageCb(const sensor_msgs::ImageConstPtr & msg)
  {
    cv::Mat rgb_frame; //Input image in matrix form
    cv::Mat out_frame; //Output image

    cv_bridge::CvImagePtr cv_ptr_rgb; //cv_bridge::CvImagePtr is a pointer type that points to NULL by default. You have to allocate storage before you can actually use it
    try
    {
      cv_ptr_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //make a copy of the ROS message data. //bgra8: BGR color image with an alpha channel
                                                         //Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    rgb_frame = cv_ptr_rgb->image; //Here we have the current frame in OpenCV Mat format
    cv::imwrite("output.jpg",rgb_frame); // Save image

    searchTemplate(rgb_frame,template_image,out_frame);
   
    cv::imshow("output_image", out_frame); //Show a window with the output image
    cv::waitKey(3); //Wait for 3 milliseconds
  }
    
 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edp2_c4"); 

  template_image = cv::imread(argv[1]); //Read an image from file.

  ros::NodeHandle nh_;
  
  image_transport::ImageTransport it_(nh_);

  image_transport::Subscriber image_sub_;

  image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, imageCb); //subscribes to the Kinect video frames

  ros::spin();
  return 0;
}
