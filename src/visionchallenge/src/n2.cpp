
#include <stdio.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "student.h"
#include <fstream>

sensor_msgs::CameraInfo cameraCalibration;
std::ofstream outfile;

void callback(const sensor_msgs::Image::ConstPtr& colorMsg, const sensor_msgs::Image::ConstPtr& depthMsg) {

    cv_bridge::CvImageConstPtr bColor;
    cv::Mat color;
    cv::Mat out_frame;

    // Convertidor de mensaje ROS a imagen OpenCV por medio de la funcion cv_bridge
    try {
        bColor = cv_bridge::toCvShare(colorMsg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bColor->image.copyTo(color);
    processImageColor_c4(color, out_frame);

    // get col and row using moments
    cv::Moments moments = cv::moments(out_frame, 1);
    double col = moments.m10 / moments.m00;
    double row = moments.m01 / moments.m00;
    
    if (!isnan(col) and !isnan(row) ) {
    
        cv::circle(color, cv::Point(col, row), 10, cv::Scalar(0, 0, 255), 3);
        std::cout << "X: " << col << ", Y:" << row << std::endl;

        // results to file
        outfile.open("res2.txt", std::ios_base::app);
        outfile << colorMsg->header.stamp.sec << "\t" << colorMsg->header.stamp.nsec << "\t" << row << "\t" << col;
        outfile << std::endl;
        outfile.close();

        // display results
        cv::imshow("Original Frame", color);
        cv::imshow("Thresholded Frame", out_frame);
    }

    cv::waitKey(5);
}

void getCalibration(const sensor_msgs::CameraInfo::ConstPtr& calib) {

    cameraCalibration = *calib;

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "n1");
    ros::NodeHandle n;

    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, "camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_disp(n, "/camera/depth_registered/image_raw", 1);

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_image, sub_disp);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    //subscribes to the Kinect video frames
    ros::Subscriber calibration = n.subscribe("/camera/rgb/camera_info", 1, getCalibration);

    ros::spin();

    return 0;
}
