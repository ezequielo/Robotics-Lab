


#include <stdio.h>
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "student.h"
      
 

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edp3_c1"); 

  cv::Mat input_image = cv::imread(argv[1]); //Read an image from file. 
  cv::Mat template_image = cv::imread(argv[2]); //Read an image from file. 
 
  cv::Mat output_image;

  searchTemplate(input_image,template_image,output_image); //This is the function to be filled. In file student.cpp

  cv::imshow("input_image", input_image); //Show a window with the input image
  cv::imshow("template", template_image); //Show a window with the template image

  cv::imshow("output_image", output_image); //Show a window with the output image

  cv::imwrite("output.jpg",output_image); //Save the output image to file
  cv::waitKey(); //Wait for a key

  return 0;
}
