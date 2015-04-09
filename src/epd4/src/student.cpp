

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/features2d/features2d.hpp"

#include "opencv2/nonfree/features2d.hpp"

#include "opencv2/nonfree/nonfree.hpp"

#include <sensor_msgs/CameraInfo.h>

cv::Mat prev_img; 
std::vector<cv::KeyPoint> prev_keypoints;
cv::Mat prev_descriptors;
bool isthere_prev_img=false;

extern sensor_msgs::CameraInfo cameraCalibration;


/**
* OpenCV image coordinate frame: u corresponds to the column (and the X axis), v corresponds to the rows
* of the image (and the Y axis)
*/
cv::Vec3f transfromToWorldCoordinates(float u, float v, float depth, sensor_msgs::CameraInfo calib)
{
	float X, Y, Z;
	cv::Vec3f point3D;

	//We apply the formulae

	std::cout << "u: " << u << " v: " << v << " d: " <<  depth << std::endl;	

	X=(u-calib.K[2])*depth/calib.K[0];
	Y=(v-calib.K[5])*depth/calib.K[4];
	Z=depth;

	std::cout << "X: " << X << " Y: " << Y << " Z: " <<  Z << std::endl;

	point3D[0]=X;
	point3D[1]=Y;
	point3D[2]=Z;
	
	return point3D;

}

/*
cv::Vec3f thresholdDepth(float u, float v, float depth, sensor_msgs::CameraInfo calib)
{
	float X, Y, Z;
	cv::Vec3f point3D;

	//We apply the formulae

	std::cout << "u: " << u << " v: " << v << " d: " <<  depth << std::endl;	

	X=(u-calib.K[2])*depth/calib.K[0];
	Y=(v-calib.K[5])*depth/calib.K[4];
	Z=depth;

	std::cout << "X: " << X << " Y: " << Y << " Z: " <<  Z << std::endl;

	point3D[0]=X;
	point3D[1]=Y;
	point3D[2]=Z;
	
	cv::threshold(o[0],rojo,70.0,110.0,cv::THRESH_BINARY);
	
	return point3D;

}
*/


void processImageColorWithDepth(cv::Mat &in, cv::Mat &depth, cv::Mat &out)
{
	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;
	cv::Mat rojo;
        cv::Mat azul;
	cv::Mat depth1;
	cv::Mat depth2;
	cv::Mat depth3;

	cv::Mat o[] = { r, g, b, alpha };

	//in is a color image. We split it into the 3 colors and the alpha channel:
	cv::split(in,o);

	//From now on, we process just one of the channels. For instance, o[2] is the blue channel

	//This is an example:
	cv::threshold(o[0],rojo,70.0,110.0,cv::THRESH_BINARY);
	cv::threshold(o[2],azul,90.0,110.0,cv::THRESH_TOZERO_INV);
	cv::bitwise_and(rojo, azul, out);

	cv::threshold(depth,depth2,0.5,255.0,cv::THRESH_BINARY);
	cv::threshold(depth,depth1,1.0,255.0,cv::THRESH_TOZERO_INV);
	//cv::bitwise_and(depth1, depth2, depth3);


	depth2.convertTo(depth2,CV_8U);
	cv::bitwise_and(depth2, out, out);


	// Muestra imagen mono
	cv::imshow("out", out);
	cv::waitKey(5);
}



void trackFeatures(cv::Mat &img_1, cv::Mat &mask)
{

	//-- Step 1: Detect the keypoints using SURF Detector
  	int minHessian = 400;
        
	std::vector<cv::KeyPoint> new_keypoints;
	cv::Mat new_descriptors;
	cv::SurfFeatureDetector detector( minHessian );
	cv::SurfDescriptorExtractor extractor;

	if(!isthere_prev_img)
    	{
		//-- Step 1: Detect the keypoints using SURF Detector. maks specifies where to look for keypoints (optional). 
		//  It must be a 8-bit integer matrix with non-zero values in the region of interest.
		detector.detect( img_1, prev_keypoints,mask);

		//-- Step 2: Calculate descriptors (feature vectors)
		extractor.compute( img_1, prev_keypoints, prev_descriptors);

		prev_img=img_1;
		isthere_prev_img = true;
	}else
	{

		//-- Step 1: Detect the keypoints using SURF Detector
		detector.detect( img_1, new_keypoints,mask);

		//-- Step 2: Calculate descriptors (feature vectors)
  		
		extractor.compute( img_1, new_keypoints, new_descriptors); 	

		//-- Step 3: Matching descriptor vectors using the BruteForce matcher with the previous image
  	
		cv::BFMatcher matcher(cv::NORM_L2,false);
  		std::vector< cv::DMatch > matches;
  		matcher.match( prev_descriptors, new_descriptors, matches );

		//-- Draw matches
  		cv::Mat img_matches;
  		cv::drawMatches( prev_img, prev_keypoints, img_1, new_keypoints,
               		matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

		prev_descriptors=new_descriptors;
		prev_keypoints=new_keypoints;
		prev_img=img_1;

  		//-- Show detected matches
  		imshow( "Good Matches", img_matches );
	}

}


void processImageAndDepth(cv::Mat &img, cv::Mat &depth, cv::Mat &out)
{

	//out = depth;

	cv::threshold(depth,out,1.5,255.0,cv::THRESH_BINARY_INV);

	
}

