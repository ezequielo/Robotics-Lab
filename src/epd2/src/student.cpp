

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//This is the function to be filled in C3
void processImage_c3(cv::Mat &in, cv::Mat &out)
{
	double k[3][3]={{-1,-2,-1},{0,0,0},{1,2,1}};

	//cv::Mat kernel = cv::Mat(3,3,CV_32F,k);

	//cv::filter2D(in,out,-1,kernel);

	//cv::Sobel(in,out,-1,1,1);

	//cv::threshold(in,out,140.0,255.0,cv::THRESH_TOZERO_INV);
	cv::threshold(in,out,30.0,255.0,cv::THRESH_BINARY);

	//cv::erode(out,out,cv::Mat());
	//cv::dilate(out,out,cv::Mat());

	
}


//This is the function to be filled in C4
void processImageColor_c4(cv::Mat &in, cv::Mat &out)
{

	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;

	cv::Mat o[] = { r, g, b, alpha };

	//in is a color image. We split it into the 3 colors and the alpha channel:
	cv::split(in,o);

	//From now on, we process just one of the channels. For instance, o[2] is the blue channel

	//This is an example:
	cv::filter2D(o[2],out,-1,cv::getGaussianKernel(5,2));

	
	

	
}
