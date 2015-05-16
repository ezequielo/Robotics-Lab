

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void processImageColor_c4(cv::Mat &in, cv::Mat &out)
{

	cv::Mat r;
	cv::Mat g;
	cv::Mat b;
	cv::Mat alpha;
	cv::Mat rojo;
	cv::Mat verde;
	cv::Mat azul;

	cv::Mat o[] = { r, g, b, alpha };

	//in is a color image. We split it into the 3 colors and the alpha channel:
	cv::split(in,o);

	//From now on, we process just one of the channels. For instance, o[2] is the blue channel

	//This is an example:
	//cv::filter2D(o[2],out,-1,cv::getGaussianKernel(5,2));
    //cv::threshold(o[0],out,70.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[0],rojo,200.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[1],verde,120.0,255.0,cv::THRESH_BINARY);
	cv::threshold(o[2],azul,200.0,255.0,cv::THRESH_BINARY_INV);
		
	cv::bitwise_and(rojo,azul,out);
	cv::erode(out,out,cv::Mat());
	cv::erode(out,out,cv::Mat());
	//cv::erode(out,out,cv::Mat());
	//cv::erode(out,out,cv::Mat());
	//cv::erode(out,out,cv::Mat());
	cv::dilate(out,out,cv::Mat());
	cv::dilate(out,out,cv::Mat());
	cv::dilate(out,out,cv::Mat());
	cv::dilate(out,out,cv::Mat());
	cv::dilate(out,out,cv::Mat());

	cv::bitwise_and(out,verde,out);


	//cv::bitwise_and(out3,out,out);	
}

