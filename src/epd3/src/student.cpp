

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



void searchTemplate(cv::Mat &in, cv::Mat &temp, cv::Mat &out)
{

	double min, max;
	cv::Point pointMin, pointMax;

	//Create similarity matrix between the template, temp, and the input image
	cv::matchTemplate(in,temp,out,CV_TM_CCORR_NORMED);

	//Look for the maxima in the similarity matrix
	cv::minMaxLoc(out,&min,&max,&pointMin,&pointMax);

	//Once we have the maxima located, we copy the input image in the output parameter
	in.copyTo(out);
    // out=in; // no copia las imagenes, se crea una referencia y libera la memoria de la variable

	//Draw a circle on the output image at the location of the maxima, and a rectangle with the dimensions of the template temp
	//cv::circle(out,pointMax,10,cv::Scalar(0,0,255),3);
        cv::rectangle(out,pointMax,cv::Point(pointMax.x+temp.cols,pointMax.y+temp.rows),cv::Scalar(0,0,255),3);

}








