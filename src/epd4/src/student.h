


cv::Vec3f transfromToWorldCoordinates(float u, float v, float depth, sensor_msgs::CameraInfo calib);

void findFeatures(cv::Mat &in, cv::Mat &out);
void processImageColorWithDepth(cv::Mat &in, cv::Mat &depth, cv::Mat &out);

void trackFeatures(cv::Mat &img_1, cv::Mat &img_2);
void processImageAndDepth(cv::Mat &img, cv::Mat &depth, cv::Mat &out);

