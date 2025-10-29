#include "opencv2/opencv.hpp"
#include <iostream>

int main()
{
    cv::VideoCapture cap = cv::VideoCapture(0);

    while (true)
    {
        cv::Mat frame;
        cap.read(frame);
        cv::imshow( "WebCam" ,frame);
        cv::waitKey(1);
    }
    

    return 0;

}