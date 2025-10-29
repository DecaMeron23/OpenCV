#include <opencv2/opencv.hpp>
#include <istream>


cv::Mat img , imgHSV , mask;
int hmin = 0 , smin = 110 , vmin = 153; 
int hmax = 19 , smax = 240 , vmax = 255; 


int main()
{
    std::string path = "immagini/image.png";
    
    img = cv::imread(path);
    
    cv::cvtColor(img , imgHSV , cv::COLOR_BGR2HSV);

    cv::namedWindow("Trackbar" , (640 , 200));
    cv::createTrackbar("Hue min" , "Trackbar" , &hmin , 179);
    cv::createTrackbar("Hue max" , "Trackbar" , &hmax , 179);
    cv::createTrackbar("Sat min" , "Trackbar" , &smin , 255);
    cv::createTrackbar("Sat max" , "Trackbar" , &smax , 255);
    cv::createTrackbar("Val min" , "Trackbar" , &vmin , 255);
    cv::createTrackbar("Val max" , "Trackbar" , &vmax , 255);


    while (true)
    {
        cv::Scalar lower(hmin, smin , vmin);
        cv::Scalar upper(hmax, smax , vmax);

        cv::inRange(imgHSV , lower , upper , mask);

        cv::imshow("Immagine", img);
        cv::imshow("Immagine HSV", imgHSV);
        cv::imshow("Mask", mask);
        cv::waitKey(1);
    }
    return 0;
}