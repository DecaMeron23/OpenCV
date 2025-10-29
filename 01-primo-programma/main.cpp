#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
int main()

{
    std::string image_path = "image.png";
    Mat img = imread(image_path, IMREAD_COLOR);

    imshow("Display window", img);
    int k = waitKey(0); // Wait for a keystroke in the window
    return 0;
}