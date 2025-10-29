#include "opencv2/opencv.hpp"
#include "iostream"

using namespace cv;

int main()
{
    Mat image = imread("image.png", IMREAD_GRAYSCALE);
    if (image.empty()) return -1;
 
    // Apply Gaussian blur
    Mat blurred;
    GaussianBlur(image, blurred, Size(0, 0), 2);
 
    // Apply Laplacian
    Mat laplacian;
    Laplacian(blurred, laplacian, CV_64F);
 
    // Convert to displayable format
    Mat result;
    convertScaleAbs(laplacian, result);

    // Set up parameters
    SimpleBlobDetector::Params params;
    params.minThreshold = 0;
    params.maxThreshold = 254;
    params.filterByArea = false;
    params.minArea = 10;
    params.filterByCircularity = false;
    params.minCircularity = 0.5;
    params.filterByConvexity = false;
    params.minConvexity = 0.9;
    params.filterByInertia = false;
    params.minInertiaRatio = 0.5;

    // Create detector and detect blobs
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    std::vector<KeyPoint> keypoints;
    detector->detect(result, keypoints);


    Mat output;

    drawKeypoints(image, keypoints , output, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // Display result
    imshow("Blobs Detected", output);
    waitKey(0);
    return 0;
}
