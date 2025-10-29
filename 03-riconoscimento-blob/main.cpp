#include "opencv2/opencv.hpp"
#include "iostream"


std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame);
void neagateFrame(cv::Mat &original , cv::Mat &destiniation);


int main()
{
    cv::VideoCapture webcam = cv::VideoCapture(0);

    if (!webcam.isOpened())
    {
        std::cerr<<"Non si è riusciti ad aprire la webcam"<<std::endl;
        return -1;
    }

    while(true)
    {
        cv::Mat original_frame;
        cv::Mat gray_scale_frame;
        
        
        webcam.read(original_frame);

        // original_frame = cv::imread("image.png");

        if(original_frame.empty())
        {
            std::cerr<<"frame vuoto... saltato"<<std::endl;
            continue;
        }

        cv::cvtColor(original_frame , gray_scale_frame , cv::COLOR_RGB2GRAY);

        cv::Mat gray_scale_frame_neg;

        neagateFrame(gray_scale_frame , gray_scale_frame_neg);


        std::vector<cv::KeyPoint> keypoints = blob_finder(gray_scale_frame);
        std::vector<cv::KeyPoint> keypoints_neg = blob_finder(gray_scale_frame_neg);
    
        // Draw the keypoints
        cv::Mat output;
        cv::drawKeypoints(original_frame, keypoints, output, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(output, keypoints_neg, output, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
        // Display result
        cv::imshow("Blobs Detected", output);
        cv::waitKey(1);
        
    }

    return 0;
}

std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame)
{
    // Set up parameters
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = 10;
    params.filterByCircularity = true;
    params.minCircularity = 0.5;
    params.filterByConvexity = true;
    params.minConvexity = 0.5;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.01;

    // Create detector and detect blobs
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(frame, keypoints);
    return keypoints;
}

void neagateFrame(cv::Mat &original , cv::Mat &destiniation){
    destiniation = 255-original;
}
