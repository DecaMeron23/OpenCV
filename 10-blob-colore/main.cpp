#include "opencv2/opencv.hpp"
#include "iostream"
#include "thread"
#include <stdlib.h>

std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame);
void negateFrame(cv::Mat &original, cv::Mat &destiniation);

// void change_parameter();
// int analize_webcam();

int areaMinima = 50;  //[px]
int kernel_size = 3;  //[px]
int circolarita = 60; //[%]
int convessita = 20;  //[%]
int inertia = 1;      //[%]

const int MAX_AREA_MINIMA = 1000; //[px]
const int MAX_CIRCOLARITA = 100;  //[%]
const int MAX_CONVESSITA = 100;   //[%]
const int MAX_INERTIA = 100;      //[%]


int hmin = 0 , smin = 0, vmin = 0; 
int hmax = 179 , smax = 255 , vmax = 255; 


int main()
{
    // Definizione nomi delle window

    std::string win_plot = "blob finder", win_bar = "Trackbar" , win_hsv = "HSV";
    // Apertura fotocamera
    cv::VideoCapture vc = cv::VideoCapture(0);

    if (!vc.isOpened())
    {
        return -1;
    }

    // Dichiarazine delle immagini che useremo
    cv::Mat frame, imgHSV , mask , gray;

    // Window per i plot
    cv::namedWindow(win_plot);

    // Window per le trackbar
    cv::namedWindow(win_bar);

    cv::createTrackbar("Area Minima", win_bar, &areaMinima, MAX_AREA_MINIMA);
    cv::createTrackbar("Cricolarità", win_bar, &circolarita, MAX_CIRCOLARITA);
    cv::createTrackbar("Convessità", win_bar, &convessita, MAX_CONVESSITA);
    cv::createTrackbar("Inertia", win_bar, &inertia, MAX_INERTIA);


    cv::namedWindow(win_hsv);

    cv::createTrackbar("Hue min" , win_hsv , &hmin , 179);
    cv::createTrackbar("Hue max" , win_hsv , &hmax , 179);
    cv::createTrackbar("Sat min" , win_hsv , &smin , 255);
    cv::createTrackbar("Sat max" , win_hsv , &smax , 255);
    cv::createTrackbar("Val min" , win_hsv , &vmin , 255);
    cv::createTrackbar("Val max" , win_hsv , &vmax , 255);
    cv::createTrackbar("Kernel Size" , win_hsv , &kernel_size , 20);


    while (true)
    {
        // lettora del frame
        vc.read(frame);

        // prerpocessing: cambio colore e aggiunta blur
        cv::cvtColor(frame, imgHSV, cv::COLOR_BGR2HSV);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // cv::medianBlur(gray_scale_neg, gray_scale_neg, kernel_size);


        cv::Scalar lower(hmin, smin , vmin);
        cv::Scalar upper(hmax, smax , vmax);

        cv::inRange(imgHSV , lower , upper , mask);

        cv::medianBlur(mask , mask , 2*kernel_size+1);
        cv::medianBlur(gray , gray , 2*kernel_size+1);
        // cv::GaussianBlur(mask , mask , cv::Size(kernel_size*2+1,kernel_size*2+1) , 2 , 2);

        cv::Mat mask_neg;
        negateFrame(mask, mask_neg);

        // blob finder
        std::vector<cv::KeyPoint> blob = blob_finder(mask_neg);
        // std::vector<cv::KeyPoint> blob_neg = blob_finder(gray_scale_neg);

        // disegno i blob trovati
        cv::drawKeypoints(frame, blob, frame, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        // cv::drawKeypoints(frame, blob_neg, frame, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 30, 1, 100 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
        );

        for( size_t i = 0; i < circles.size(); i++ )
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // circle center
            circle( gray, center, 1, cv::Scalar(0,100,100), 5, cv::LINE_AA);
            // circle outline
            int radius = c[2];
            circle( gray, center, radius, cv::Scalar(255,0,255), 5, cv::LINE_AA);
        }


        // show dell'immagine
        cv::imshow("HSV immagie" , mask);
        cv::imshow(win_plot, frame);        
        cv::waitKey(10);
    }
}

std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame)
{
    // Set up parameters
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 10;
    params.maxThreshold = 200;
    params.filterByArea = true;
    params.minArea = areaMinima;
    params.filterByCircularity = true;
    params.minCircularity = circolarita / 100;
    params.filterByConvexity = true;
    params.minConvexity = convessita / 100;
    params.filterByInertia = true;
    params.minInertiaRatio = inertia / 100;

    // Create detector and detect blobs
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(frame, keypoints);
    return keypoints;
}

void negateFrame(cv::Mat &original, cv::Mat &destiniation)
{
    destiniation = 255 - original;
}

// int main()
// {
//     std::thread webcam(analize_webcam);
//     std::thread cli(change_parameter);

//     webcam.join();
//     cli.join();

//     return 0;
// }

// int analize_webcam()
// {
//     cv::VideoCapture webcam = cv::VideoCapture(0);
//     if (!webcam.isOpened())
//     {
//         std::cerr << "Non si è riusciti ad aprire la webcam" << std::endl;
//         return -1;
//     }

//     while (true)
//     {
//         cv::Mat original_frame;
//         cv::Mat gray_scale_frame;

//         webcam.read(original_frame);

//         // original_frame = cv::imread("image.png");

//         if (original_frame.empty())
//         {
//             std::cerr << "frame vuoto... saltato" << std::endl;
//             continue;
//         }

//         cv::cvtColor(original_frame, gray_scale_frame, cv::COLOR_RGB2GRAY);

//         cv::Mat gray_scale_frame_neg;

//         // neagateFrame(gray_scale_frame, gray_scale_frame_neg);

//         std::vector<cv::KeyPoint> keypoints = blob_finder(gray_scale_frame);
//         // std::vector<cv::KeyPoint> keypoints_neg = blob_finder(gray_scale_frame_neg);

//         // Draw the keypoints
//         cv::Mat output;
//         cv::drawKeypoints(original_frame, keypoints, output, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//         // cv::drawKeypoints(output, keypoints_neg, output, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

//         // Display result
//         cv::imshow("Blobs Detected", output);
//         cv::waitKey(1);
//     }

// void change_parameter()
// {
//     char tipo;
//     float valore;
//     while(true)
//     {
//         std::cout<<"Quale Parametro: ";
//         std::cin >>tipo;
//         std::cout<<"Valore: ";
//         std::cin>>valore;

//         switch (tipo)
//         {
//         case 'a':
//             areaMinima = valore;
//             break;
//         case 'c':
//             circolarita = valore;
//             break;
//         case 'C':
//             convessita = valore;
//             break;
//         case 'i':
//             inertia = valore;
//             break;
//         }

//     }
// }
// }
