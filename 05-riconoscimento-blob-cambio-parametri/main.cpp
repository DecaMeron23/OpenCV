#include "opencv2/opencv.hpp"
#include "iostream"
#include "thread"
#include <stdlib.h>

std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame);
void neagateFrame(cv::Mat &original, cv::Mat &destiniation);

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

int main()
{
    // Definizione nomi delle window

    std::string win_plot = "blob finder", win_bar = "Trackbar";
    // Apertura fotocamera
    cv::VideoCapture vc = cv::VideoCapture(0);

    if (!vc.isOpened())
    {
        return -1;
    }

    // Dichiarazine delle immagini che useremo
    cv::Mat frame, gray_scale , gray_scale_neg;

    // Window per i plot
    cv::namedWindow(win_plot);

    // Window per le trackbar
    cv::namedWindow(win_bar);

    cv::createTrackbar("Area Minima", win_bar, &areaMinima, MAX_AREA_MINIMA);
    cv::createTrackbar("Cricolarità", win_bar, &circolarita, MAX_CIRCOLARITA);
    cv::createTrackbar("Convessità", win_bar, &convessita, MAX_CONVESSITA);
    cv::createTrackbar("Inertia", win_bar, &inertia, MAX_INERTIA);


    while (true)
    {
        // lettora del frame
        vc.read(frame);

        // prerpocessing: cambio colore e aggiunta blur
        cv::cvtColor(frame, gray_scale, cv::COLOR_BGR2GRAY);

        neagateFrame(gray_scale , gray_scale_neg);

        // cv::medianBlur(gray_scale, gray_scale, kernel_size);
        // cv::medianBlur(gray_scale_neg, gray_scale_neg, kernel_size);

        // blob finder
        std::vector<cv::KeyPoint> blob = blob_finder(gray_scale);
        std::vector<cv::KeyPoint> blob_neg = blob_finder(gray_scale_neg);

        // disegno i blob trovati
        cv::drawKeypoints(frame, blob, frame, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(frame, blob_neg, frame, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // show dell'immagine
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

void neagateFrame(cv::Mat &original, cv::Mat &destiniation)
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