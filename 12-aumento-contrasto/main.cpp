#include "opencv2/opencv.hpp"
#include "iostream"
#include <stdlib.h>

std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame);
void negateFrame(cv::Mat &original, cv::Mat &destiniation);

int areaMinima = 50;  //[px]
int kernel_size = 3;  //[px]
int circolarita = 60; //[%]
int convessita = 20;  //[%]
int inertia = 1;      //[%]

const int MAX_AREA_MINIMA = 1000; //[px]
const int MAX_CIRCOLARITA = 100;  //[%]
const int MAX_CONVESSITA = 100;   //[%]
const int MAX_INERTIA = 100;      //[%]


int dp = 1, minDist = 10 , par1 = 100 , par2 = 100 , minRad = 10 , maxRad = 100;



int main()
{
    // Definizione nomi delle window

    std::string win_plot = "blob finder", win_bar_blob = "Trackbar - BLOB",  win_bar_hough = "Trackbar - Hough";
    // Apertura fotocamera
    cv::VideoCapture vc = cv::VideoCapture(0);

    if (!vc.isOpened()){
        return -1;
    }


    // Dichiarazine delle immagini che useremo
    cv::Mat frame, gray , gray_neg;

    // Window per i plot
    cv::namedWindow(win_plot);

    // Window per le trackbar
    cv::namedWindow(win_bar_blob);

    cv::createTrackbar("Area Minima", win_bar_blob, &areaMinima, MAX_AREA_MINIMA);
    cv::createTrackbar("Cricolarità", win_bar_blob, &circolarita, MAX_CIRCOLARITA);
    cv::createTrackbar("Convessità", win_bar_blob, &convessita, MAX_CONVESSITA);
    cv::createTrackbar("Inertia", win_bar_blob, &inertia, MAX_INERTIA);


    cv::namedWindow(win_bar_hough);

    cv::createTrackbar("Rapporto Scala", win_bar_hough, &dp, 10);
    cv::createTrackbar("Distanza Minima", win_bar_hough, &minDist, 500);
    cv::createTrackbar("Parametro 1", win_bar_hough, &par1, 500);
    cv::createTrackbar("Parametro 2", win_bar_hough, &par2, 500);
    cv::createTrackbar("Raggio Minimo", win_bar_hough, &minRad, 500);
    cv::createTrackbar("Raggio Massimo", win_bar_hough, &maxRad, 500);


    while (true)
    {
        // lettora del frame
        vc.read(frame);

        // frame = cv::imread("../image.png");


        // PREPROCESSING
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray , gray , cv::Size(kernel_size*2+1,kernel_size*2+1) , 2 , 2);
        // cv::medianBlur(gray, gray, 2 * kernel_size + 1);
        cv::equalizeHist(gray , gray);


        negateFrame(gray , gray_neg);

        // RICERCA DEI BLOB
        std::vector<cv::KeyPoint> blob = blob_finder(gray);
        std::vector<cv::KeyPoint> blob_neg = blob_finder(gray_neg);

        std::vector<cv::Vec4f> cerchi , cerchi_neg;
        cv::HoughCircles(gray, cerchi ,cv::HOUGH_GRADIENT , 1+dp , 1+minDist , 1+par1 , 1+par2 , minRad , maxRad );
        cv::HoughCircles(gray_neg, cerchi_neg ,cv::HOUGH_GRADIENT , 1+dp , 1+minDist , 1+par1 , 1+par2 , minRad , maxRad );

        for (size_t i = 0 ; i < cerchi.size() ; ++i)
        {
            cv::Vec4f cerchio = cerchi[i];
            cv::circle(frame , cv::Point(cerchio[0] , cerchio[1]) , cerchio[2] , cv::Scalar(12 , 255 , 0) , 2 , cv::LINE_AA);
            cv::circle(frame , cv::Point(cerchio[0] , cerchio[1]) , 2 , cv::Scalar(0 , 0 , 0) , 2 , cv::LINE_AA);
            cv::putText(frame , ("Voto: " + std::to_string(cerchio[3])) ,  cv::Point(cerchio[0] , cerchio[1]) , cv::FONT_HERSHEY_DUPLEX , 0.5 , cv::Scalar(12 , 255 , 0));
        }

        for (size_t i = 0 ; i < cerchi_neg.size() ; ++i)
        {
            cv::Vec4f cerchio = cerchi_neg[i];
            cv::circle(frame , cv::Point(cerchio[0] , cerchio[1]) , cerchio[2] , cv::Scalar(12 , 255 , 0) , 2 , cv::LINE_AA);
            cv::circle(frame , cv::Point(cerchio[0] , cerchio[1]) , 2 , cv::Scalar(0 , 0 , 0) , 2 , cv::LINE_AA);
            cv::putText(frame , ("Voto: " + std::to_string(cerchio[3])) ,  cv::Point(cerchio[0] , cerchio[1]) , cv::FONT_HERSHEY_DUPLEX , 0.5 , cv::Scalar(12 , 255 , 0));
        }

        // PLOT DEI BLOB
        cv::drawKeypoints(frame, blob, frame, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::drawKeypoints(frame, blob_neg, frame, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        cv::imshow("gray" , gray );
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
