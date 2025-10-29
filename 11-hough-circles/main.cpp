#include "opencv2/opencv.hpp"
#include "iostream"
#include <stdlib.h>

// std::vector<cv::KeyPoint> blob_finder(cv::Mat &frame);
void negateFrame(cv::Mat &original, cv::Mat &destiniation);

int raggio_min = 1;  //[px]
int raggio_max = 10; //[px]
int valore = 15;

const int MAX_RAGGIO = 1000; //[px]
const int MIN_RAGGIO = 1000; //[px]
const int MAX_VALORE = 99;   //[px]

int main()
{
    // Definizione nomi delle window

    std::string win_plot = "Hough Circles", win_bar = "Trackbar";
    // Apertura fotocamera
    cv::VideoCapture vc = cv::VideoCapture(0);

    if (!vc.isOpened())
    {
        return -1;
    }

    // Dichiarazine delle immagini che useremo
    cv::Mat original, gray;

    // Window per i plot
    cv::namedWindow(win_plot);

    // Window per le trackbar
    cv::namedWindow(win_bar);

    cv::createTrackbar("Raggio Minimo", win_bar, &raggio_min, MIN_RAGGIO);
    cv::createTrackbar("Raggio Massimo", win_bar, &raggio_max, MAX_RAGGIO);
    cv::createTrackbar("Valore", win_bar, &valore, MAX_VALORE);

    while (true)
    {
        // lettora del frame
        vc.read(original);
        // original = cv::imread("../image.png");

        // prerpocessing: cambio colore e aggiunta blur
        cv::cvtColor(original, gray, cv::COLOR_BGR2GRAY);

        cv::medianBlur(gray, gray, 3);
        // cv::GaussianBlur(gray, gray, cv::Size(7, 7), 1.5, 1.5);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                         gray.rows / (1 + valore),       // change this value to detect circles with different distances to each other
                         100, 30, raggio_min, raggio_max // change the last two parameters
                                                         // (min_radius & max_radius) to detect larger circles
        );

        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Vec3i c = circles[i];
            cv::Point center = cv::Point(c[0], c[1]);
            // circle center
            circle(original, center, 1, cv::Scalar(0, 100, 100), 5, cv::LINE_AA);
            // circle outline
            int radius = c[2];
            circle(original, center, radius, cv::Scalar(255, 0, 255), 5, cv::LINE_AA);
        }

        cv::KeyPoint kp= cv::KeyPoint();

        // show dell'immagine
        cv::imshow(win_plot, original);
        cv::waitKey(10);
    }
}