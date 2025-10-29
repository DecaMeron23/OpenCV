#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

vector<KeyPoint> findBlob(Mat img, SimpleBlobDetector::Params parametri);
Mat filtroColore(Mat img, Scalar lower, Scalar upper);
Mat evidenziaBlob(Mat img, vector<KeyPoint> blobs, Scalar colore);
Mat scalaImmagine(Mat img, double fattore);

int main()
{

    VideoCapture video(0);

    Mat frame;

    while (true)
    {
        video >> frame;
        Mat scalata = scalaImmagine(frame, 2);

        // Filtraggio immagine per colore
        cv::Scalar lower(35, 40, 40);
        cv::Scalar upper(85, 255, 255);

        Mat filtrata = filtroColore(frame, lower, upper);
        Mat filtrata_scalata = filtroColore(scalata, lower, upper);

        // Definizione dei parametri
        SimpleBlobDetector::Params parametri;
        parametri.minThreshold = 0;
        parametri.maxThreshold = 255;

        // Ricerca dei blob
        vector<KeyPoint> blobs_no_scale = findBlob(filtrata, parametri);
        vector<KeyPoint> blobs_scale = findBlob(filtrata_scalata, parametri);

        // Disegno dei cerchi
        scalata = evidenziaBlob(scalata, blobs_scale, Scalar(0, 0, 255));
        frame = evidenziaBlob(frame, blobs_no_scale, Scalar(255, 0, 0));


        imshow("Filtrata - Scalata", filtrata_scalata);
        imshow("Filtrata - Originale", filtrata);

        imshow("Scalata", scalata);
        imshow("Originale", frame);
        waitKey(10);
    }
}

vector<KeyPoint> findBlob(Mat img, SimpleBlobDetector::Params parametri)
{
    // Create detector and detect blobs
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(parametri);
    vector<KeyPoint> keypoints;

    detector->detect(img, keypoints);

    return keypoints;
}

Mat filtroColore(Mat img, Scalar lower, Scalar upper)
{
    Mat imgHSV;

    // prerpocessing: cambio colore e aggiunta blur
    cvtColor(img, imgHSV, COLOR_BGR2HSV);

    inRange(imgHSV, lower, upper, imgHSV);

    return imgHSV;
}

Mat evidenziaBlob(Mat img, vector<KeyPoint> blobs, Scalar colore)
{
    Mat dst;
    drawKeypoints(img, blobs, dst, colore, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    return dst;
}

Mat scalaImmagine(Mat img, double fattore)
{
    Mat dst;
    resize(img , dst , Size(0 , 0) , fattore , fattore);
    return dst;
}
