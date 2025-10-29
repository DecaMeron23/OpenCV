#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


/*  Riconoscimento ci cerchi:
        1. mettere limmagine in gray scale
        2. fare un blur dell'immagine (usiamo median blur)
*/

int parm1 = 100;
int parm2 = 100;
int min_dist = 20;
int raggio_minimo = 200;
int raggio_massimo = 500;

int main(){

    VideoCapture vc = VideoCapture(0);

    if(!vc.isOpened()) return -1;

    string nome_window = "Parametri";

    namedWindow(nome_window);
    createTrackbar("Parm1" , nome_window , &parm1 , 100);
    createTrackbar("Parm2" , nome_window , &parm2 , 100);
    createTrackbar("Distanza Minima" , nome_window , &min_dist , 500);
    createTrackbar("Raggio Minimo" , nome_window , &raggio_minimo , 100);
    createTrackbar("Raggio Massimo" , nome_window , &raggio_massimo , 500);


    string path_image = "../immagini/image.png";

    
    while (true)
    {



        Mat frame , original;
        vc.read(original);

        
        
        // original = imread(path_image);
    
        cvtColor(original , frame , COLOR_BGR2GRAY);
        medianBlur(frame , frame , 5);
        
        // Preprocessing
        vector<Vec3f> circle;

        HoughCircles(frame , circle , HOUGH_GRADIENT , 1 , 50+min_dist , 1+parm1 , 1+parm2 , 1+raggio_minimo , 1+raggio_massimo);

        // Itero per tutti i cerchi
        vector<Vec3f>::const_iterator iter = circle.begin();

        while (iter != circle.end())
        {
            Point center = Point((*iter)[0] , (*iter)[1]);
            float radius = (*iter)[3];
            Scalar colour = Scalar(255 , 0 , 255);

            cv::circle(original , center , radius , colour , 2);
            iter++;
        }

        namedWindow("Cerchi!!" , WINDOW_NORMAL);
        imshow("Cerchi!!" , original);
        waitKey(100);
        
    }
    
    vc.release();
    return 0;

}