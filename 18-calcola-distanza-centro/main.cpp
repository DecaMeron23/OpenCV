#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

/**
 * restituisce il punto centrale dell'immagine
 */
Point2i get_center_image(Mat img);

/**
 * disegna un punto sull'immagine, e sopra ci scrive un testo
 */
void disegna_distanza(Mat &frame, Point2i &point, String &testo);

/**
 * restituisce la distanza su x nel punto centrale inquadrato dalla telecamera
 */
Point2f calcola_distanza_centro(float altezza, float angolo_deg);

/**
 * crea una trackbar
 */
void create_trackbar();

Scalar colore_centro(0, 255, 0);
char nome_window_immagine[50] = "Immagine";
char nome_window_trackbar[50] = "Trackbar";

int altezza_int = 90;    //[cm]
int angolo_deg_int = 6000; //[cdeg]

int main()
{
    VideoCapture video_capture(0);
    Mat frame;
    namedWindow(nome_window_immagine);
    create_trackbar();
    char testo_char[100];
    
    while (true)
    {
        video_capture >> frame;
        Point2i center_image = get_center_image(frame);
        Point2f distanza = calcola_distanza_centro(((float)altezza_int) / 100 , ((float)angolo_deg_int) / 100);
        sprintf(testo_char, "distanza: %.2f m", distanza.x);
        String testo(testo_char);
        disegna_distanza(frame, center_image, testo);
        imshow(nome_window_immagine, frame);
        waitKey(10);
    }

    video_capture.release();
}

void disegna_distanza(Mat &frame, Point2i &point, String &testo)
{
    circle(frame, point, 2, colore_centro, 10, LINE_AA);
    putText(frame, testo, point + Point2i(0, -25), FONT_HERSHEY_SIMPLEX, 0.6, colore_centro, 1, LINE_AA);
}

Point2f calcola_distanza_centro(float altezza, float angolo_deg)
{
    float angolo = M_PI * angolo_deg / 180.0;
    float distanza = tan(angolo) * altezza;
    return Point2f(distanza, 0);
}

Point2i get_center_image(Mat img)
{
    Point2i center(img.size().width / 2, img.size().height / 2);
    return center;
}

void create_trackbar()
{
    // namedWindow(nome_window_trackbar, WINDOW_AUTOSIZE);
    createTrackbar("Altezza", nome_window_immagine, &altezza_int, 200);
    createTrackbar("Angolo", nome_window_immagine, &angolo_deg_int, 9000);
}