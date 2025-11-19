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
void scrivi_testo(Mat &frame, Point2i &punto, String &testo);

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

int altezza_int = 90;      //[cm]
int angolo_deg_int = 6000; //[cdeg]

int main()
{
    VideoCapture video_capture(0);
    Mat frame;
    namedWindow(nome_window_immagine);
    create_trackbar();
    char testo[100];

    while (true)
    {
        video_capture >> frame;
        Point2i center_image = get_center_image(frame);
        Point2f distanza = calcola_distanza_centro(((float)altezza_int) / 100, ((float)angolo_deg_int) / 100);
        sprintf(testo, "distanza: %.2f m", distanza.x);
        scrivi_testo(frame, center_image, testo);
        imshow(nome_window_immagine, frame);
        waitKey(10);
    }

    video_capture.release();
}

cv::Point2f calcola_distanza_centro(float altezza, float angolo_deg)
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

/**
 * Calcola la posizione (relativa alla base della telecamera) di un punto su una immagine.
 *
 * @param[in] lunghezza_focale      lunghezza focale della telecamera in pixel.
 * @param[in] dimensione_immagine   dimensioni dell'immagine in pixel (altezza e larghezza).
 * @param[in] punto_immagine        posizione del punto (in pixel) nell'immagine da calcolare la distanza.
 * @param[in] angolo                angolo di beccheggio della telecamera in gradi.
 * @param[in] altezza               altezza della telcamera in metri.
 *
 * @return cv::Point2f posizione x e y del punto.
 */
cv::Point2f calcola_distanza(float lunghezza_focale, cv::Size dimensione_immagine, cv::Point2i punto_immagine, float angolo, float altezza)
{
    // Conversione delle cordinate immagine in coordinate camera
    double pos_orizzontale = punto_immagine.x - dimensione_immagine.width / 2.0;
    double pos_verticale = punto_immagine.y - dimensione_immagine.height / 2.0;

    // calcolo degli angoli nel sistema di coordinate della camera
    double theta_orizzontale = atan2(pos_orizzontale, lunghezza_focale); // Angolo orizzontale [radianti]
    double theta_verticale = atan2(pos_verticale, lunghezza_focale);     // Angolo verticale [radianti]

    // conversione dell'inclinazione della camera in radianti
    double angolo_rad = deg2rad(angolo);

    // Angolo totale dal piano orizzontale
    double angolo_totale_verticale = angolo_rad + theta_verticale;

    // Stima della distanza
    double world_x = altezza * tan(angolo_totale_verticale);
    double world_y = world_x * tan(theta_orizzontale);

    return Point2f(world_x, world_y);
}
/**
 * Stima della lunghezza focale in pixel.
 *
 * @param[in] larghezza_immagine larghezza dell'immagine in pixel
 * @param[in] fov_orizzontale fov orizzontale in gradi
 *
 * @return lunghezza focale in pixel
 *
 * @note
 * - Converting Diagonal Field of View (FOV) to Horizontal FOV: https://www.litchiutilities.com/docs/fov.php
 *
 */
float lunghezza_focale(int larghezza_immagine, float fov_orizzontale)
{
    // Conversione in radianti
    float fov_orizzontale_rad = deg2rad(fov_orizzontale);
    return (larghezza_immagine / 2.0) / tan(fov_orizzontale_rad / 2.0);
}

/**
 * Funzione che scrive del testo sull'immagine in un punto specifico
 *
 * @param[in] immagine immagine su cui scrivere.
 * @param[in] punto posizione dove scrivere.
 * @param[in] stringa testo da scrivere.
 *
 */
void scrivi_testo(Mat &immagine, Point2i &punto, char stringa[])
{
    cv::Scalar colore_centro(0, 255, 0);
    cv::Point2i tralsazione_testo(0, -25);
    cv::circle(immagine, punto, 2, colore_centro, 10, cv::LINE_AA);
    cv::putText(immagine, stringa, punto + tralsazione_testo, cv::FONT_HERSHEY_SIMPLEX, 0.6, colore_centro, 1, cv::LINE_AA);
}

/**
 * Calcolo del FOV orizzontale a partire dal FOV diagonale e l'aspect ratio
 *
 * @param[in] Df fov diagonale in gradi.
 * @param[in] Ha aspect ratio orizzontale (es: il 16 dei 16:9).
 * @param[in] Va aspect ratio verticale (es: il 9 dei 16:9).
 *
 * @return fov orizzontale in gradi
 *
 * @note
 * Calcolo del Fov Orizzontale. Sia l'aspect ratio diagonale, Da:
 *
 *      Da = √(Ha^2 + Va^2)
 *
 * Da cui si ricava il fov Orizzontale, Hf:
 *
 *      Hf = 2·atan( tan( Df / 2 ) · (Ha / Da))
 *
 *
 * - Converting Diagonal Field of View (FOV) to Horizontal FOV: https://www.litchiutilities.com/docs/fov.php
 *
 * - Converting diagonal field of view and aspect ratio to horizontal and vertical field of view: https://medium.com/insights-on-virtual-reality/converting-diagonal-field-of-view-and-aspect-ratio-to-horizontal-and-vertical-field-of-view-13bcc1d8600c
 *
 */
float fov_orizzontale(float Df, float Ha, float Va)
{
    float Df_rad = deg2rad(Df);

    float Da = sqrt(Ha * Ha + Va * Va);
    float Hf_rad = 2 * atan(tan(Df_rad / 2) * (Ha / Da));
    return rad2deg(Hf_rad);
}
/**
 * conversione da radianti a gradi.
 *
 * @param[in] rad valore in radianti
 *
 * @return angolo in gradi
 *
 * @note
 *  deg = rad · 180° / 𝛑
 */
float rad2deg(float rad)
{
    return rad * 180.0 / CV_PI;
}
/**
 * conversione da gradi a radianti.
 *deg2rad(Df)
 * @param[in] deg valore in gradi
 *
 * @return angolo in radianti
 *
 * @note
 *  rad = deg · 𝛑 / 180°
 *
 */
float deg2rad(float deg)
{
    return deg * CV_PI / 180.0;
}
