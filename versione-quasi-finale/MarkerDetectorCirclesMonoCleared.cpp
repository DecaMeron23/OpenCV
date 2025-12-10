/********************************************************************************
 *
 * MarkerDetectionCirclesMono
 *
 * Copyright (c) 2025
 * All rights reserved.
 *
 * Davide Brugali, Università degli Studi di Bergamo
 *
 * -------------------------------------------------------------------------------
 *
 * File: MarkerDetectionCirclesMono.cpp
 * Created: February 14, 2025
 *
 * Supervised by: <A HREF="mailto:brugali@unibg.it">Davide Brugali</A>
 *
 * -------------------------------------------------------------------------------
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * -------------------------------------------------------------------------------
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the University of Bergamo nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 *******************************************************************************/
#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <stresa/runtime/VProperty.hpp>
#include "interfaces/perception/MarkerDetector.hpp"

using namespace stresa;
using namespace cv;
using namespace std;

#define DEBUG

namespace aurora {
namespace functionality {
namespace perception {

// ======================================================================
// Struct e costanti globali
// ======================================================================

struct InfoPoint {
    int id;             // Identificare del cerchio
    cv::Point2f pos;    // Posizione del cerchio in m
    float dim;          // Dimensione del cerchio
    float dimPix;       // Dimensione del cerchio in pixel
    cv::Point2f posPix; // Posizione del cerchio in pixel
};

float GRANDE   = 0.10f; // Definisce il diametro di un cerchio classificato GRANDE.
float PICCOLO  = 0.04f; // Definisce il diametro di un cerchio classificato PICCOLO.
float maxDist  = 0.30f; // Definisce la massima distanza all'interno della quale cercare un cerchio adiacente.

class MarkerDetectorCirclesMono : public MarkerDetector {
public:
    MarkerDetectorCirclesMono() {
        registerProperty(marker_size,     "MarkerSize");
        registerProperty(show_marker,     "ShowMarker");
        registerProperty(camera_frame_id, "CameraFrameId");
        registerProperty(robot_frame_id,  "RobotFrameId");
    }

    /**
     * Funzione di setup del detector.
     * Calcola la lunghezza focale della camera a partire dai parametri noti del sensore:
     * - FOV diagonale (Df);
     * - aspect ratio della camera espresso come Ha : Va;
     * - larghezza dell'immagine in pixel.
     *
     * La lunghezza focale ottenuta viene memorizzata e successivamente utilizzata
     * per tutte le stime geometriche dei marker nell’immagine.
     */
    void setup() {
        float Df = 68.5f;             // Fov Diagonale [°]
        int Ha = 16, Va = 9;          // aspect ratio orizzontale e verticale
        int width = 1280;            
        focal_length = lunghezza_focale(width, Df, Ha, Va);
    }

    void detectMarkers(cv::Mat& frame,
                       sensor_msgs::msg::dds_::Image_ image_msg,
                       aurora_msgs::msg::dds_::Object3DArray_& markers_set_msg) {
        // ----------------------------------------------------------------
        // Recupero delle rototraslazioni tra i vari frame del robot.
        // Queste trasformazioni permettono di conoscere:
        //  - posizione della camera rispetto alla base del robot;
        //  - posizione della camera rispetto al tilt;
        //  - posizione della camera rispetto al pan.
        // Da queste è possibile ricavare roll, pitch, yaw.
        // ----------------------------------------------------------------
        SceneTransform robot2camera;
        SceneTransform tilt2camera;
        SceneTransform pan2tilt;

        timespec time_stamp;
        time_stamp = now();

        double roll_tilt, pitch_tilt, yaw_tilt;
        double roll_pan,  pitch_pan,  yaw_pan;
        double roll, pitch, yaw;

        try {
            getSceneTransform("rgb_tilt_link", "rgb_camera_link", tilt2camera, time_stamp);
            getSceneTransform("rgb_pan_link", "rgb_tilt_link", pan2tilt, time_stamp);
            getSceneTransform(robot_frame_id, camera_frame_id, robot2camera, time_stamp);
            tilt2camera.getRPY(roll_tilt, pitch_tilt, yaw_tilt);
            pan2tilt.getRPY(roll_pan, pitch_pan, yaw_pan);
            robot2camera.getRPY(roll, pitch, yaw);

            std::cout << "Robot2Camera T(" << robot2camera.tra[0] << ", " << robot2camera.tra[1] << ", " << robot2camera.tra[2] << ")  R(" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
        } catch (const std::invalid_argument& e) {
            std::cout << "\n*** ERROR *** [MarkerDetectorCirclesMono]::detectMarkers()" << std::endl;
            std::cout << e.what() << std::endl;
            return;
        }

        // ----------------------------------------------------------------
        // Preparazione messaggio marker
        // ----------------------------------------------------------------
        double qx, qy, qz, qw;  // (non utilizzati che facciamo?)

        aurora_msgs::msg::dds_::Object3DArray_ markers_msg;

        aurora_msgs::msg::dds_::Object3D_ marker_msg;
        marker_msg.header().stamp().sec(image_msg.header().stamp().sec());
        marker_msg.header().stamp().nanosec(image_msg.header().stamp().nanosec());
        marker_msg.header().frame_id(image_msg.header().frame_id());

        marker_msg.type("red_circle");

        marker_msg.size().x(marker_size);  // width
        marker_msg.size().y(marker_size);  // length
        marker_msg.size().z(0.0);          // height

        // ----------------------------------------------------------------
        // Segmentazione in HSV per estrarre separatamente componenti rosse e verdi.
        // Vengono create:
        //  - due immagini ottimizzate per il detector di cerchi;
        //  - due versioni invertite per il detector di blob.
        // Queste quattro immagini permettono di analizzare sia forma che colore.
        // ----------------------------------------------------------------
        // createTrackBar();
        cv::Mat hsv;
        cv::Mat immagine = frame.clone();

        cv::cvtColor(immagine, hsv, cv::COLOR_BGR2HSV);

        cv::Mat img_final_red = redFilter(hsv);
        cv::Mat img_final_green = greenFilter(hsv);

        cv::Mat img_final_red_blob = 255 - redFilter(hsv);
        cv::Mat img_final_green_blob = 255 - greenFilter(hsv);

        // ----------------------------------------------------------------
        // Applicazione dei due estrattori:
        //  - blobFinder(): individua regioni compatte indipendentemente dalla forma;
        //  - circleFinder(): individua regioni con elevata circularity.
        // Il risultato sono quattro insiemi di keypoint (rossi/verdi, blob/cerchi).
        // ----------------------------------------------------------------
        std::vector<cv::KeyPoint> keypointsPos_blobRed = blobFinder(img_final_red_blob);
        std::vector<cv::KeyPoint> keypointsPos_blobGreen = blobFinder(img_final_green_blob);

        std::vector<cv::KeyPoint> keypointsPos_circleRed = circleFinder(img_final_red);
        std::vector<cv::KeyPoint> keypointsPos_circleGreen = circleFinder(img_final_green);

        cv::Scalar coloreCerchi(255, 0, 0);
        cv::Scalar coloreBlob(0, 255, 255);

#ifdef DEBUG
        /*
        disegnaCerchi(immagine, keypointsPos_blobRed,   coloreBlob);
        disegnaCerchi(immagine, keypointsPos_blobGreen, coloreBlob);

        disegnaCerchi(immagine, keypointsPos_circleRed,   coloreCerchi);
        disegnaCerchi(immagine, keypointsPos_circleGreen, coloreCerchi);
        */
#endif

        // ----------------------------------------------------------------
        // Calcolo dell'orientamento effettivo della camera tramite RPY e
        // determinazione di pan e tilt in radianti.
        // Si calcola inoltre il punto centrale dell'immagine proiettato
        // sul pavimento, utile per debugging e analisi della geometria.
        // ----------------------------------------------------------------
        float pan = -CV_PI / 2 - yaw; // Stima del pan usando lo yaw ricavato dalla rototraslazione.
        float tilt = roll + CV_PI / 2; // Stima del tilt usando il roll ricavato dalla rototraslazione.
        float h = robot2camera.tra[2];  // Altezza stimata della camera ricavata dalla traslazione di robot2camera.

        char testo[100]; // Utilizzato per scrivere del testo sull'immagine.

        // Unione di tutti i cerchi e blob, rossi e verdi
        std::vector<cv::KeyPoint> cerchi_blob(keypointsPos_circleRed);
        cerchi_blob.insert(cerchi_blob.end(), keypointsPos_blobRed.begin(), keypointsPos_blobRed.end());
        cerchi_blob.insert(cerchi_blob.end(), keypointsPos_blobGreen.begin(), keypointsPos_blobGreen.end());
        cerchi_blob.insert(cerchi_blob.end(), keypointsPos_circleGreen.begin(), keypointsPos_circleGreen.end());

        cv::KeyPoint punto_immagine;

        SceneTransform base2punto = calcola_distanza_geometrica_centro_immagine(immagine.size(), robot2camera);

        sprintf(testo, "tilt: %.2f , pan: %.2f, h: %.2fm , x: %.2f", tilt, pan, h, base2punto.tra[0]);

        cv::Point2f punto(50.0, 50.0);
        cv::Point2f punto2(immagine.size().width / 2, immagine.size().height / 2);
        cv::circle(immagine, punto2, 5, cv::Scalar(255, 0, 0), -1);

        cv::putText(immagine, testo, punto, cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        // ----------------------------------------------------------------
        // Per ogni keypoint rilevato si calcola la sua posizione 3D stimata
        // rispetto alla base del robot. La funzione stimaTotale combina
        // due metodi geometrici per ottenere una stima più stabile.
        // I punti vengono salvati in un vettore InfoPoint per analisi successive
        // e per la classificazione GRANDE/PICCOLO.
        // ----------------------------------------------------------------
        std::vector<InfoPoint> cerchi;

        for (int i = 0; i < cerchi_blob.size(); i++) {
            // marker_msg.id(1);

            SceneTransform posizione3 = stimaTotale(immagine.size(), cerchi_blob[i], robot2camera);
            InfoPoint p{i, cv::Point2f(posizione3.tra[0], posizione3.tra[1]), 0, cerchi_blob.at(i).size, cerchi_blob.at(i).pt};

            cerchi.push_back(p);
            std::cout << "Pos x: " << p.pos.x << std::endl;
            marker_msg.pose().pose().position().x(posizione3.tra[0]);
            marker_msg.pose().pose().position().y(posizione3.tra[1]);
            marker_msg.pose().pose().position().z(0.0);  // Hp: marker sul pavimento quindi traslazione rispetto a z nulla.

            // marker_msg.pose().pose().orientation().setRPY(0,0,0);

            markers_msg.objects().push_back(marker_msg);

            std::cout << "[MarkerDetection] Marker id:" << i << " Pose(" << posizione3.tra[0] << ", " << posizione3.tra[1] << ", " << 0 << ")" << std::endl;

#ifdef DEBUG
            /*
            punto  = cv::Point2f(cerchi_blob[i].pt.x,     cerchi_blob[i].pt.y);
            punto2 = cv::Point2f(cerchi_blob[i].pt.x + 20, cerchi_blob[i].pt.y + 10);
            cv::Point2f punto3(cerchi_blob[i].pt.x + 40,  cerchi_blob[i].pt.y + 20);
            std::sprintf(testo, "id: %d ", i);
            scrivi_testo(immagine, punto, testo);
            */
#endif
        }

        // ----------------------------------------------------------------
        // Una volta trovati tutti i marker, si classificano in GRANDE o PICCOLO
        // in base alla distanza tra essi e alle loro dimensioni reali nell'immagine.
        // La classificazione è utile per sistemi di navigazione basati su marker.
        // Successivamente si disegnano i cerchi colorati sull'immagine:
        //  - ROSSO  = GRANDE
        //  - VERDE  = PICCOLO
        //  - BLU    = non classificato
        // ----------------------------------------------------------------
        determina_parametri(cerchi);
        std::cout << "Abbiamo determinato i parametri: qui quo qua quaquaraqua" << std::endl;
        for (int i = 0; i < cerchi.size(); i++) {
            if (cerchi.at(i).dim == GRANDE) {
                cv::circle(immagine, cerchi.at(i).posPix, cerchi.at(i).dimPix, cv::Scalar(0, 0, 255), 0);
            } else if (cerchi.at(i).dim == PICCOLO) {
                cv::circle(immagine, cerchi.at(i).posPix, cerchi.at(i).dimPix, cv::Scalar(0, 255, 0), 0);
            } else {
                cv::circle(immagine, cerchi.at(i).posPix, cerchi.at(i).dimPix, cv::Scalar(255, 0, 0), 0);
            }
        }

#ifdef DEBUG
        // ----------------------------------------------------------------
        // Se DEBUG è attivo, viene mostrata l’immagine con:
        // - cerchi rossi = marker grandi
        // - cerchi verdi = marker piccoli
        // - cerchi blu   = marker non classificati
        // La finestra si aggiorna in tempo reale.
        // ----------------------------------------------------------------
        cv::namedWindow("Marker Detection", cv::WINDOW_AUTOSIZE);
        cv::imshow("Marker Detection", immagine);
        cv::waitKey(1);
#endif
    }

   protected:
    VProperty<double> marker_size;  // [m]
    VProperty<bool> show_marker;
    VProperty<std::string> camera_frame_id;
    VProperty<std::string> robot_frame_id;

   private:
   // SONO DA TENERE? UNICO USATO FOCAL_LENGTH MA NON SO SE E' QUESTO CHE VIENE USATO?
    // Lunghezza focale in pixel
    float focal_length;
    // Altezza della camera con angolo tilt = 0°
    float altezza_camera_piana;
    // Lunghezza del braccio della camera per tilt, con tilt = 0°
    float braccio_tilt;

    // Parametri blob / filtri. Da tenere perché presi per riferimento dalla TrackBar? Resto usa SimpleBlobDetector params.
    int thrStep = 5;
    int minThreshold = 0;
    int maxThreshold = 255;
    int minArea = 10;
    int maxArea = 100000;
    int minCircularity = 75;
    int minConvexity = 0;
    int minInertiaRatio = 0;
    int kernel_size = 3;
    float fattore = 0.5;

    /**
     * Utility grafica che disegna dei cerchi intorno ai blob.
     * Se il blob è riconosciuto come cerchio, di solito viene usato un cerchio blu;
     * se è riconosciuto come semplice blob, di solito viene usato un cerchio giallo.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param frame   Immagine su cui vengono disegnati i cerchi.
     * @param punti   Vettore dei punti riconosciuti come blob.
     * @param colore  Colore da assegnare ai cerchi.
     */

    void disegnaCerchi(cv::Mat& frame, std::vector<cv::KeyPoint> punti, cv::Scalar colore) {
        for (size_t i = 0; i < punti.size(); ++i) {
            auto blob = punti[i];
            cv::circle(frame, blob.pt, blob.size, colore, 5, cv::LINE_AA);
            cv::circle(frame, blob.pt, 1, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
        }
    }

    /**
     * Utility grafica che permette di scrivere del testo in un determinato punto dell'immagine.
     * In particolare è usata per scrivere del testo sopra ai punti riconosciuti come blob o cerchi.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param immagine Immagine su cui viene scritto il testo.
     * @param punto    Punto dell'immagine in cui centrare il testo.
     * @param stringa  Testo da scrivere.
     */

    void scrivi_testo(cv::Mat& immagine, cv::Point2f& punto, char stringa[]) {
        cv::Scalar colore_centro(0, 255, 0);
        cv::Point2f tralsazione_testo(0, -25);
        cv::circle(immagine, punto, 2, colore_centro, 10, cv::LINE_AA);
        cv::putText(immagine, stringa, punto + tralsazione_testo, cv::FONT_HERSHEY_SIMPLEX, 0.4, colore_centro, 1, cv::LINE_AA);
    }

    /**
     * Utility grafica che crea una TrackBar per aggiustare i parametri di riconoscimento
     * dei blob/cerchi in tempo reale.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     */
    void createTrackBar() {
        std::string nameWin = "TrackBar";
        cv::namedWindow(nameWin);
        /*cv::createTrackbar("Thr step" , nameWin , &thrStep , 100);
        cv::createTrackbar("thr min" , nameWin , &minThreshold , 255);
        cv::createTrackbar("Thr max" , nameWin , &maxThreshold , 255);
        */
        cv::createTrackbar("area min", nameWin, &minArea, 5000);
        cv::createTrackbar("area max", nameWin, &maxArea, 100000);
        cv::createTrackbar("circularity min", nameWin, &minCircularity, 100);
        // cv::createTrackbar("minConvexity" , nameWin , &minConvexity , 100);
        // cv::createTrackbar("filterByInertia" , nameWin , &minInertiaRatio , 100);
        // cv::createTrackbar("blobColor" , nameWin , &blobColor , 255);
        cv::createTrackbar("Kernel", nameWin, &kernel_size, 255);
        // cv::createTrackbar("Contrasto" , nameWin , &fattore , 10);
    }
    
    /**
     * Funzione che permette di ricavare i blob, che rispettano determinati parametri,
     * dall'immagine video.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param image  Immagine da cui si ricavano i blob (tipicamente binaria).
     *
     * @return Vettore di cv::KeyPoint che contiene tutti i punti assimilabili a blob.
     */
    std::vector<cv::KeyPoint> blobFinder(cv::Mat& image) {
        // Set up parameters
        cv::SimpleBlobDetector::Params params;
        params.thresholdStep = thrStep;
        params.filterByArea = true;
        params.minArea = 10;
        params.maxArea = maxArea;
        params.filterByCircularity = false;
        params.minCircularity = 0;

        // Create detector and detect blobs
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(image, keypoints);

        return keypoints;
    }

    /**
     * Funzione che permette di ricavare i cerchi, che rispettano determinati parametri,
     * dall'immagine video.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param image  Immagine da cui si ricavano i cerchi (tipicamente binaria).
     *
     * @return Vettore di cv::KeyPoint che contiene tutti i punti assimilabili a cerchi.
     */
    std::vector<cv::KeyPoint> circleFinder(cv::Mat& image) {
        // Set up parameters
        cv::SimpleBlobDetector::Params params;
        params.thresholdStep = thrStep;
        params.filterByArea = true;
        params.minArea = minArea;
        params.maxArea = maxArea;
        params.filterByCircularity = true;
        params.minCircularity = .6;

        // Create detector and detect blobs
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(image, keypoints);

        return keypoints;
    }

    /**
     * Filtro che permette di estrarre gli oggetti verdi da un'immagine in spazio HSV.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param imgHSV  Immagine in spazio HSV da cui si ricavano oggetti di colore verde.
     *
     * @return cv::Mat, maschera binaria che contiene solo le regioni verdi.
     */
    cv::Mat greenFilter(cv::Mat imgHSV) {
        cv::Mat mask;

        cv::Scalar lower_green(35, 50, 50);
        cv::Scalar upper_green(85, 255, 255);

        cv::inRange(imgHSV, lower_green, upper_green, mask);

        cv::medianBlur(mask, mask, kernel_size * 2 + 1);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size * 2 + 1, kernel_size * 2 + 1));

        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);

        return mask;
    }

    /**
     * Filtro che permette di estrarre gli oggetti rossi da un'immagine in spazio HSV.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param imgHSV  Immagine in spazio HSV che viene filtrata per evidenziare le regioni rosse.
     *
     * @return cv::Mat, maschera binaria (in HSV modificata nel corso dell'elaborazione)
     *         che evidenzia le regioni rosse.
     */
    cv::Mat redFilter(cv::Mat imgHSV) {
        cv::Mat mask1, mask2, mask3, res;

        cv::Scalar lower_red1(0, 100, 100);
        cv::Scalar upper_red1(10, 255, 255);
        cv::Scalar lower_red2(160, 100, 100);
        cv::Scalar upper_red2(179, 255, 255);

        cv::inRange(imgHSV, lower_red1, upper_red1, mask1);
        cv::inRange(imgHSV, lower_red2, upper_red2, mask2);
        cv::bitwise_or(mask1, mask2, mask3);

        cv::bitwise_or(mask1, mask2, imgHSV, mask3);

        cv::medianBlur(imgHSV, imgHSV, kernel_size * 2 + 1);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size * 2 + 1, kernel_size * 2 + 1));

        cv::morphologyEx(imgHSV, imgHSV, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
        cv::morphologyEx(imgHSV, imgHSV, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 1);

        return imgHSV;
    }

    /**
     * Calcola la posizione (relativa alla base del robot) del punto centrale dell'immagine
     * utilizzando le geometrie dei triangoli rettangoli.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param dimensione_immagine   Dimensioni dell'immagine in pixel (altezza e larghezza).
     * @param base2camera           Rototraslazione dalla base del robot alla camera.
     *
     * @return SceneTransform, rototraslazione dalla base del robot al punto del piano
     *         individuato dal centro dell'immagine.
     */
    SceneTransform calcola_distanza_geometrica_centro_immagine(cv::Size dimensione_immagine, SceneTransform base2camera) {
        double roll, pitch, yaw;
        base2camera.getRPY(roll, pitch, yaw);

        // conversione dell'inclinazione della camera in radianti
        double tilt_rad = -roll;
        double pan_rad = -CV_PI / 2 - yaw;

        double h = base2camera.tra[2];

        double distanza_z = cv::abs(h / cos(tilt_rad));

        SceneTransform base2punto;
        // SceneTransform camera2base = base2camera.inverse();
        SceneTransform camera2punto = SceneTransform();
        camera2punto.setTranslation(0, 0, distanza_z);

        base2punto = base2camera * camera2punto;

        return base2punto;
    }

    /**
     * Calcola la rototraslazione di un punto dell'immagine rispetto alla base del robot
     * utilizzando le geometrie dei triangoli.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param dimensione_immagine   Dimensioni dell'immagine in pixel (altezza e larghezza).
     * @param punto_immagine        Posizione nell'immagine del punto (in pixel) sul quale
     *                              calcolare la rototraslazione.
     * @param base2camera           Rototraslazione dalla base del robot alla camera.
     *
     * @return SceneTransform, rototraslazione dalla base del robot al punto sul piano.
     */
    SceneTransform calcola_distanza_geometrica(cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, SceneTransform base2camera) {
        double roll, pitch, yaw;
        base2camera.getRPY(roll, pitch, yaw);

        double alphax, betax, gammax;
        double Ax, Bx, Cx;
        double alphay, betay, gammay;
        double Ay, By, Cy;

        double tilt = roll + CV_PI / 2;  // 0° gradi quando punta dritto e negativo verso il basso
        double pan = -CV_PI / 2 - yaw;
        // Conversione delle cordinate immagine in coordinate camera
        double pos_orizzontale = punto_immagine.pt.x - dimensione_immagine.width / 2.0;
        double pos_verticale = punto_immagine.pt.y - dimensione_immagine.height / 2.0;

        double theta_orizzontale = atan2(pos_orizzontale, focal_length);  // Angolo orizzontale [radianti]
        double theta_verticale = atan2(pos_verticale, focal_length);      // Angolo verticale [radianti]

        SceneTransform base2punto, camera2centro;

        // calcolo degli angoli nel sistema di coordinate della camera lungo x
        alphax = CV_PI / 2 + tilt;
        betax = CV_PI / 2 - alphax;
        gammax = CV_PI - theta_verticale - betax;

        Cx = base2camera.tra[2] / cos(alphax);
        Ax = Cx * sin(theta_verticale) / sin(gammax);

        // calcolo degli angoli nel sistema di coordinate della camera lungo y
        alphay = theta_orizzontale;
        betay = CV_PI / 2;
        gammay = CV_PI - alphay - betay;

        Cy = Cx;
        Ay = Cy * sin(alphay) / sin(gammay);

        camera2centro.setTranslation(Ay, 0, Cx);
        base2punto = base2camera * camera2centro;
        base2punto.setTranslation(base2punto.tra[0] - Ax, base2punto.tra[1], base2punto.tra[2]);

        return base2punto;
    }

    /**
     * Calcola la lunghezza focale della camera in pixel, a partire dal FOV e
     * dall'aspect ratio.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param larghezza_immagine   Larghezza dell'immagine in pixel.
     * @param Df                   FOV diagonale da specifiche della camera (in gradi).
     * @param Ha                   Componente relativa all'altezza dell'aspect ratio della camera (es. 16:9).
     * @param Va                   Componente relativa alla larghezza dell'aspect ratio della camera (es. 16:9).
     *
     * @return float, lunghezza focale della camera espressa in pixel.
     */
    float lunghezza_focale(int larghezza_immagine, float Df, float Ha, float Va) {
        // Calcolo del fov orizzontale e conversione in radianti
        float Hf_rad = deg2rad(fov_orizzontale(Df, Ha, Va));
        return (larghezza_immagine / 2.0) / tan(Hf_rad / 2.0);
    }

    /**
     * Calcola il FOV orizzontale della camera a partire dal FOV diagonale.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param Df    FOV diagonale da specifiche della camera (in gradi).
     * @param Ha    Componente relativa all'altezza dell'aspect ratio della camera (es. 16:9).
     * @param Va    Componente relativa alla larghezza dell'aspect ratio della camera (es. 16:9).
     *
     * @return float, FOV orizzontale della camera espresso in gradi.
     */
    float fov_orizzontale(float Df, float Ha, float Va) {
        float Df_rad = deg2rad(Df);

        float Da = sqrt(Ha * Ha + Va * Va);
        float Hf_rad = 2 * atan(tan(Df_rad / 2) * (Ha / Da));
        return rad2deg(Hf_rad);
    }

    /**
     * Utility algebrica che converte i radianti in gradi.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param rad    Angolo in radianti da convertire.
     *
     * @return float, angolo convertito in gradi.
     */
    float rad2deg(float rad) {
        return rad * 180.0 / CV_PI;
    }

    /**
     * Utility algebrica che converte i gradi in radianti.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param deg    Angolo in gradi da convertire.
     *
     * @return float, angolo convertito in radianti.
     */
    float deg2rad(float deg) {
        return deg * CV_PI / 180.0;
    }

    /**
     * Calcola la rototraslazione di un punto dell'immagine rispetto alla base del robot
     * utilizzando l'intersezione retta-piano (piano z = 0).
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param dimensione_immagine   Dimensioni dell'immagine in pixel (altezza e larghezza).
     * @param punto_immagine        Posizione nell'immagine del punto (in pixel) sul quale
     *                              calcolare la rototraslazione.
     * @param base2camera           Rototraslazione dalla base del robot alla camera.
     *
     * @return SceneTransform, rototraslazione dalla base del robot al punto sul piano z = 0.
     */
    SceneTransform intersezionePiano(cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, SceneTransform base2camera) {
        double roll, pitch, yaw;
        base2camera.getRPY(roll, pitch, yaw);

        double tilt = roll + CV_PI / 2;  // 0° gradi quando punta dritto e negativo verso il basso
        double pan = -CV_PI / 2 - yaw;
        // Conversione delle cordinate immagine in coordinate camera
        double pos_orizzontale = punto_immagine.pt.x - dimensione_immagine.width / 2.0;
        double pos_verticale = punto_immagine.pt.y - dimensione_immagine.height / 2.0;

        double theta_orizzontale = atan2(pos_orizzontale, focal_length);  // Angolo orizzontale [radianti]
        double theta_verticale = atan2(pos_verticale, focal_length);      // Angolo verticale [radianti]

        tilt = tilt - theta_verticale;
        pan = pan + theta_orizzontale;

        // direzione raggio
        double dx = std::cos(tilt) * std::cos(pan);
        double dy = std::cos(tilt) * std::sin(pan);
        double dz = std::sin(tilt);

        double t = -base2camera.tra[2] / dz;
        SceneTransform base2punto;
        base2punto.tra[0] = base2camera.tra[0] + t * dx;
        base2punto.tra[1] = base2camera.tra[1] + t * dy;
        base2punto.tra[2] = 0;

        std::cout << "tilt: " << tilt << std::endl;
        std::cout << "pitch: " << pitch << std::endl;
        std::cout << "yaw: " << yaw << std::endl;

        std::cout << "roll: " << roll << std::endl;
        std::cout << "theta_verticale: " << theta_verticale << std::endl;
        std::cout << "theta_orizzontale: " << theta_orizzontale << std::endl;

        return base2punto;
    }

    /**
     * Combina i due metodi di calcolo utilizzati per stimare la rototraslazione di un punto
     * dell'immagine rispetto alla base del robot.
     * In particolare:
     * - sfrutta calcola_distanza_geometrica che dà una migliore stima sull'asse x;
     * - sfrutta intersezionePiano che dà una migliore stima sull'asse y.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param dimensione_immagine   Dimensioni dell'immagine in pixel (altezza e larghezza).
     * @param punto_immagine        Posizione nell'immagine del punto (in pixel) sul quale
     *                              calcolare la rototraslazione.
     * @param base2camera           Rototraslazione dalla base del robot alla camera.
     *
     * @return SceneTransform, rototraslazione stimata dalla base del robot al punto sul piano.
     */
    SceneTransform stimaTotale(cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, SceneTransform base2camera) {
        SceneTransform intPiano = intersezionePiano(dimensione_immagine, punto_immagine, base2camera);
        SceneTransform distGeom = calcola_distanza_geometrica(dimensione_immagine, punto_immagine, base2camera);
        SceneTransform totale;
        totale.tra[0] = distGeom.tra[0];  // prendo la x della prima
        totale.tra[1] = intPiano.tra[1];  // prendo la y della seconda
        totale.tra[2] = 0;

        return totale;
    }

    /**
     * Calcola i parametri di ogni blob o cerchio riconosciuto nell'immagine per permettere poi di
     * classificarlo come GRANDE o PICCOLO.
     * Questa funzione è stata realizzata con l'intento futuro di utilizzare i blob/cerchi PICCOLO
     * posizionati vicino ai blob/cerchi GRANDE per dare direzionalità al robot.
     * 
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param cerchi  Vettore che contiene, per ogni blob/cerchio, le informazioni associate.
     */
    void determina_parametri(std::vector<InfoPoint>& cerchi) {
        for (int i = 0; i < cerchi.size(); i++) {
            std::cout << "FOR 1: " << i << std::endl;
            std::cout << "Cerchi size: " << cerchi.size() << std::endl;
            if (cerchi.at(i).dim == 0) {
                float minDist;
                int minId;
                int j;
                std::cout << "Prima di FOR 2: " << std::endl;
                for (j = 0; j < cerchi.size(); j++) {
                    std::cout << "FOR 2: " << j << std::endl;
                    float distTraBlob = sqrt(pow(cerchi.at(i).pos.x - cerchi.at(j).pos.x, 2) + pow(cerchi.at(i).pos.y - cerchi.at(j).pos.y, 2));

                    if ((minDist == 0 || minDist > distTraBlob) && i != j) {
                        minId = j;
                        minDist = distTraBlob;
                    }
                }
                if (j == cerchi.size()) {
                    j = cerchi.size() - 1;
                }
                std::cout << "Dopo FOR 2: " << std::endl;
                if (minDist > maxDist) {
                    cerchi.at(i).dim = GRANDE;
                } else if (cerchi.at(i).dimPix > cerchi.at(j).dimPix) {
                    cerchi.at(i).dim = GRANDE;
                    cerchi.at(j).dim = PICCOLO;
                } else {
                    cerchi.at(j).dim = GRANDE;
                    cerchi.at(i).dim = PICCOLO;
                }
            }
            std::cout << "FUORI FOR 2: " << std::endl;
        }
        std::cout << "FUORI FOR 1: " << std::endl;
    }
};

extern "C" BOOST_SYMBOL_EXPORT MarkerDetectorCirclesMono marker_detector_circles_mono;
MarkerDetectorCirclesMono marker_detector_circles_mono;

}  // namespace perception
}  // namespace functionality
}  // namespace aurora

