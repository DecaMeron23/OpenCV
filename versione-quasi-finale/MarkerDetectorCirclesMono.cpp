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
#include <stresa/runtime/VProperty.hpp>

#include "interfaces/perception/MarkerDetector.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using namespace stresa;
using namespace cv;
using namespace std;
#define DEBUG
namespace aurora {
namespace functionality {
namespace perception {

struct InfoPoint {
    int id;
    cv::Point2f pos;
    float dim;
    float dimPix;
    cv::Point2f posPix;
};

float GRANDE = 0.10;
float PICCOLO = 0.04;
float maxDist = 0.30;

class MarkerDetectorCirclesMono : public MarkerDetector {
   public:
    MarkerDetectorCirclesMono() {
        registerProperty(marker_size, "MarkerSize");
        registerProperty(show_marker, "ShowMarker");
        registerProperty(camera_frame_id, "CameraFrameId");
        registerProperty(robot_frame_id, "RobotFrameId");
    }
    void setup() {
        float Df = 68.5;                    // Fov Diagonale [°]
        int Ha = 16, Va = 9, width = 1280;  // aspect ratio orizzontale e verticale
        focal_length = lunghezza_focale(width, Df, Ha, Va);
    }

    void detectMarkers(cv::Mat& frame, sensor_msgs::msg::dds_::Image_ image_msg, aurora_msgs::msg::dds_::Object3DArray_& markers_set_msg) {
        // get the robot to camera transformation
        SceneTransform robot2camera;
        SceneTransform tilt2camera;
        SceneTransform pan2tilt;
        timespec time_stamp;
        time_stamp = now();
        double roll_tilt, pitch_tilt, yaw_tilt;
        double roll_pan, pitch_pan, yaw_pan;
        double roll, pitch, yaw;
        /*
                        try {
                                getSceneTransform("base_link", "camera_rgb_link", robot2camera, time_stamp);
                                base2camera.getRPY(roll, pitch, yaw);

                                std::cout << "Base2Camera T("<<base2camera.tra[0] <<", "<< base2camera.tra[1] << ", " << base2camera.tra[2] <<
                                        ")  R(" << roll << ", " << pitch << ", " << yaw << ")" << std::endl;

                        } catch (const std::invalid_argument& e) {
                                std::cout << "\n*** ERROR *** [MarkerDetectorCirclesMono]::detectMarkers()" << std::endl;
                                std::cout << e.what() << std::endl;
                                return;
                        }
        */

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

        // create the marker message with the list of detected markers
        double qx, qy, qz, qw;

        aurora_msgs::msg::dds_::Object3DArray_ markers_msg;

        aurora_msgs::msg::dds_::Object3D_ marker_msg;
        marker_msg.header().stamp().sec(image_msg.header().stamp().sec());
        marker_msg.header().stamp().nanosec(image_msg.header().stamp().nanosec());
        marker_msg.header().frame_id(image_msg.header().frame_id());

        marker_msg.type("red_circle");

        marker_msg.size().x(marker_size);  // width
        marker_msg.size().y(marker_size);  // length
        marker_msg.size().z(0.0);          // height

        // #############################################################################################

        // createTrackBar();

        // imagePos immagine in gray scale
        // imageNeg negato di imagePos

        cv::Mat hsv;  // Image in grayscale (required from detectMarkers(...) function)

        // Aumentiam la luminosità
        // frame += cv::Scalar(75,75,75);

        cv::Mat immagine = frame.clone();
        // cv::Mat calibra;

        // calibrazione(immagine, immagine);

        cv::cvtColor(immagine, hsv, cv::COLOR_BGR2HSV);

        cv::Mat img_final_red = redFilter(hsv);
        cv::Mat img_final_green = greenFilter(hsv);

        cv::Mat img_final_red_blob = 255 - redFilter(hsv);
        cv::Mat img_final_green_blob = 255 - greenFilter(hsv);

        std::vector<cv::KeyPoint> keypointsPos_blobRed = blobFinder(img_final_red_blob);
        std::vector<cv::KeyPoint> keypointsPos_blobGreen = blobFinder(img_final_green_blob);

        std::vector<cv::KeyPoint> keypointsPos_circleRed = circleFinder(img_final_red);
        std::vector<cv::KeyPoint> keypointsPos_circleGreen = circleFinder(img_final_green);

        cv::Scalar coloreCerchi(255, 0, 0);
        cv::Scalar coloreBlob(0, 255, 255);

        //
#ifdef DEBUG /*                                                             \
                                                                          \ \
         disegnaCerchi(immagine, keypointsPos_blobRed, coloreBlob);         \
         disegnaCerchi(immagine, keypointsPos_blobGreen, coloreBlob);       \
                                                                          \ \
         disegnaCerchi(immagine, keypointsPos_circleRed, coloreCerchi);     \
         disegnaCerchi(immagine, keypointsPos_circleGreen, coloreCerchi);   \
         */
#endif

        // #########################################################################################################################

        // Calcolo posizione dei cerchi rispetto al robot sfruttando la telecamera.

        // Per il momento utilizziamo come keypoints i blob rossi.

        // Sempre per il momento utilizziamo dati fissi per provare a vedere il funzionamento.
        float pan = -CV_PI / 2 - yaw;

        // Il roll è il nostro pitch(?)
        float tilt = roll + CV_PI / 2;

        float h = robot2camera.tra[2];  // Stimata usando metro, unità di misura in metri.

        char testo[100];

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

#ifdef DEBUG  // contenere codice che visualizza
              /*punto = cv::Point2f(cerchi_blob[i].pt.x, cerchi_blob[i].pt.y);
              punto2 = cv::Point2f(cerchi_blob[i].pt.x + 20, cerchi_blob[i].pt.y + 10);
              cv::Point2f punto3(cerchi_blob[i].pt.x + 40, cerchi_blob[i].pt.y + 20);
              // sprintf(testo, "id: %d , x0: %.2f , y0: %.2f    x1: %.2f , y1: %.2f    x2: %.2f , y2: %.2f", i, posizione.tra[0], posizione.tra[1], posizione2.tra[0], posizione2.tra[1], posizione3.tra[0], posizione3.tra[1]);
              sprintf(testo, "id: %d ", i);
              scrivi_testo(immagine, punto, testo);*/
#endif
        }

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

// cv::Mat img2, calibra2;
// cv::resize(immagine, img2, cv::Size(immagine.size().width / 2, immagine.size().height / 2));
// cv::resize(calibra, calibra2, cv::Size(immagine.size().width / 2, immagine.size().height / 2));
//  Display result
#ifdef DEBUG
        cv::namedWindow("Marker Detection", cv::WINDOW_AUTOSIZE);
        cv::imshow("Marker Detection", immagine);
        // cv::imshow("Immagine calibrata", calibra2);
        cv::waitKey(1);
#endif
    }

   protected:
    VProperty<double> marker_size;  // [m]
    VProperty<bool> show_marker;
    VProperty<std::string> camera_frame_id;
    VProperty<std::string> robot_frame_id;

   private:
    // Lunghezza focale in pixel
    float focal_length;
    // Altezza della camera con angolo tilt = 0°
    float altezza_camera_piana;
    // Lunghezza del braccio della camera per tilt, con tilt = 0°
    float braccio_tilt;

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

    void disegnaCerchi(cv::Mat& frame, std::vector<cv::KeyPoint> punti, cv::Scalar colore) {
        for (size_t i = 0; i < punti.size(); ++i) {
            auto blob = punti[i];
            cv::circle(frame, blob.pt, blob.size, colore, 5, cv::LINE_AA);
            cv::circle(frame, blob.pt, 1, cv::Scalar(0, 0, 0), 5, cv::LINE_AA);
        }
    }

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

    cv::Mat scalaImmagine(cv::Mat img, double fattore) {
        cv::Mat dst;
        cv::resize(img, dst, cv::Size(0, 0), fattore, fattore);
        return dst;
    }

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
     * Calcola la posizione (relativa alla base della telecamera) di un punto su una immagine.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param lunghezza_focale      lunghezza focale della telecamera in pixel.
     * @param dimensione_immagine   dimensioni dell'immagine in pixel (altezza e larghezza).
     * @param punto_immagine        posizione del punto (in pixel) nell'immagine da calcolare la distanza.
     * @param tilt                  angolo di beccheggio della telecamera in gradi.
     * @param pan                   angolo di imbardata della telecamera in gradi.
     * @param altezza               altezza della telcamera in metri.
     *
     * @return cv::Point2f posizione x e y del punto.
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
     * Calcolo della rototraslazione di un punto, di un'immagine, rispetto alla base del robot.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param dimensione_immagine   dimensioni dell'immagine in pixel (altezza e larghezza).
     * @param punto_immagine        posizione del punto (in pixel) nell'immagine da calcolare la rototraslazione.
     * @param base2camera			rototraslazione dalla base del robot al centro della camera
     * @return SceneTrasform dalla base del robot al punto_immagine.
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
     * Stima della lunghezza focale in pixel.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param larghezza_immagine larghezza dell'immagine in pixel
     * @param Df fov diagonale in gradi.
     * @param Ha aspect ratio orizzontale (es: il 16 dei 16:9).
     * @param Va aspect ratio verticale (es: il 9 dei 16:9).
     *
     * @return lunghezza focale in pixel
     *
     * @note
     * - Converting Diagonal Field of View (FOV) to Horizontal FOV: https://www.litchiutilities.com/docs/fov.php
     *
     */
    float lunghezza_focale(int larghezza_immagine, float Df, float Ha, float Va) {
        // Calcolo del fov orizzontale e conversione in radianti
        float Hf_rad = deg2rad(fov_orizzontale(Df, Ha, Va));
        return (larghezza_immagine / 2.0) / tan(Hf_rad / 2.0);
    }

    /**
     * Funzione che scrive del testo sull'immagine in un punto specifico
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     *
     * @param immagine immagine su cui scrivere.
     * @param punto posizione dove scrivere.
     * @param stringa testo da scrivere.
     *
     */
    void scrivi_testo(cv::Mat& immagine, cv::Point2f& punto, char stringa[]) {
        cv::Scalar colore_centro(0, 255, 0);
        cv::Point2f tralsazione_testo(0, -25);
        cv::circle(immagine, punto, 2, colore_centro, 10, cv::LINE_AA);
        cv::putText(immagine, stringa, punto + tralsazione_testo, cv::FONT_HERSHEY_SIMPLEX, 0.4, colore_centro, 1, cv::LINE_AA);
    }

    /**
     * Calcolo del FOV orizzontale a partire dal FOV diagonale e l'aspect ratio
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param Df fov diagonale in gradi.
     * @param Ha aspect ratio orizzontale (es: il 16 dei 16:9).
     * @param Va aspect ratio verticale (es: il 9 dei 16:9).
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
    float fov_orizzontale(float Df, float Ha, float Va) {
        float Df_rad = deg2rad(Df);

        float Da = sqrt(Ha * Ha + Va * Va);
        float Hf_rad = 2 * atan(tan(Df_rad / 2) * (Ha / Da));
        return rad2deg(Hf_rad);
    }

    /**
     * conversione da radianti a gradi.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     * @param rad valore in radianti
     *
     * @return angolo in gradi
     *
     * @note
     *  deg = rad · 180° / 𝛑
     */
    float rad2deg(float rad) {
        return rad * 180.0 / CV_PI;
    }
    /**
     * conversione da gradi a radianti.
     *
     * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
     *
     *
     * @param deg valore in gradi
     *
     * @return angolo in radianti
     *
     * @note
     *  rad = deg · 𝛑 / 180°
     *
     */
    float deg2rad(float deg) {
        return deg * CV_PI / 180.0;
    }

    // secondo metodo matematico per stimare il punto
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
    // unione di intersezionePiano e calcola_distanza->la prima stima bene la y, mentre la seconda la x
    SceneTransform stimaTotale(cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, SceneTransform base2camera) {
        SceneTransform intPiano = intersezionePiano(dimensione_immagine, punto_immagine, base2camera);
        SceneTransform distGeom = calcola_distanza_geometrica(dimensione_immagine, punto_immagine, base2camera);
        SceneTransform totale;
        totale.tra[0] = distGeom.tra[0];  // prendo la x della prima
        totale.tra[1] = intPiano.tra[1];  // prendo la y della seconda
        totale.tra[2] = 0;

        return totale;
    }

    void calibrazione(cv::Mat& img, cv::Mat& img_dst) {
        // Matrice camera
        cv::Mat mtx = (cv::Mat_<double>(3, 3) << 859.5064370989816, 0, 464.2947880328538,
                       0, 859.2516111814972, 248.0367363905911,
                       0, 0, 1);
        // std::cout << "Stampa punto mtx." << std::endl;
        //  Coefficienti di distorsione
        cv::Mat dist = (cv::Mat_<double>(1, 5) << 0.1680120526597095, -0.8452358916431213, -0.001724305891844227, 0.0002730512314013186, 0.592314193167666);
        // std::cout << "Stampa punto dist." << std::endl;
        //  Leggere l'immagine

        int w = img.cols;
        int h = img.rows;

        cv::Mat newcameramtx;
        cv::Rect roi;
        newcameramtx = cv::getOptimalNewCameraMatrix(mtx, dist, cv::Size(w, h), 1, cv::Size(w, h), &roi);
        // std::cout << "Stampa punto newcameramtx." << std::endl;
        //  Undistorsion
        cv::Mat dst;
        cv::undistort(img, dst, mtx, dist, newcameramtx);
        // std::cout << "Stampa punto undistort." << std::endl;
        //  ROI
        //  img_dst = dst(roi);
        // std::cout << "Stampa punto dst." << std::endl;
    }

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
