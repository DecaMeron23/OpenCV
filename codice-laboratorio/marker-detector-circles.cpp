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

namespace aurora {
namespace functionality {
namespace perception {

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

        disegnaCerchi(immagine, keypointsPos_blobRed, coloreBlob);
        disegnaCerchi(immagine, keypointsPos_blobGreen, coloreBlob);

        disegnaCerchi(immagine, keypointsPos_circleRed, coloreCerchi);
        disegnaCerchi(immagine, keypointsPos_circleGreen, coloreCerchi);

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

        SceneTransform base2punto = calcola_distanza(frame.size(), robot2camera);

        sprintf(testo, "tilt: %.2f , pan: %.2f, h: %.2fm , x: %.2f", tilt, pan, h, base2punto.tra[0]);

        cv::Point2f punto(50.0, 50.0);
        cv::Point2f punto2(frame.size().width / 2, frame.size().height / 2);
        cv::circle(immagine, punto2, 5, cv::Scalar(255, 0, 0), -1);

        cv::putText(immagine, testo, punto, cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

        for (int i = 0; i < cerchi_blob.size(); i++) {
            // marker_msg.id(1);

            SceneTransform posizione = calcola_distanza(frame.size(), cerchi_blob[i], robot2camera);

            // marker_msg.confidence(objects[i].confidence);

            marker_msg.pose().pose().position().x(posizione.tra[0]);
            marker_msg.pose().pose().position().y(posizione.tra[1]);
            marker_msg.pose().pose().position().z(0.0);  // Hp: marker sul pavimento quindi traslazione rispetto a z nulla.

            // marker_msg.pose().pose().orientation().setRPY(0,0,0);

            markers_msg.objects().push_back(marker_msg);

            std::cout << "[MarkerDetection] Marker id:" << i << " Pose(" << posizione.tra[0] << ", " << posizione.tra[1] << ", " << 0 << ")" << std::endl;

            punto = cv::Point2f(cerchi_blob[i].pt.x, cerchi_blob[i].pt.y);

            sprintf(testo, "x: %.2f , y:%.2f", posizione.tra[0], posizione.tra[1]);

            scrivi_testo(immagine, punto, testo);
        }

        // Display result
        cv::namedWindow("Marker Detection", cv::WINDOW_AUTOSIZE);
        cv::imshow("Marker Detection", immagine);

        // cv::imshow("RED", img_final_red);
        // cv::imshow("GREEN", img_final_green);
        cv::waitKey(1);
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
    SceneTransform calcola_distanza(cv::Size dimensione_immagine, SceneTransform base2camera) {
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

   SceneTransform calcola_distanza_new(cv::Size dimensione_immagine,
                                const cv::KeyPoint& punto_immagine,
                                const SceneTransform& base2camera)
{
    // 1) Ray in camera coordinates (pinhole)
    double cx = dimensione_immagine.width  / 2.0;
    double cy = dimensione_immagine.height / 2.0;

    double u = punto_immagine.pt.x;
    double v = punto_immagine.pt.y;

    // Direzione del raggio nel frame camera
    cv::Vec3d dir_cam( (u - cx) / focal_length,
                       (v - cy) / focal_length,
                       1.0 );

    // Normalizza (non strettamente necessario ma pulito)
    dir_cam = dir_cam / cv::norm(dir_cam);

    // 2) Porta origine e direzione nel frame base
    // base2camera: T_base_camera
    // R_base_camera = rotazione (3x3)
    // t_base_camera = traslazione (3x1)
    cv::Matx33d R_base_cam;
    {
        double roll, pitch, yaw;
        base2camera.getRPY(roll, pitch, yaw);
        // qui devi costruire la Matx33d da RPY oppure, se SceneTransform
        // ti dà già la matrice di rotazione, usala direttamente
        R_base_cam = base2camera.getRotationMatrix(); // pseudo-codice
    }

    cv::Vec3d t_base_cam(base2camera.tra[0],
                         base2camera.tra[1],
                         base2camera.tra[2]);

    cv::Vec3d dir_base = R_base_cam * dir_cam;   // direzione del raggio in base
    cv::Vec3d orig_base = t_base_cam;            // origine del raggio (posizione della camera in base)

    // 3) Intersezione con il piano z = 0 (pavimento nel frame base)
    //   r(t) = orig_base + t * dir_base
    //   vogliamo: r_z(t) = 0 -> orig_z + t * dir_z = 0
    double dz = dir_base[2];
    double oz = orig_base[2];

    if (std::abs(dz) < 1e-6) {
        // raggio quasi parallelo al pavimento -> niente intersezione utile
        SceneTransform base2punto;
        base2punto.setTranslation(0, 0, 0); // o come preferisci gestire l'errore
        return base2punto;
    }

    double t = -oz / dz;
    cv::Vec3d p_base = orig_base + t * dir_base;

    // 4) Riempie un SceneTransform che rappresenta la posa del punto nel frame base
    SceneTransform base2punto;
    base2punto.setTranslation(p_base[0], p_base[1], p_base[2]);
    // Rotazione identità (stai solo stimando la posizione)
    base2punto.setRPY(0,0,0);

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
    SceneTransform calcola_distanza(cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, SceneTransform base2camera) {
        double roll, pitch, yaw;
        base2camera.getRPY(roll, pitch, yaw);

        double tilt = roll + CV_PI / 2;  // 0° gradi quando punta dritto e negativo verso il basso
        // double pan_rad = roll; //(Da controllare)

        // Conversione delle cordinate immagine in coordinate camera
        double pos_orizzontale = punto_immagine.pt.x - dimensione_immagine.width / 2.0;
        double pos_verticale = punto_immagine.pt.y - dimensione_immagine.height / 2.0;

        // calcolo degli angoli nel sistema di coordinate della camera
        // double theta_orizzontale = atan2(pos_orizzontale, focal_length); 	// Angolo orizzontale [radianti]
        double theta_verticale = atan2(pos_verticale, focal_length);  // Angolo verticale [radianti]

        double alpha, beta, gamma;
        double B, C;

        alpha = CV_PI / 2 + tilt;                // 90 - 33 = 57
        beta = CV_PI / 2 - alpha;                // 90 - 57 = 33
        gamma = CV_PI - theta_verticale - beta;  // 180 - theta_verticale - 33 = 147-theta_verticale

        C = base2camera.tra[2] / cos(alpha);
        B = C * sin(beta) / sin(gamma);

        SceneTransform camera2rotaz, rotaz2punto, base2punto;
        // Passo 1: Rotazione sull'asse Y della camera (verso il basso) (¿ quindi con il meno ?)
        camera2rotaz.setRPY(0, theta_verticale, 0);

        // Passo 2: Traslazione sull'asse Z di B
        rotaz2punto.setTranslation(0, 0, B);

        // Passo 3: Trasformazione da camera a punto
        base2punto = base2camera * camera2rotaz * rotaz2punto;

        std::cout << "theta_verticale: " << theta_verticale << std::endl;

        return base2punto;
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
    cv::Point2f calcola_distanza_old(float lunghezza_focale, cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, SceneTransform base2camera) {
        double roll, pitch, yaw;
        base2camera.getRPY(roll, pitch, yaw);

        // Conversione delle cordinate immagine in coordinate camera
        double pos_orizzontale = punto_immagine.pt.x - dimensione_immagine.width / 2.0;
        double pos_verticale = punto_immagine.pt.y - dimensione_immagine.height / 2.0;

        // calcolo degli angoli nel sistema di coordinate della camera
        double theta_orizzontale = atan2(pos_orizzontale, lunghezza_focale);  // Angolo orizzontale [radianti]
        double theta_verticale = atan2(pos_verticale, lunghezza_focale);      // Angolo verticale [radianti]

        // conversione dell'inclinazione della camera in radianti
        double tilt_rad = CV_PI - roll;
        double pan_rad = yaw - CV_PI / 2;

        // Angolo totale dal piano orizzontale
        double tilt_totale_verticale = tilt_rad + theta_verticale;

        // Angolo totale dal piano verticale
        double pan_totale_verticale = pan_rad + theta_orizzontale;

        // Stima della distanza
        double world_x = base2camera.tra[2] / tan(tilt_totale_verticale);
        double world_y = world_x * tan(pan_totale_verticale);

        return cv::Point2f(world_x, world_y);
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
};

extern "C" BOOST_SYMBOL_EXPORT MarkerDetectorCirclesMono marker_detector_circles_mono;
MarkerDetectorCirclesMono marker_detector_circles_mono;

}  // namespace perception
}  // namespace functionality
}  // namespace aurora
