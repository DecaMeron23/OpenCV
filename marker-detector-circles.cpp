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
#include <stresa/runtime/VProperty.hpp>

#include "interfaces/perception/MarkerDetector.hpp"

#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>


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
	}
	void setup() {
		float Df = 68.5; 						// Fov Diagonale [°]
		int Ha = 16 , Va = 9 , width = 1280; 	// aspect ratio orizzontale e verticale
		
		altezza_camera_piana = 0.72;			// Altezza della camera con angolo tilt = 0° 
		braccio_tilt = 0.06;					// Lunghezza del braccio della camera per tilt, con tilt = 0°
		focal_length = lunghezza_focale(width, Df, Ha, Va);
	
	}

	void detectMarkers(cv::Mat &frame, sensor_msgs::msg::dds_::Image_ image_msg, aurora_msgs::msg::dds_::Object3DArray_ &markers_set_msg) {
		// create the marker message with the list of detected markers
		double qx, qy, qz, qw;

		aurora_msgs::msg::dds_::Object3DArray_ markers_msg;

		aurora_msgs::msg::dds_::Object3D_ marker_msg;
		marker_msg.header().stamp().sec(image_msg.header().stamp().sec());
		marker_msg.header().stamp().nanosec(image_msg.header().stamp().nanosec());
		marker_msg.header().frame_id(image_msg.header().frame_id());

		marker_msg.type("red_circle");

		marker_msg.size().x(marker_size); 	// width
		marker_msg.size().y(marker_size); 	// length
		marker_msg.size().z(0.0); 			// height
		
		
		
		// #############################################################################################
		        
        //createTrackBar();
        
        // imagePos immagine in gray scale
        // imageNeg negato di imagePos
        
        cv::Mat hsv; // Image in grayscale (required from detectMarkers(...) function)

		// Aumentiam la luminosità
		//frame += cv::Scalar(75,75,75);

		cv::Mat immagine = frame.clone();
		
		cv::cvtColor(immagine, hsv, cv::COLOR_BGR2HSV);
		
		cv::Mat img_final_red = redFilter(hsv);
		cv::Mat img_final_green = greenFilter(hsv);
		
		cv::Mat img_final_red_blob = 255-redFilter(hsv);
		cv::Mat img_final_green_blob = 255-greenFilter(hsv);
		
		
		
		std::vector<cv::KeyPoint> keypointsPos_blobRed = blobFinder(img_final_red_blob);
        std::vector<cv::KeyPoint> keypointsPos_blobGreen = blobFinder(img_final_green_blob);
        
        std::vector<cv::KeyPoint> keypointsPos_circleRed = circleFinder(img_final_red);
        std::vector<cv::KeyPoint> keypointsPos_circleGreen = circleFinder(img_final_green);
		
		cv::Scalar coloreCerchi(255, 0 ,0 );
		cv::Scalar coloreBlob(0 , 255 , 255);

		//
		
		disegnaCerchi(immagine, keypointsPos_blobRed , coloreBlob);
		disegnaCerchi(immagine, keypointsPos_blobGreen , coloreBlob);
		
		disegnaCerchi(immagine, keypointsPos_circleRed , coloreCerchi);
		disegnaCerchi(immagine, keypointsPos_circleGreen , coloreCerchi);

                

		// #########################################################################################################################
		
		// Calcolo posizione dei cerchi rispetto al robot sfruttando la telecamera.
		
		// Per il momento utilizziamo come keypoints i blob rossi.
		
		// Sempre per il momento utilizziamo dati fissi per provare a vedere il funzionamento. 
		float pan = 0.0;
		float tilt = 45.0; 			// Stiamata usando metro, unità di misura in °.
		float h = altezza_camera_piana - sin(tilt) * braccio_tilt; // Stimata usando metro, unità di misura in metri.

		char testo[100];

		// Unione di tutti i cerchi e blob, rossi e verdi		
		std::vector<cv::KeyPoint> cerchi_blob(keypointsPos_circleRed);
		cerchi_blob.insert( cerchi_blob.end(), keypointsPos_blobRed.begin(), keypointsPos_blobRed.end() );
		cerchi_blob.insert( cerchi_blob.end(), keypointsPos_blobGreen.begin(), keypointsPos_blobGreen.end() );
		cerchi_blob.insert( cerchi_blob.end(), keypointsPos_circleGreen.begin(), keypointsPos_circleGreen.end() );
		
		
		sprintf(testo ,"tilt: %.2f , pan: %.2f , h: %.2fm" , tilt , pan , h);
		
		cv::Point2f punto(50.0,50.0);
		
		cv::putText(immagine, testo , punto , cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0,255,0), 1, cv::LINE_AA);
		
		
		for(int i=0; i < cerchi_blob.size(); i++) {
			
			//marker_msg.id(1);
			
			cv::Point2f coordinate = calcola_distanza(focal_length, frame.size(), cerchi_blob[i] , tilt , pan, h);
			
			//marker_msg.confidence(objects[i].confidence);

			marker_msg.pose().pose().position().x(coordinate.x);
			marker_msg.pose().pose().position().y(coordinate.y);
			marker_msg.pose().pose().position().z(0.0); 			// Hp: marker sul pavimento quindi traslazione rispetto a z nulla.

			//marker_msg.pose().pose().orientation().setRPY(0,0,0);

			markers_msg.objects().push_back(marker_msg);

			std::cout<<"[MarkerDetection] Marker id:" << i << " Pose("<< coordinate.x << ", " << coordinate.y << ", " << 0 << ")"<<std::endl;
		
			punto = cv::Point2f(cerchi_blob[i].pt.x, cerchi_blob[i].pt.y);
			
			sprintf(testo ,"x: %.2f , y:%.2f" , coordinate.x , coordinate.y);
			
			scrivi_testo(immagine, punto , testo);
		}
		
		// Display result
        cv::namedWindow("Marker Detection", cv::WINDOW_AUTOSIZE );
        cv::imshow("Marker Detection", immagine);
        

        
        // cv::imshow("RED", img_final_red);
        // cv::imshow("GREEN", img_final_green);
		cv::waitKey(1);
		
		
		
	}

protected:
	VProperty<double> marker_size;	// [m]
	VProperty<bool> show_marker;

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
	
	void disegnaCerchi(cv::Mat &frame , std::vector<cv::KeyPoint> punti , cv::Scalar colore){
	  for (size_t i =0 ; i < punti.size() ; ++i)
		{
		  auto blob =  punti[i];
		  cv::circle(frame , blob.pt , blob.size  , colore , 5 , cv::LINE_AA);
		  cv::circle(frame , blob.pt , 1  , cv::Scalar(0, 0, 0) , 5 , cv::LINE_AA);
                }
	}
	
	void createTrackBar()
	{
	  std::string nameWin = "TrackBar";
	  cv::namedWindow(nameWin);
	  /*cv::createTrackbar("Thr step" , nameWin , &thrStep , 100);
	  cv::createTrackbar("thr min" , nameWin , &minThreshold , 255);
	  cv::createTrackbar("Thr max" , nameWin , &maxThreshold , 255);
	  */cv::createTrackbar("area min" , nameWin , &minArea , 5000);
	  cv::createTrackbar("area max" , nameWin , &maxArea , 100000);
	  cv::createTrackbar("circularity min" , nameWin , &minCircularity , 100);
	  //cv::createTrackbar("minConvexity" , nameWin , &minConvexity , 100);
	  //cv::createTrackbar("filterByInertia" , nameWin , &minInertiaRatio , 100);
	  //cv::createTrackbar("blobColor" , nameWin , &blobColor , 255);
	  cv::createTrackbar("Kernel" , nameWin , &kernel_size , 255);
	  //cv::createTrackbar("Contrasto" , nameWin , &fattore , 10);
	}
	
	std::vector<cv::KeyPoint> blobFinder(cv::Mat &image) {
	        
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
	
	std::vector<cv::KeyPoint> circleFinder(cv::Mat &image) {
	        
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
	
	
	cv::Mat greenFilter(cv::Mat imgHSV){
	
	  cv::Mat mask;
	
	  cv::Scalar lower_green(35, 50, 50);
      cv::Scalar upper_green(85, 255, 255);
      
      cv::inRange(imgHSV , lower_green , upper_green , mask);
      
      cv::medianBlur(mask,  mask , kernel_size*2+1);
	
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(kernel_size*2+1 , kernel_size*2+1));

      cv::morphologyEx(mask , mask,  cv::MORPH_OPEN  , kernel , cv::Point(-1,-1) , 1);
      cv::morphologyEx(mask , mask,  cv::MORPH_CLOSE  , kernel , cv::Point(-1,-1) , 1);

	  return mask;
	}
	
	cv::Mat scalaImmagine(cv::Mat img, double fattore)
	{
	    cv::Mat dst;
	    cv::resize(img , dst , cv::Size(0 , 0) , fattore , fattore);
	    return dst;
	}
	
	
	cv::Mat redFilter(cv::Mat imgHSV)
	{
          cv::Mat mask1, mask2, mask3 ,  res;
          
          cv::Scalar lower_red1(0, 100, 100);
          cv::Scalar upper_red1(10, 255, 255);
          cv::Scalar lower_red2(160, 100, 100);
          cv::Scalar upper_red2(179, 255, 255);

          cv::inRange(imgHSV, lower_red1, upper_red1, mask1);
          cv::inRange(imgHSV, lower_red2, upper_red2, mask2);
          cv::bitwise_or(mask1, mask2, mask3);

          cv::bitwise_or(mask1, mask2, imgHSV, mask3);
          
          cv::medianBlur(imgHSV,  imgHSV , kernel_size*2+1);
		
          cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE , cv::Size(kernel_size*2+1 , kernel_size*2+1));

          cv::morphologyEx(imgHSV , imgHSV,  cv::MORPH_OPEN  , kernel , cv::Point(-1,-1) , 1);
          cv::morphologyEx(imgHSV , imgHSV,  cv::MORPH_CLOSE  , kernel , cv::Point(-1,-1) , 1);
		
          
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
	cv::Point2f calcola_distanza(float lunghezza_focale, cv::Size dimensione_immagine, cv::KeyPoint punto_immagine, float tilt , float pan , float altezza)
	{
		// Conversione delle cordinate immagine in coordinate camera
		double pos_orizzontale = punto_immagine.pt.x - dimensione_immagine.width / 2.0;
		double pos_verticale = punto_immagine.pt.y - dimensione_immagine.height / 2.0;

		// calcolo degli angoli nel sistema di coordinate della camera
		double theta_orizzontale = atan2(pos_orizzontale, lunghezza_focale); // Angolo orizzontale [radianti]
		double theta_verticale = atan2(pos_verticale, lunghezza_focale);     // Angolo verticale [radianti]

		// conversione dell'inclinazione della camera in radianti
		double tilt_rad = deg2rad(tilt);
		double pan_rad = deg2rad(pan);


		// Angolo totale dal piano orizzontale
		double tilt_totale_verticale = tilt_rad + theta_verticale;
		
		// Angolo totale dal piano verticale
		double pan_totale_verticale = pan_rad + theta_orizzontale;


		// Stima della distanza
		double world_x = altezza / tan(tilt_totale_verticale);
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
	float lunghezza_focale(int larghezza_immagine, float Df, float Ha, float Va)
	{
		// Calcolo del fov orizzontale e conversione in radianti 
		float Hf_rad = deg2rad(fov_orizzontale(Df , Ha , Va));
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
	void scrivi_testo(cv::Mat &immagine, cv::Point2f &punto, char stringa[])
	{
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
	 * @authors Raffaele Giacomo Giovanni Di Maio, Giulia Signori, Emilio Meroni
	 * 
	 * @param rad valore in radianti
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
	float deg2rad(float deg)
	{
		return deg * CV_PI / 180.0;
	}


};

extern "C" BOOST_SYMBOL_EXPORT MarkerDetectorCirclesMono marker_detector_circles_mono;
MarkerDetectorCirclesMono marker_detector_circles_mono;

} // namespace perception
} // namespace functionality
} // namespace aurora