#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

// Parametri iniziali della camera
double CAMERA_HEIGHT = 0.90;    // 90cm in metri
double CAMERA_TILT = 30.0;      // 30 gradi verso il basso
double FOV_HORIZONTAL = 60.0;   // Campo visivo orizzontale in gradi
double FOV_VERTICAL = 40.0;     // Campo visivo verticale in gradi

// Variabili globali per le trackbar
int heightTrackbar = 90;        // Altezza in cm (10-200)
int tiltTrackbar = 30;          // Inclinazione in gradi (0-80)
int fovHTrackbar = 60;          // FOV orizzontale (30-120)
int fovVTrackbar = 40;          // FOV verticale (20-80)
bool paramsChanged = false;

class DistanceCalculator {
private:
    double cameraHeight;
    double cameraTilt;
    double fovHorizontal;
    double fovVertical;
    double focalLength;
    Size imageSize;

public:
    DistanceCalculator() {}
    
    // Inizializza con le dimensioni dell'immagine
    void initialize(const Size& size) {
        imageSize = size;
        updateInternalParameters();
        
        cout << "Distance Calculator inizializzato:" << endl;
        cout << "  Risoluzione: " << size.width << " x " << size.height << endl;
        printParameters();
    }
    
    // Aggiorna i parametri interni
    void updateInternalParameters() {
        // Calcola la lunghezza focale approssimativa in pixel
        focalLength = (imageSize.width / 2.0) / tan((fovHorizontal * CV_PI / 180.0) / 2.0);
    }
    
    // Stampa i parametri correnti
    void printParameters() {
        cout << "  Altezza camera: " << cameraHeight << " m" << endl;
        cout << "  Inclinazione: " << cameraTilt << " gradi" << endl;
        cout << "  FOV Orizzontale: " << fovHorizontal << " gradi" << endl;
        cout << "  FOV Verticale: " << fovVertical << " gradi" << endl;
        cout << "  Lunghezza focale: " << focalLength << " pixel" << endl;
    }
    
    // Aggiorna tutti i parametri
    void updateAllParameters(double height, double tilt, double fovH, double fovV) {
        cameraHeight = height;
        cameraTilt = tilt;
        fovHorizontal = fovH;
        fovVertical = fovV;
        updateInternalParameters();
    }
    
    // Calcola la distanza di un punto dall'immagine (in metri)
    double calculateDistance(const Point2f& imagePoint) {
        // Converti le coordinate immagine in coordinate camera
        double x = imagePoint.x - imageSize.width / 2.0;
        double y = imagePoint.y - imageSize.height / 2.0;
        
        // Calcola gli angoli nel sistema di coordinate della camera
        double theta_x = atan2(x, focalLength);  // Angolo orizzontale
        double theta_y = atan2(y, focalLength);  // Angolo verticale
        
        // Converti l'inclinazione della camera in radianti
        double tilt_rad = cameraTilt * CV_PI / 180.0;
        
        // Angolo totale dal piano orizzontale
        double total_angle = tilt_rad + theta_y;
        
        // Calcola la distanza usando trigonometria
        // d = h / tan(angolo)
        // double distance = cameraHeight / tan(total_angle);
        // ! PERCHÉ NON: D = TAN(TOTAL_ANGLE) * h
        double distance = cameraHeight * tan(total_angle);


        // Limita la distanza a valori ragionevoli
        // ! PERCHÉ LIMITARLA?
        distance = max(0.1, min(distance, 50.0));
        
        return distance;
    }
    
    // Calcola le coordinate mondo (x,y) dal punto immagine
    Point3f calculateWorldCoordinates(const Point2f& imagePoint) {
        double distance = calculateDistance(imagePoint);
        
        // Converti le coordinate immagine in coordinate camera
        double x_img = imagePoint.x - imageSize.width / 2.0;
        double y_img = imagePoint.y - imageSize.height / 2.0;
        
        // Calcola gli angoli
        double theta_x = atan2(x_img, focalLength);
        double theta_y = atan2(y_img, focalLength);
        double tilt_rad = cameraTilt * CV_PI / 180.0;
        double total_angle = tilt_rad + theta_y;
        
        // Calcola le coordinate mondo
        double world_x = distance * tan(theta_x);
        double world_y = cameraHeight / tan(total_angle);
        
        return Point3f(world_x, world_y, distance);
    }
    
    // Disegna la griglia delle distanze sull'immagine
    Mat drawDistanceGrid(const Mat& image) {
        Mat result = image.clone();
        
        // Colori per le diverse distanze
        vector<Scalar> colors = {
            Scalar(0, 0, 255),    // Rosso per 1m
            Scalar(0, 165, 255),  // Arancione per 2m
            Scalar(0, 255, 255),  // Giallo per 3m
            Scalar(0, 255, 0),    // Verde per 4m
            Scalar(255, 0, 0),    // Blu per 5m
            Scalar(128, 0, 128)   // Viola per 10m
        };
        
        vector<double> distances = {1.0, 2.0, 3.0, 4.0, 5.0, 10.0};
        
        // Disegna linee orizzontali per distanze fisse
        for(int i = 0; i < distances.size(); i++) {
            double distance = distances[i];
            
            // Trova la coordinata y corrispondente a questa distanza
            double tilt_rad = cameraTilt * CV_PI / 180.0;
            double angle = atan2(cameraHeight, distance) - tilt_rad;
            double y_pixel = tan(angle) * focalLength + imageSize.height / 2.0;
            
            if(y_pixel >= 0 && y_pixel < imageSize.height) {
                line(result, Point(0, y_pixel), Point(imageSize.width, y_pixel), 
                     colors[i], 2);
                putText(result, to_string((int)distance) + "m", Point(10, y_pixel - 10),
                       FONT_HERSHEY_SIMPLEX, 0.6, colors[i], 2);
            }
        }
        
        return result;
    }
};

// Funzioni callback per le trackbar
void onHeightChange(int value, void* userdata) {
    CAMERA_HEIGHT = value / 100.0; // Converti cm in metri
    paramsChanged = true;
}

void onTiltChange(int value, void* userdata) {
    CAMERA_TILT = value;
    paramsChanged = true;
}

void onFovHChange(int value, void* userdata) {
    FOV_HORIZONTAL = value;
    paramsChanged = true;
}

void onFovVChange(int value, void* userdata) {
    FOV_VERTICAL = value;
    paramsChanged = true;
}

// Crea la finestra di controllo con le trackbar
void createControlWindow() {
    namedWindow("Controlli Camera", WINDOW_NORMAL);
    resizeWindow("Controlli Camera", 400, 400);
    
    createTrackbar("Altezza (cm)", "Controlli Camera", &heightTrackbar, 200, onHeightChange);
    createTrackbar("Inclinazione (°)", "Controlli Camera", &tiltTrackbar, 80, onTiltChange);
    createTrackbar("FOV Orizz (°)", "Controlli Camera", &fovHTrackbar, 120, onFovHChange);
    createTrackbar("FOV Vert (°)", "Controlli Camera", &fovVTrackbar, 80, onFovVChange);
    
    // Imposta valori iniziali
    setTrackbarPos("Altezza (cm)", "Controlli Camera", heightTrackbar);
    setTrackbarPos("Inclinazione (°)", "Controlli Camera", tiltTrackbar);
    setTrackbarPos("FOV Orizz (°)", "Controlli Camera", fovHTrackbar);
    setTrackbarPos("FOV Vert (°)", "Controlli Camera", fovVTrackbar);
}

// Disegna la finestra di controllo con informazioni
void updateControlWindow() {
    Mat controlWindow = Mat::zeros(300, 400, CV_8UC3);
    controlWindow.setTo(Scalar(60, 60, 60));
    
    putText(controlWindow, "CONTROLLI CAMERA - STIMA DISTANZA", Point(20, 30), 
           FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
    
    putText(controlWindow, "Altezza: " + to_string(CAMERA_HEIGHT) + " m", Point(20, 70), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    putText(controlWindow, "Inclinazione: " + to_string(CAMERA_TILT) + " gradi", Point(20, 95), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    putText(controlWindow, "FOV Orizz: " + to_string(FOV_HORIZONTAL) + " gradi", Point(20, 120), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    putText(controlWindow, "FOV Vert: " + to_string(FOV_VERTICAL) + " gradi", Point(20, 145), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    
    putText(controlWindow, "ISTRUZIONI:", Point(20, 180), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
    putText(controlWindow, "Clicca sull'immagine per misurare", Point(20, 205), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
    putText(controlWindow, "g: Griglia ON/OFF, c: Cancella punto", Point(20, 230), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
    putText(controlWindow, "r: Reset parametri, q: Esci", Point(20, 255), 
           FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
    
    imshow("Controlli Camera", controlWindow);
}

int main() {
    // Apri la camera
    VideoCapture cap(0);
    if(!cap.isOpened()) {
        cout << "Errore: Impossibile aprire la camera!" << endl;
        return -1;
    }
    
    // Imposta risoluzione
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    
    // Leggi una frame per inizializzare
    Mat frame;
    cap >> frame;
    if(frame.empty()) {
        cout << "Errore: Impossibile leggere dalla camera!" << endl;
        return -1;
    }
    
    // Crea il calcolatore di distanza
    DistanceCalculator distCalc;
    distCalc.initialize(frame.size());
    distCalc.updateAllParameters(CAMERA_HEIGHT, CAMERA_TILT, FOV_HORIZONTAL, FOV_VERTICAL);
    
    // Variabili per il punto selezionato
    Point2f selectedPoint(-1, -1);
    bool pointSelected = false;
    
    // Crea finestre
    namedWindow("Distanza Camera", WINDOW_AUTOSIZE);
    namedWindow("Griglia Distanze", WINDOW_AUTOSIZE);
    createControlWindow();
    
    // Callback per il mouse
    setMouseCallback("Distanza Camera", [](int event, int x, int y, int flags, void* userdata) {
        Point2f* point = static_cast<Point2f*>(userdata);
        if(event == EVENT_LBUTTONDOWN) {
            point->x = x;
            point->y = y;
            *static_cast<bool*>(userdata + sizeof(Point2f)) = true;
            cout << "Punto selezionato: (" << x << ", " << y << ")" << endl;
        }
    }, &selectedPoint);
    
    cout << "=== SISTEMA DI STIMA DISTANZA ===" << endl;
    cout << "Parametri iniziali:" << endl;
    distCalc.printParameters();
    cout << "\nIstruzioni:" << endl;
    cout << "  - Usa le trackbar per configurare i parametri della camera" << endl;
    cout << "  - Clicca su un punto nell'immagine per calcolare la distanza" << endl;
    cout << "  - Premi 'g' per attivare/disattivare la griglia" << endl;
    cout << "  - Premi 'c' per cancellare il punto selezionato" << endl;
    cout << "  - Premi 'r' per resettare i parametri" << endl;
    cout << "  - Premi 'q' per uscire" << endl;
    
    bool showGrid = true;
    
    while(true) {
        cap >> frame;
        if(frame.empty()) break;
        
        // Controlla se i parametri sono cambiati
        if(paramsChanged) {
            distCalc.updateAllParameters(CAMERA_HEIGHT, CAMERA_TILT, FOV_HORIZONTAL, FOV_VERTICAL);
            cout << "Parametri aggiornati:" << endl;
            distCalc.printParameters();
            paramsChanged = false;
        }
        
        Mat displayFrame = frame.clone();
        Mat gridFrame;
        
        // Disegna griglia delle distanze
        if(showGrid) {
            gridFrame = distCalc.drawDistanceGrid(frame);
        } else {
            gridFrame = frame.clone();
        }
        
        // Se è stato selezionato un punto, calcola e mostra la distanza
        if(pointSelected) {
            // Calcola distanza e coordinate mondo
            double distance = distCalc.calculateDistance(selectedPoint);
            Point3f worldCoords = distCalc.calculateWorldCoordinates(selectedPoint);
            
            // Disegna il punto selezionato
            circle(displayFrame, selectedPoint, 8, Scalar(0, 0, 255), -1);
            circle(displayFrame, selectedPoint, 12, Scalar(255, 255, 255), 2);
            circle(gridFrame, selectedPoint, 8, Scalar(0, 0, 255), -1);
            circle(gridFrame, selectedPoint, 12, Scalar(255, 255, 255), 2);
            
            // Mostra informazioni
            string distText = "Distanza: " + to_string(distance).substr(0, 4) + " m";
            string coordText = "Mondo: (" + to_string(worldCoords.x).substr(0, 4) + ", " + 
                              to_string(worldCoords.y).substr(0, 4) + ")";
            
            putText(displayFrame, distText, Point(selectedPoint.x + 15, selectedPoint.y - 20),
                   FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            putText(displayFrame, coordText, Point(selectedPoint.x + 15, selectedPoint.y + 5),
                   FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            
            putText(gridFrame, distText, Point(selectedPoint.x + 15, selectedPoint.y - 20),
                   FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
            putText(gridFrame, coordText, Point(selectedPoint.x + 15, selectedPoint.y + 5),
                   FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        }
        
        // Aggiungi informazioni sui parametri
        putText(displayFrame, "H:" + to_string(CAMERA_HEIGHT).substr(0,4) + "m T:" + to_string(CAMERA_TILT) + "deg", 
               Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(gridFrame, "FOV:" + to_string(FOV_HORIZONTAL) + "deg Grid:" + (showGrid ? "ON" : "OFF"), 
               Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        
        // Mostra le finestre
        imshow("Distanza Camera", displayFrame);
        imshow("Griglia Distanze", gridFrame);
        updateControlWindow();
        
        // Gestione tasti
        char key = waitKey(1);
        if(key == 'q' || key == 27) {
            break;
        }
        else if(key == 'g') {
            showGrid = !showGrid;
            cout << "Griglia " << (showGrid ? "attivata" : "disattivata") << endl;
        }
        else if(key == 'c') {
            pointSelected = false;
            cout << "Punto cancellato" << endl;
        }
        else if(key == 'r') {
            // Reset ai valori iniziali
            heightTrackbar = 90;
            tiltTrackbar = 30;
            fovHTrackbar = 60;
            fovVTrackbar = 40;
            
            setTrackbarPos("Altezza (cm)", "Controlli Camera", heightTrackbar);
            setTrackbarPos("Inclinazione (°)", "Controlli Camera", tiltTrackbar);
            setTrackbarPos("FOV Orizz (°)", "Controlli Camera", fovHTrackbar);
            setTrackbarPos("FOV Vert (°)", "Controlli Camera", fovVTrackbar);
            
            paramsChanged = true;
            cout << "Parametri resettati ai valori predefiniti" << endl;
        }
    }
    
    cap.release();
    destroyAllWindows();
    
    return 0;
}



/*

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
*/
