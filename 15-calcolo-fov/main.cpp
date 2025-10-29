#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

class FOVCalibrator {
private:
    Size imageSize;
    double knownDistance;  // Distanza nota in metri
    double knownWidth;     // Larghezza oggetto nota in metri

public:
    FOVCalibrator(const Size& size, double dist, double width) 
        : imageSize(size), knownDistance(dist), knownWidth(width) {}
    
    // Calcola FOV da misurazione pratica
    double calculateHorizontalFOV(double pixelWidth) {
        // Larghezza angolare = 2 * atan(larghezza_oggetto / (2 * distanza))
        double angularWidth = 2.0 * atan(knownWidth / (2.0 * knownDistance));
        
        // FOV = (larghezza_immagine / larghezza_pixel) * larghezza_angolare
        double fovH = (imageSize.width / pixelWidth) * angularWidth * 180.0 / CV_PI;
        
        return fovH;
    }
    
    double calculateVerticalFOV(double pixelHeight) {
        double angularHeight = 2.0 * atan(knownWidth / (2.0 * knownDistance)); // Usa stessa larghezza
        double fovV = (imageSize.height / pixelHeight) * angularHeight * 180.0 / CV_PI;
        return fovV;
    }
};

int main() {
    VideoCapture cap(0);
    if(!cap.isOpened()) {
        cout << "Errore: Impossibile aprire la camera!" << endl;
        return -1;
    }
    
    // Imposta risoluzione
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    
    Mat frame;
    cap >> frame;
    if(frame.empty()) {
        cout << "Errore: Frame vuoto!" << endl;
        return -1;
    }
    
    Size imageSize = frame.size();
    
    // Parametri per calibrazione
    double knownDistance = 1.0;   // 1 metro
    double knownWidth = 0.2;      // 20 cm (larghezza oggetto di riferimento)
    
    FOVCalibrator calibrator(imageSize, knownDistance, knownWidth);
    
    cout << "=== CALIBRAZIONE FOV PRATICA ===" << endl;
    cout << "Istruzioni:" << endl;
    cout << "1. Posiziona un oggetto di larghezza " << (knownWidth * 100) << " cm a " << knownDistance << " m" << endl;
    cout << "2. Misura quanti pixel occupa in larghezza" << endl;
    cout << "3. Inserisci il valore qui sotto" << endl;
    
    double pixelWidth, pixelHeight;
    cout << "Inserisci larghezza in pixel: ";
    cin >> pixelWidth;
    cout << "Inserisci altezza in pixel: ";
    cin >> pixelHeight;
    
    double fovH = calibrator.calculateHorizontalFOV(pixelWidth);
    double fovV = calibrator.calculateVerticalFOV(pixelHeight);
    
    cout << "\n=== RISULTATI CALIBRAZIONE ===" << endl;
    cout << "FOV Orizzontale stimato: " << fovH << " gradi" << endl;
    cout << "FOV Verticale stimato: " << fovV << " gradi" << endl;
    
    cap.release();
    return 0;
}