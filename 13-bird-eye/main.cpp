#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;

// Struttura per i parametri della trasformazione
struct BirdEyeParams {
    Point2f src[4];    // Punti sorgente nell'immagine originale
    Point2f dst[4];    // Punti destinazione nella vista bird's eye
    Size outputSize;   // Dimensione dell'output
};

// Variabili globali per le trackbar
int tiltAngle = 40;    // Angolo in gradi (0-80)
int heightCm = 40;     // Altezza in cm (10-200)
bool paramsChanged = false;

class BirdEyeTransformer {
private:
    Mat homographyMatrix;
    BirdEyeParams params;

public:
    // Costruttore con parametri
    BirdEyeTransformer(const BirdEyeParams& parameters) : params(parameters) {
        calculateHomography();
    }

    // Calcola la matrice di omografia
    void calculateHomography() {
        homographyMatrix = getPerspectiveTransform(params.src, params.dst);
    }

    // Applica la trasformazione bird's eye
    Mat transform(const Mat& inputImage) {
        Mat outputImage;
        warpPerspective(inputImage, outputImage, homographyMatrix, params.outputSize);
        return outputImage;
    }

    // Disegna i punti di controllo sull'immagine
    Mat drawControlPoints(const Mat& image) {
        Mat result = image.clone();
        
        // Disegna i punti sorgente
        for(int i = 0; i < 4; i++) {
            circle(result, params.src[i], 5, Scalar(0, 0, 255), -1); // Rosso
            putText(result, to_string(i), params.src[i] + Point2f(10, -10), 
                   FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
        }
        
        // Disegna il poligono che definisce l'area
        vector<Point> srcPoints;
        for(int i = 0; i < 4; i++) {
            srcPoints.push_back(params.src[i]);
        }
        polylines(result, srcPoints, true, Scalar(0, 255, 0), 2); // Verde
        
        // Aggiungi informazioni sui parametri
        putText(result, "Angolo: " + to_string(tiltAngle) + "deg", Point(10, 30), 
               FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        putText(result, "Altezza: " + to_string(heightCm) + "cm", Point(10, 60), 
               FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
        
        return result;
    }

    // Aggiorna i parametri
    void updateParams(const BirdEyeParams& newParams) {
        params = newParams;
        calculateHomography();
    }

    // Getter per la matrice di omografia
    Mat getHomographyMatrix() const {
        return homographyMatrix.clone();
    }
};

// Funzioni callback per le trackbar
void onTiltAngleChange(int value, void* userdata) {
    paramsChanged = true;
}

void onHeightChange(int value, void* userdata) {
    paramsChanged = true;
}

// Calcola parametri basati su inclinazione e altezza
BirdEyeParams calculateParamsFromCameraGeometry(int imageWidth, int imageHeight, 
                                               float tiltDegrees, 
                                               float heightCm) {
    BirdEyeParams params;
    
    // Converti angolo in radianti
    float tiltRad = tiltDegrees * CV_PI / 180.0f;
    
    // Calcola fattori basati su angolo e altezza
    float angleFactor = tiltDegrees / 45.0f;  // Normalizza su 45°
    float heightFactor = heightCm / 100.0f;   // Normalizza su 100cm
    
    int centerX = imageWidth / 2;
    int bottomY = imageHeight - 1;
    
    // Larghezza alla base (più larga con angoli più accentuati)
    int baseWidth = imageWidth * (0.6f + 0.2f * angleFactor);
    // Larghezza in alto (più stretta con angoli più accentuati)
    int topWidth = imageWidth * (0.2f + 0.2f * angleFactor);
    
    // Altezza dell'area di interesse (dipende dall'altezza e dall'angolo)
    int roiHeight = imageHeight * (0.4f + 0.3f * heightFactor / angleFactor);
    roiHeight = min(roiHeight, imageHeight - 50); // Limita massimo
    int topY = imageHeight - roiHeight;
    
    // Punti sorgente (area trapezoidale)
    params.src[0] = Point2f(centerX - topWidth/2, topY);     // Alto-sinistra
    params.src[1] = Point2f(centerX + topWidth/2, topY);     // Alto-destra
    params.src[2] = Point2f(centerX + baseWidth/2, bottomY); // Basso-destra
    params.src[3] = Point2f(centerX - baseWidth/2, bottomY); // Basso-sinistra
    
    // Punti destinazione (vista bird's eye - rettangolare)
    // Usiamo una dimensione leggermente ridotta per l'output
    int outputWidth = imageWidth;
    int outputHeight = imageHeight;
    
    params.dst[0] = Point2f(0, 0);                           // Alto-sinistra
    params.dst[1] = Point2f(outputWidth, 0);                 // Alto-destra
    params.dst[2] = Point2f(outputWidth, outputHeight);      // Basso-destra
    params.dst[3] = Point2f(0, outputHeight);                // Basso-sinistra
    
    params.outputSize = Size(outputWidth, outputHeight);
    
    return params;
}

int main() {
    // Apri la camera (0 = camera predefinita)
    VideoCapture cap(0);
    if(!cap.isOpened()) {
        cout << "Errore: Impossibile aprire la camera!" << endl;
        return -1;
    }
    
    // Imposta risoluzione camera
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    
    // Leggi una frame per ottenere le dimensioni
    Mat frame;
    cap >> frame;
    if(frame.empty()) {
        cout << "Errore: Impossibile leggere dalla camera!" << endl;
        return -1;
    }
    
    int imageWidth = frame.cols;
    int imageHeight = frame.rows;
    
    cout << "Risoluzione camera: " << imageWidth << " x " << imageHeight << endl;
    
    // Crea finestre e trackbar
    namedWindow("Camera Originale", WINDOW_AUTOSIZE);
    namedWindow("Bird Eye View", WINDOW_AUTOSIZE);
    namedWindow("Controlli", WINDOW_AUTOSIZE);
    
    // Crea finestra per i controlli
    Mat controlWindow = Mat::zeros(200, 400, CV_8UC3);
    imshow("Controlli", controlWindow);
    
    // Crea trackbar
    createTrackbar("Angolo (deg)", "Controlli", &tiltAngle, 80, onTiltAngleChange);
    createTrackbar("Altezza (cm)", "Controlli", &heightCm, 200, onHeightChange);
    
    // Calcola parametri iniziali
    BirdEyeParams params = calculateParamsFromCameraGeometry(imageWidth, imageHeight, 
                                                           tiltAngle, heightCm);
    
    // Crea il trasformatore
    BirdEyeTransformer transformer(params);
    
    cout << "Premi:" << endl;
    cout << "  'q' per uscire" << endl;
    cout << "  's' per salvare un frame" << endl;
    cout << "  'r' per resettare parametri" << endl;
    cout << "  Usa le trackbar per modificare angolo e altezza" << endl;
    
    bool pause = false;
    int frameCount = 0;
    
    while(true) {
        if(!pause) {
            cap >> frame;
            if(frame.empty()) {
                cout << "Frame vuoto!" << endl;
                break;
            }
        }
        
        // Controlla se i parametri sono cambiati
        if(paramsChanged) {
            params = calculateParamsFromCameraGeometry(imageWidth, imageHeight, 
                                                     tiltAngle, heightCm);
            transformer.updateParams(params);
            paramsChanged = false;
            
            cout << "Parametri aggiornati - Angolo: " << tiltAngle 
                 << "deg, Altezza: " << heightCm << "cm" << endl;
        }
        
        // Disegna i punti di controllo
        Mat frameWithPoints = transformer.drawControlPoints(frame);
        
        // Applica la trasformazione bird's eye
        Mat birdEyeView = transformer.transform(frame);
        
        // Mostra i risultati
        imshow("Camera Originale", frameWithPoints);
        imshow("Bird Eye View", birdEyeView);
        
        // Aggiorna finestra controlli con informazioni
        controlWindow.setTo(Scalar(50, 50, 50));
        putText(controlWindow, "CONTROLLI BIRD EYE VIEW", Point(20, 30), 
               FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
        putText(controlWindow, "Angolo: " + to_string(tiltAngle) + " gradi", Point(20, 70), 
               FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
        putText(controlWindow, "Altezza: " + to_string(heightCm) + " cm", Point(20, 100), 
               FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
        putText(controlWindow, "Istruzioni:", Point(20, 140), 
               FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
        putText(controlWindow, "q=Esci, s=Salva, r=Reset", Point(20, 160), 
               FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 0), 1);
        imshow("Controlli", controlWindow);
        
        // Gestione tasti
        char key = waitKey(1);
        if(key == 'q' || key == 27) { // 'q' o ESC
            break;
        }
        else if(key == 's') { // Salva frame
            string origName = "frame_original_" + to_string(frameCount) + ".jpg";
            string birdEyeName = "frame_bird_eye_" + to_string(frameCount) + ".jpg";
            imwrite(origName, frameWithPoints);
            imwrite(birdEyeName, birdEyeView);
            cout << "Frame salvati: " << origName << " e " << birdEyeName << endl;
            frameCount++;
        }
        else if(key == 'r') { // Reset parametri
            tiltAngle = 40;
            heightCm = 40;
            paramsChanged = true;
            setTrackbarPos("Angolo (deg)", "Controlli", tiltAngle);
            setTrackbarPos("Altezza (cm)", "Controlli", heightCm);
            cout << "Parametri resettati a valori predefiniti" << endl;
        }
        else if(key == 'p') { // Pausa
            pause = !pause;
            cout << (pause ? "PAUSA" : "RIPRENDI") << endl;
        }
    }
    
    cap.release();
    destroyAllWindows();
    
    return 0;
}