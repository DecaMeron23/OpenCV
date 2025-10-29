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
    Point2d calculateDistance(const Point2f& imagePoint) {
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
        // double distance_x = cameraHeight / tan(total_angle);
        // ! PERCHÉ NON: D = TAN(TOTAL_ANGLE) * H

        // distanza y è la distanza tra la camera e l'oggetto andando "in avanti"
        double distance_y = cameraHeight * tan(total_angle);
        // distanza x è la distanza tra la camera e l'oggetto andando "a destra/sinista"
        double distance_x = distance_y * tan(theta_x);


        return Point2d(distance_y , distance_x);
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