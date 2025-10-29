#include <opencv2/opencv.hpp>
#include <istream>

void getContours(cv::Mat imgDil , cv::Mat imgOut){
    
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(imgDil , contours , hierarchy , cv::RETR_EXTERNAL , cv::CHAIN_APPROX_SIMPLE);

    cv::drawContours(imgOut , contours , -1 , cv::Scalar(255,0,255) , 2);

}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Errore: impossibile aprire la webcam." << std::endl;
        return -1;
    }

    cv::Mat img, imgGray, imgBlur, imgCanny, imgDil;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    while (true) {
        cap.read(img);
        if (img.empty()) break;

        // Preprocessing
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(imgGray, imgBlur, cv::Size(3, 3), 3, 0);
        cv::Canny(imgBlur, imgCanny, 25, 75);
        cv::dilate(imgCanny, imgDil, kernel);

        getContours(imgDil, img);

        cv::imshow("Immagine con contorni", img);

        // Esce se premi 'q'
        if (cv::waitKey(1) == 'q') break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}