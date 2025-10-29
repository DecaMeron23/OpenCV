#include <opencv2/opencv.hpp>
#include <istream>

int main()
{
    std::string path = "../immagini/image.png";

    cv::Mat img = cv::imread(path);

    float w = 500, h = 250;
    cv::Point2f src[4] = {{502, 318}, {1351, 723}, {181, 745}, {1053, 1179}};
    cv::Point2f dst[4] = {{0.0f, 0.0f}, {w, 0.0f}, {0.0f, h}, {w, h}};

    cv::Mat matrix, imgWarp;
    matrix = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(img, imgWarp, matrix, cv::Point(w, h));

    cv::imshow("Immagine", img);
    cv::imshow("Immagine Warp", imgWarp);
    cv::waitKey(0);

    return 0;
}