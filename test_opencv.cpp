#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    try {
        std::cout << "Testing OpenCV GUI..." << std::endl;
        cv::Mat test_image = cv::Mat::zeros(100, 100, CV_8UC3);
        cv::namedWindow("Test", cv::WINDOW_AUTOSIZE);
        cv::imshow("Test", test_image);
        cv::waitKey(1);
        cv::destroyAllWindows();
        std::cout << "OpenCV GUI test successful!" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cout << "OpenCV GUI test failed: " << e.what() << std::endl;
        return 1;
    }
}
