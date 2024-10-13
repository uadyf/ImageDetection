#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main(){
    cv::Mat image = cv::imread("20 by 20 orthogonal maze.", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Error: Unable to open the image!" << std::endl;
        return -1;
    }

    // Convert the grayscale image to a binary image using a threshold
    cv::Mat binaryImage;
    cv::threshold(image, binaryImage, 128, 255, cv::THRESH_BINARY);

    // Print the binary values (0 or 255) of the binary image
    for (int i = 0; i < binaryImage.rows; ++i) {
        for (int j = 0; j < binaryImage.cols; ++j) {
            std::cout << (binaryImage.at<uchar>(i, j) == 255 ? 1 : 0) << " ";
        }
        std::cout << std::endl;
    }
    
    return 0;
}