#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <filesystem>

using namespace cv;
using namespace std;

string im = "D:/ImageDetection/maze/";

void get_pixel_info() {
    // Load the image in grayscale mode
    Mat image = imread(im, IMREAD_GRAYSCALE);

    // Convert the grayscale image to a binary image using a threshold
    Mat binaryImage;
    threshold(image, binaryImage, 128, 255, THRESH_BINARY);

    // Print the binary values (0 or 255) of the binary image
    for (int i = 0; i < binaryImage.rows; ++i) {
        for (int j = 0; j < binaryImage.cols; ++j) {
            cout << (binaryImage.at<uchar>(i, j) == 255 ? 0 : 1);
        }
        cout << endl;
    }
    imshow("Image", image);
    waitKey(0);
}

int main() {
    vector<string> all_files;
    for (const auto& files : filesystem::directory_iterator(im)) {
        all_files.push_back(files.path().filename().string());
    }
    int inp;
    while (1) {
        system("cls");
        for (int i = 0; i < (int)all_files.size(); i++) {
            cout << i << ' ' << all_files[i] << endl;
        }
        cin >> inp;
        if (inp != -1) {
            im += all_files[inp];
            get_pixel_info();
            system("pause");
            for (int i = 0; i < (int)all_files[inp].size(); i++) {
                im.pop_back();
            }
        }
        else {
            break;
        }
    }
    return 0;
}