#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void detectAndDisplay(Mat frame);

CascadeClassifier car_cascade;

int main(int argc, const char** argv) {
    printf("\nA\n");
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{car_cascade|./cascade.xml|Path to car cascade.}"
                             "{camera|0|Camera device number.}");
    printf("\nB\n");
    parser.printMessage();
    printf("\nC\n");
    String car_cascade_name = parser.get<String>("car_cascade");
    printf("\nD\n");

    //-- 1. Load the cascades
    if (!car_cascade.load(car_cascade_name)) {
        printf("\nE\n");
        cout << "--(!)Error loading car cascade\n";
        printf("\nF\n");
        return -1;
    };
    printf("\nG\n");
    int camera_device = parser.get<int>("camera");
    printf("\nH\n");
    VideoCapture capture;
    printf("\nI\n");

    //-- 2. Read the video stream
    capture.open(camera_device);
    printf("\nJ\n");
    if (!capture.isOpened()) {
        printf("\nK\n");
        cout << "--(!)Error opening video capture\n";
        printf("\nL\n");
        return -1;
    }
    printf("\nM\n");

    Mat frame;
    printf("\nN\n");

    while (capture.read(frame)) {
        printf("\nO1\n");
        if (frame.empty()) {
            printf("\nO2\n");
            cout << "--(!) No captured frame -- Break!\n";
            printf("\nO3\n");
            break;
        }
        printf("\nO4\n");
        
        //-- 3. Apply the classifier to the frame
        //detectAndDisplay(frame);
        printf("\nO5\n");
        if (waitKey(10) == 27) {
            printf("\nO6\n");
            break; // escape
        }
        printf("\nO7\n");
    }
    printf("\nP\n");

    return 0;
}
