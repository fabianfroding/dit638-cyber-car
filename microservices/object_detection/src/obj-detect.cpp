#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void detectAndDisplay(Mat frame);

CascadeClassifier casc_classer;

int main(int argc, const char** argv) {
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{casc_classer|./cascade.xml|Path to cascade.}"
                             "{camera|0|Camera device number.}");
    parser.printMessage();
    String casc_classer_name = parser.get<String>("casc_classer");
    
    //-- 1. Load the cascades
    if (!casc_classer.load(casc_classer_name)) {
        cout << "--(!)Error loading cascade\n";
        return -1;
    };

    VideoCapture capture("vid.mp4"); // NAME OF VIDEO FILE.
    if (!capture.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }

    Mat frame;
    while (capture.read(frame)) {
        if (frame.empty()) {
            cout << "--(!) No captured frame -- Break!\n";
            break;
        }
        
        //-- 3. Apply the classifier to the frame
        detectAndDisplay(frame);
        if (waitKey(10) == 27) {
            break; // escape
        }
    }
    return 0;
}

void detectAndDisplay(Mat frame) {
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect object
    std::vector<Rect> objects;
    casc_classer.detectMultiScale(frame_gray, objects);
    for (size_t i = 0; i < objects.size(); i++) {
        Point center(objects[i].x + objects[i].width/2, objects[i].y + objects[i].height/2);
        ellipse(frame, center, Size(objects[i].width/2, objects[i].height/2), 0, 0, 360, Scalar(255, 0, 255), 4);
        Mat objectsROI = frame_gray(objects[i]);
    }

    //-- Show what you got
    imshow("Capture - Detection", frame);
}
