#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

void detectAndDisplay(Mat frame);

CascadeClassifier car_cascade;

int main(int argc, const char **argv)
{
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{car_cascade|./face-classifier.xml|Path to car cascade.}"
                             "{camera|0|Camera device number.}");
    parser.printMessage();
    String car_cascade_name = parser.get<String>("car_cascade");

    //-- 1. Load the cascades
    if (!car_cascade.load(car_cascade_name))
    {
        cout << "--(!)Error loading car cascade\n";
        return -1;
    };
    int camera_device = parser.get<int>("camera");
    VideoCapture capture;

    //-- 2. Read the video stream
    capture.open(camera_device);
    if (!capture.isOpened())
    {
        cout << "--(!)Error opening video capture\n";
        return -1;
    }

    Mat frame;
    while (capture.read(frame))
    {
        if (frame.empty())
        {
            cout << "--(!) No captured frame -- Break!\n";
            break;
        }

        //-- 3. Apply the classifier to the frame
        detectAndDisplay(frame);
        if (waitKey(10) == 27)
        {
            break; // escape
        }
    }
    return 0;
}

void detectAndDisplay(Mat frame)
{
    Mat frame_gray;
    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    //-- Detect cars
    std::vector<Rect> cars;
    car_cascade.detectMultiScale(frame_gray, cars);
    float cof = -1;
    for (size_t i = 0; i < cars.size(); i++)
    {
        Point center(cars[i].x + cars[i].width / 2, cars[i].y + cars[i].height / 2);

        //============================== PRINT % DECIMAL VALUE FOR CENTER OF DETECTED OBJECT
        cof = (double)center.x / (double)frame.size().width
                                     std::cout
              << cof << std::endl;
        //==============================

        ellipse(frame, center, Size(cars[i].width / 2, cars[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4);
        Mat carROI = frame_gray(cars[i]);
    }

    //-- Show what you got
    imshow("Capture - Car detection", frame);
}
