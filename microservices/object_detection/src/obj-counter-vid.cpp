#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

#include "cluon-complete.hpp"
#include "messages.hpp"

#define SHOW_STREAM true
#define VIDEO "vid1.mp4" // Name of video file.

using namespace std;
using namespace cv;

int countObjects(Mat frame);
CascadeClassifier casc_classer;

int main(int argc, const char** argv) {
    CommandLineParser parser(argc, argv,
                             "{help h||}"
                             "{casc_classer|./cascade.xml|Path to cascade.}"
                             "{camera|0|Camera device number.}");
    //parser.printMessage();
    String casc_classer_name = parser.get<String>("casc_classer");
    
    // Load the cascade.
    if (!casc_classer.load(casc_classer_name)) {
        cout << "Error loading cascade." << endl;
        return -1;
    };

	// Read from VIDEO stream (for testing).
    VideoCapture capture(VIDEO);
    if (!capture.isOpened()) {
        cout << "Error opening video stream or file." << endl;
        return -1;
    }

    Mat frame;
    while (capture.read(frame)) {
        if (frame.empty()) {
            cout << "Error: No captured frame." << endl;
            break;
        }
        
        // Apply the classifier to the frame.
		int nObjects = countObjects(frame);
    	cout << nObjects << endl;

		//===============
		// SEND MESSAGE HERE
		//===============

        if (waitKey(10) == 27) {
            break; // escape
        }
    }
    return 0;
}

int countObjects(Mat frame) {
    Mat frame_gray;
	size_t nObjects; // Number of objects detected.

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, frame_gray);

    // Detect objects.
    std::vector<Rect> objects;
    casc_classer.detectMultiScale(frame_gray, objects);
	nObjects = objects.size();
    for (size_t i = 0; i < nObjects; i++) {
        Point center(objects[i].x + objects[i].width/2, objects[i].y + objects[i].height/2);
        ellipse(frame, center, Size(objects[i].width/2, objects[i].height/2), 0, 0, 360, Scalar(255, 0, 255), 4);
        Mat objectsROI = frame_gray(objects[i]);
    }

    // Display the result.
    if (SHOW_STREAM)
		imshow("Result", frame);
	
	// Return number of detected objects.	
    return static_cast<int>(nObjects);
}


