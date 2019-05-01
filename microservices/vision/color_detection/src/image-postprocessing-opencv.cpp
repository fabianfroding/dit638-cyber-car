/*
 * Copyright (C) 2019  Carlos
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
using namespace std;
using namespace cv;

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};

  Point centerOfCar, centerOfStop;
  vector<vector<Point>> car_contours, car_polygons, stop_contours, stop_polygons;
  vector<Rect> car_rectangle, stop_rectangle;
  vector<Vec4i> car_hierarchy, stop_hierarchy;
  Rect temp;
  Mat img, img_hsv, car_frame_threshold, car_detected_edges, blur;
  Mat stop_frame_threshold, stop_detected_edges;
  Scalar color = Scalar(0, 255, 0);
  Scalar color2 = Scalar(0, 0, 255);
  float car_x, car_y, stop_x, stop_y, percentage;
  double area = 0, perimeter = 0;                                                                                     //,perimeter=0, maxArea=0;
  double stop_low_H = 130, stop_high_H = 166, stop_low_S = 87, stop_high_S = 255, stop_low_V = 69, stop_high_V = 255; //,sensitivity=0;
  double car_low_H = 40, car_high_H = 80, car_low_S = 85, car_high_S = 255, car_low_V = 80, car_high_V = 255;         //,sensitivity=0;

  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height")))
  {
    cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << endl;
    cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << endl;
    cerr << "         --cid:    CID of the OD4Session to send and receive messages" << endl;
    cerr << "         --name:   name of the shared memory area to attach" << endl;
    cerr << "         --width:  width of the frame" << endl;
    cerr << "         --height: height of the frame" << endl;
    cerr << "Example: " << argv[0] << " --cid=112 --name=img.i420 --width=640 --height=480" << endl;
  }
  else
  {
    const string NAME{commandlineArguments["name"]};
    const uint32_t WIDTH{static_cast<uint32_t>(stoi(commandlineArguments["width"]))};
    const uint32_t HEIGHT{static_cast<uint32_t>(stoi(commandlineArguments["height"]))};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};

    // Attach to the shared memory.
    unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
    if (sharedMemory && sharedMemory->valid())
    {
      clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << endl;
      // cout<<"hello!"<<flush;
      // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
      cluon::OD4Session od4{static_cast<uint16_t>(stoi(commandlineArguments["cid"]))};

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning())
      {
        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
          // Copy image into cvMat structure.
          // Be aware of that any code between lock/unlock is blocking
          // the camera to provide the next frame. Thus, any
          // computationally heavy algorithms should be placed outside
          // lock/unlock.
          Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
          img = wrapped.clone();
          cvtColor(img, img_hsv, CV_BGR2HSV);
        }
        sharedMemory->unlock();

        // TODO: Do something with the frame.
        //GaussianBlur(img_hsv,blur,Size(1,1),0,0,borderType);
        medianBlur(img_hsv, blur, 11);
        inRange(blur, Scalar(car_low_H, car_low_S, car_low_V), Scalar(car_high_H, car_high_S, car_high_V), car_frame_threshold);
        inRange(blur, Scalar(stop_low_H, stop_low_S, stop_low_V), Scalar(stop_high_H, stop_high_S, stop_high_V), stop_frame_threshold);
        Canny(car_frame_threshold, car_detected_edges, 0, 0, 5, true);
        Canny(stop_frame_threshold, stop_detected_edges, 0, 0, 5, true);
        findContours(car_detected_edges, car_contours, car_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(stop_detected_edges, stop_contours, stop_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

        car_polygons.resize(car_contours.size());
        car_rectangle.resize(car_contours.size());

        stop_polygons.resize(stop_contours.size());
        stop_rectangle.resize(stop_contours.size());

        for (size_t k = 0; k < stop_contours.size(); k++)
        {
          approxPolyDP(stop_contours[k], stop_polygons[k], 3, true);
          if (boundingRect(stop_polygons[k]).area() > 100 && arcLength(stop_contours[k], true) > 100)
          {
            stop_rectangle[k] = boundingRect(stop_polygons[k]);

            stop_y = minAreaRect(stop_contours[k]).center.y;
            stop_x = minAreaRect(stop_contours[k]).center.x;
            centerOfStop = minAreaRect(stop_contours[k]).center; //center of the rectangle <Point>

            if (stop_x <= (img.size().width) / 3) //center is on the left
              cout << "Detected STOP SIGN - LEFT |" << stop_x << "," << stop_y << "|" << flush << endl;
            else if (stop_x >= (img.size().width) / 3 * 2) //center on the right
              cout << "Detected STOP SIGN - RIGHT |" << stop_x << "," << stop_y << "|" << flush << endl;
            else //center in the middle
              cout << "Detected STOP SIGN - CENTER |" << stop_x << "," << stop_y << "|" << flush << endl;
          }
          groupRectangles(stop_rectangle, 3, 0.8); //group overlapping rectangles
          cv::rectangle(img, stop_rectangle[k].tl(), stop_rectangle[k].br(), color2, 2, 8, 0);
        }

        for (size_t k = 0; k < car_contours.size(); k++)
        {
          approxPolyDP(car_contours[k], car_polygons[k], 3, true); //approximate the curve of the polygon
          area = boundingRect(car_polygons[k]).area();
          perimeter = arcLength(car_contours[k], true);
          if (area > 100 && perimeter > 100) //filter by area
          {
            car_rectangle[k] = boundingRect(car_polygons[k]); //generate boundingrect for each closed contour
            //coordinates of the center of each rectangle
            car_x = minAreaRect(car_contours[k]).center.x;
            car_y = minAreaRect(car_contours[k]).center.y;
            centerOfCar = minAreaRect(car_contours[k]).center;         //center of the rectangle <Point>
            percentage = car_x / static_cast<float>(img.size().width); //percentage till the end of the frame
            if (car_x <= (img.size().width) / 3)                       //center is on the left
              cout << "Detected CAR - LEFT |" << car_x << "," << car_y << "|"
                   << " %" << static_cast<int>(percentage * 100) << flush << endl;
            else if (car_x >= (img.size().width) / 3 * 2) //center on the right
              cout << "Detected CAR - RIGHT |" << car_x << "," << car_y << "|"
                   << " %" << static_cast<int>(percentage * 100) << flush << endl;
            else //center in the middle
              cout << "Detected CAR - CENTER |" << car_x << "," << car_y << "|"
                   << " %" << static_cast<int>(percentage * 100) << flush << endl;
          }
          groupRectangles(car_rectangle, 3, 0.8); //group overlapping rectangles
          cv::rectangle(img, car_rectangle[k].tl(), car_rectangle[k].br(), color, 2, 8, 0);

          //** OLIVER HERE**//
          //cout<<" % "<<percentage<<endl;
          //<double>area, <float>percentage and <double>perimeter are the variables you need
          //** END OLIVER HERE**//
        }

        // Display image.
        if (VERBOSE)
        {
          imshow(sharedMemory->name().c_str(), img);
          waitKey(1);
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}
