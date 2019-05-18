/**
 * Copyright (C) 2019 Carlos, the car
*/

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "envelopes.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
using namespace std;
using namespace cv;
using namespace cluon;

float carlos_converter(float num);
Point getCenterOfContour(vector<Point> contour);
double getAreaOfContour(vector<Point> contour);
double getPerimeterOfContour(vector<Point> contour);
void drawRectangle(Rect rectangle, Mat image, Scalar color);
float getPercentageOfWidth(vector<Point> contour, Mat image);
void printRectangleLocation(vector<Point> contour, Mat image);
vector<vector<Point>> getContours(Mat hsvImage, Scalar color_low, Scalar color_high);

int32_t main(int32_t argc, char **argv)
{
  CommandLineParser parser(argc, argv,
                           "{help h||}"
                           "{stopSigns_cascade|./cascade.xml|Path to car cascade.}"
                           "{camera|0|Camera device number.}");

  //**VARIABLES**//
  int32_t retCode{1};
  bool stopSignPresent=false, stopSignDetected=false, westCar=false, northCar=true, eastCar=false;
  uint16_t stage = 0; //uint16_t status=0;
  String stopSigns_cascade_name;
  CascadeClassifier stopSigns_cascade;
  vector<vector<Point>> car_contours, car_polygons, stop_contours, stop_polygons;
  vector<Rect> car_rectangle, stop_rectangle;
  vector<Vec4i> car_hierarchy, stop_hierarchy;
  Rect temp, empty;
  Mat img, img_hsv, car_frame_threshold, car_detected_edges, blur, resizedImg, img_higher_brightness, carROI, obj_frame, img2,resizedImg2;
  Scalar edge = Scalar(255, 255, 255);

  //car sticker colors
  double car_low_H = 40, car_high_H = 90, car_low_S = 60, car_high_S = 255, car_low_V = 51, car_high_V = 255; //,sensitivity=0;
  Scalar car_low = Scalar(car_low_H, car_low_S, car_low_V), car_high = Scalar(car_high_H, car_high_S, car_high_V);
  //**END VARIABLES**//

  stopSigns_cascade_name = parser.get<String>("stopSigns_cascade");
  if (!stopSigns_cascade.load(stopSigns_cascade_name))
  {
    cout << "--(!)Error loading car cascade\n";
    return -1;
  };

  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("name")) ||
      (0 == commandlineArguments.count("width")) ||
      (0 == commandlineArguments.count("height")))
  {
    cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << endl;
    cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << endl;
    cerr << "         --carlos:    CID of the Carlos Micro-Service session to send and receive messages" << endl;
    cerr << "         --name:   name of the shared memory area to attach" << endl;
    cerr << "         --width:  width of the frame" << endl;
    cerr << "         --height: height of the frame" << endl;
    cerr << "Example: " << argv[0] << " --carlos=113 --name=img.i420 --width=640 --height=480" << endl;
  }
  else
  {
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t CID_SESSION{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
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
      cluon::OD4Session carlos_session{CARLOS_SESSION};
      cluon::OD4Session kiwi_session{CID_SESSION};

      opendlv::proxy::GroundSteeringRequest wheel;
      carlos::color::lead_car lead_car;
      carlos::color::intersection intersection_tracker;
      carlos::color::status status;
      carlos::object::sign signStatus;
      // carlos::vision::sign sign_tracker;

      bool SEMAPHORE = true;

      /*prepared callback*/
      auto semaphore = [VERBOSE, &SEMAPHORE](cluon::data::Envelope &&envelope) {
        /** unpack message recieved*/
        auto msg = cluon::extractMessage<carlos::color::status>(std::move(envelope));
        /*store data*/
        SEMAPHORE = msg.semaphore();
      };
      /*registered callback*/
      carlos_session.dataTrigger(carlos::color::status::ID(), semaphore);
      
      // Variables for storing which cars were present before approaching the intersection.
      bool wasPresentBeforeWest = false;
      bool wasPresentBeforeNorth = false;
      bool wasPresentBeforeEast = false;
      
      //==============================
      // CHECK STAGES
      //==============================
      
      //==================== CODE FOR STAGE !
      if (stage == 1) {
		 // Variables to get average stop signs detected of every fifth frame.
		  float framesCounted = 0;
		  float objectsCounted = 0;
		  // Variables to get average green colors detected in west, north and east frames for every fifth frame.
		  float framesCountedColor = 0;
		  float colorsCountedWest = 0;
		  float colorsCountedNorth = 0;
		  float colorsCountedEast = 0;
		  // Endless loop; end the program by pressing Ctrl-C.
		  while (carlos_session.isRunning() || kiwi_session.isRunning())
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
		      Mat wrapped(HEIGHT - 110, WIDTH, CV_8UC4, sharedMemory->data());
		      img = wrapped.clone();
		      img2 = wrapped.clone();
		    }
		    sharedMemory->unlock();

		    resize(img, resizedImg, Size(static_cast<double>(img.cols) * 0.5, static_cast<double>(img.rows * 0.5)), 0, 0, CV_INTER_LINEAR);
		    resize(img2, resizedImg2, Size(static_cast<double>(img.cols) * 0.5, static_cast<double>(img2.rows * 0.5)), 0, 0, CV_INTER_LINEAR);
		    //resizedImg.convertTo(img_higher_brightness, -1, 1, 70); //increase the brightness by 20 for each pixel
		    cvtColor(resizedImg, img_hsv, CV_BGR2HSV);
		    cvtColor(resizedImg2, resizedImg2, COLOR_BGR2GRAY);
		    equalizeHist(resizedImg2, obj_frame); //equalize greyscale histogram

		    //==============================
			// OBJECT DETECTION
			//==============================
		    vector<Rect> stopSigns;
		    stopSigns_cascade.detectMultiScale(obj_frame, stopSigns);
		    size_t nStopSigns = stopSigns.size();
		    objectsCounted += (double)nStopSigns;
			framesCounted++;

			if (nStopSigns != 0) {
			  for (size_t i = 0; i < nStopSigns; i++) {
			    Point center(stopSigns[i].x + stopSigns[i].width / 2, stopSigns[i].y + stopSigns[i].height / 2);
			    ellipse(resizedImg, center, Size(stopSigns[i].width / 2, stopSigns[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4);
			  }
			}

			if (framesCounted >= 5) {
				int avgObjects = int((objectsCounted / 5) + 0.5);
				cout << "Average objects detected of last 5 frames: " << avgObjects << endl;
				framesCounted = 0;
				objectsCounted = 0;

				stopSignPresent = (0 < avgObjects) ? true : false;
				if (stopSignPresent) {
					stopSignDetected = true;
				}
				if (stopSignPresent && stopSignDetected) {
					signStatus.detected(true);
					signStatus.reached(false);
				} else if (!stopSignPresent && stopSignDetected) {
					signStatus.detected(false);
					signStatus.reached(true);
				}
				if (stopSignDetected) {
					carlos_session.send(signStatus);
				}
			}
		    cout << "Stop sign present: " << stopSignPresent << "| Stop sign detected: " << stopSignDetected << flush << endl;
		    //==============================

		    car_contours = getContours(img_hsv, car_low, car_high);
		    car_polygons.resize(car_contours.size());
		    car_rectangle.resize(car_contours.size());

		    //**PROCESS CAR GREEN DETECTION**
		    eastCar=false; northCar=false; westCar=false;
		      for (size_t k = 0; k < car_contours.size(); k++)
		      {
		        approxPolyDP(car_contours[k], car_polygons[k], 3, true); //approximate the curve of the polygon
		        if (arcLength(car_contours[k], false) > 60)
		        {
		          car_rectangle[k] = boundingRect(car_polygons[k]);
		          //printRectangleLocation(car_contours[k], resizedImg); //coordinates and position of the center of each rectangle
		          if(getCenterOfContour(car_contours[k]).x < resizedImg.size().width/100*30) westCar=true;
		          //else westCar=false;
		          if(getCenterOfContour(car_contours[k]).x >= resizedImg.size().width/100*30 && getCenterOfContour(car_contours[k]).x <= resizedImg.size().width/100*65) northCar=true;
		          //else northCar=false;
		          if(getCenterOfContour(car_contours[k]).x > resizedImg.size().width/100*65) eastCar=true;
		          //else eastCar=false;
		        }
		        groupRectangles(car_rectangle, 3, 0.65); //group overlapping rectangles
		        drawRectangle(car_rectangle[k], resizedImg, edge);
		      }
		        cout <<westCar<<" | "<<northCar<<" | "<<eastCar<<flush<<endl;
		        
		        //==================== GET AVERAGE OF COLORS DETECTED (5th frame)
		        colorsCountedWest += (double)westCar;
		        colorsCountedNorth += (double)northCar;
		        colorsCountedEast += (double)eastCar;
				framesCountedColor++;
		        if (framesCountedColor == 5) {
					int avgCarsWest = int((colorsCountedWest / 5) + 0.5);
					int avgCarsNorth = int((colorsCountedNorth / 5) + 0.5);
					int avgCarsEast = int((colorsCountedEast / 5) + 0.5);
					cout << "Avg cars west: " << avgCarsWest << endl;
					cout << "Avg cars north: " << avgCarsNorth << endl;
					cout << "Avg cars east: " << avgCarsEast << endl;
					
					//send intersection message
				    intersection_tracker.west(avgCarsWest);
				    intersection_tracker.north(avgCarsNorth);
				    intersection_tracker.east(avgCarsEast);
				    carlos_session.send(intersection_tracker);
				    
				    // Store which cars were present and not (so they can be retrieved in stage 2)
					if (avgCarsWest >= 1) wasPresentBeforeWest = true;
					if (avgCarsNorth >= 1) wasPresentBeforeNorth = true;
					if (avgCarsEast >= 1) wasPresentBeforeEast = true;
				}
				
				//====================

		    //message sending stopped
		    // Display image.
		    if (VERBOSE)
		    {
		      imshow(sharedMemory->name().c_str(), resizedImg);
		      //  cout << "RECIEVED -> SEMAPHORE_KEY [" << SEMAPHORE_KEY << "]" << endl;
		      waitKey(1);
		    }
		  }
		  	
      }
      
      //==================== CODE FOR STAGE 2
      // Note: stage 2 does not use object detection since it already passed the stop sign.
      else if (stage == 2) {
      	
      	// TO DO:
      	// Check frames for north and east car IF THEY WERE PRESENT IN STAGE 1.
      	
      	framesCountedColor = 0; // Reset from usage in stage 1.
      	
      	// This section is almost identical to the one in stage 1, except it doesnt look for the car in west.
      	while (carlos_session.isRunning() || kiwi_session.isRunning())
      	{
		    sharedMemory->wait();
		    sharedMemory->lock();
		    {
		      Mat wrapped(HEIGHT - 110, WIDTH, CV_8UC4, sharedMemory->data());
		      img = wrapped.clone();
		      img2 = wrapped.clone();
		    }
		    sharedMemory->unlock();

		    resize(img, resizedImg, Size(static_cast<double>(img.cols) * 0.5, static_cast<double>(img.rows * 0.5)), 0, 0, CV_INTER_LINEAR);
		    resize(img2, resizedImg2, Size(static_cast<double>(img.cols) * 0.5, static_cast<double>(img2.rows * 0.5)), 0, 0, CV_INTER_LINEAR);
		    cvtColor(resizedImg, img_hsv, CV_BGR2HSV);
		    cvtColor(resizedImg2, resizedImg2, COLOR_BGR2GRAY);
		    equalizeHist(resizedImg2, obj_frame);
      	
      		car_contours = getContours(img_hsv, car_low, car_high);
		    car_polygons.resize(car_contours.size());
		    car_rectangle.resize(car_contours.size());

		    //**PROCESS CAR GREEN DETECTION**
		    eastCar=false; northCar=false; westCar=false;
		      for (size_t k = 0; k < car_contours.size(); k++)
		      {
		        approxPolyDP(car_contours[k], car_polygons[k], 3, true);
		        if (arcLength(car_contours[k], false) > 60)
		        {
		          car_rectangle[k] = boundingRect(car_polygons[k]);
		          if(getCenterOfContour(car_contours[k]).x >= resizedImg.size().width/100*30 && getCenterOfContour(car_contours[k]).x <= resizedImg.size().width/100*65) northCar=true;
		          if(getCenterOfContour(car_contours[k]).x > resizedImg.size().width/100*65) eastCar=true;
		        }
		        groupRectangles(car_rectangle, 3, 0.65); //group overlapping rectangles
		        drawRectangle(car_rectangle[k], resizedImg, edge);
		      }
		        cout <<northCar<<" | "<<eastCar<<flush<<endl;
		        colorsCountedNorth += (double)northCar;
		        colorsCountedEast += (double)eastCar;
				framesCountedColor++;
		        if (framesCountedColor >= 4) {
					int avgCarsNorth = int((colorsCountedNorth / 5) + 0.5);
					int avgCarsEast = int((colorsCountedEast / 5) + 0.5);
					cout << "Avg cars north: " << avgCarsNorth << endl;
					cout << "Avg cars east: " << avgCarsEast << endl;
					
					// Here we send the status of north and east car depending on if they were present during stage 1.
					// If a car was not present in stage 1, we dont send anything since we dont care.
					if (wasPresentBeforeNorth || wasPresentBeforeEast) {
						if (wasPresentBeforeNorth) {
							intersection_tracker.north(avgCarsNorth);
						}
						if (wasPresentBeforeEast) {
							intersection_tracker.east(avgCarsEast);
						}
						carlos_session.send(intersection_tracker);
					}
					
					// For stage 2, we want to repeatedly check for the cars, unlike in stage 1. So we need to reset the average frame counter.
					framesCountedColor = 0;
					avgCarsNorth = 0;
					avgCarsEast = 0;
					
				}
				
				//====================

		    //message sending stopped
		    // Display image.
		    if (VERBOSE)
		    {
		      imshow(sharedMemory->name().c_str(), resizedImg);
		      //  cout << "RECIEVED -> SEMAPHORE_KEY [" << SEMAPHORE_KEY << "]" << endl;
		      waitKey(1);
		    }
      	
      	// Here goes logic for the LEFT car.
      	
      }

	  
    }
  }
  return retCode;
}

float carlos_converter(float num)
{
  float res = 0;
  float wheel_range = 1.2;
  res = ((num * 100) * wheel_range) / 100;
  res = -1 * (res - (wheel_range / 2));
  return static_cast<float>(res);
}

//return the center of a contour -> Point(x,y)
Point getCenterOfContour(vector<Point> contour)
{
  Point center = minAreaRect(contour).center;
  return center;
}

//area
double getAreaOfContour(vector<Point> contour)
{
  return contourArea(contour);
}

//perimeter
double getPerimeterOfContour(vector<Point> contour)
{
  return arcLength(contour, true);
}
//draw the rectangle on the frame
void drawRectangle(Rect rectangle, Mat image, Scalar color)
{
  cv::rectangle(image, rectangle.tl(), rectangle.br(), color, 1, 8, 0);
}

float getPercentageOfWidth(vector<Point> contour, Mat image)
{
  float x, percentage; //y;
                       //  Point centerOfObject;
  x = getCenterOfContour(contour).x;
  //  y=getCenterOfContour(contour).y;
  //  centerOfObject=minAreaRect(contour).center;
  percentage = x / static_cast<float>(image.size().width); //percentage till the end of the frame

  return percentage;
}

//print the location of a detected color according to its name
void printRectangleLocation(vector<Point> contour, Mat image)
{
  float x, y, percentage = 0;
  Point centerOfObject;
  x = getCenterOfContour(contour).x;
  y = getCenterOfContour(contour).y;
  centerOfObject = minAreaRect(contour).center;      //center of the rectangle <Point>
  percentage = getPercentageOfWidth(contour, image); //percentage till the end of the frame
  if (x <= (image.size().width) / 100*30) //center is on the left
    cout << "Detected CAR - LEFT |" << x << "," << y << "|"
         << " %" << static_cast<int>(percentage * 100) << " - Area= " << getAreaOfContour(contour) << " - Perimeter= " << getPerimeterOfContour(contour) << flush << endl;
  else if (x >= (image.size().width) /100*63) //center on the right
    cout << "Detected CAR - RIGHT |" << x << "," << y << "|"
         << " %" << static_cast<int>(percentage * 100) << " - Area= " << getAreaOfContour(contour) << " - Perimeter= " << getPerimeterOfContour(contour) << flush << endl;
  else //center in the middle
    cout << "Detected CAR - CENTER |" << x << "," << y << "|"
         << " %" << static_cast<int>(percentage * 100) << " - Area= " << getAreaOfContour(contour) << " - Perimeter= " << getPerimeterOfContour(contour) << flush << endl;
}

//get contours of a detected range of colors in hsv image
vector<vector<Point>> getContours(Mat hsvImage, Scalar color_low, Scalar color_high)
{
  Mat blur, frame_threshold, detected_edges;
  vector<Vec4i> hierarchy;
  vector<vector<Point>> contours;
  medianBlur(hsvImage, blur, 11);
  inRange(blur, Scalar(color_low), Scalar(color_high), frame_threshold);
  Canny(frame_threshold, detected_edges, 0, 0, 5, false);
  findContours(detected_edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
  //cout<<"there are "<<hierarchy.size()<<" objects detected"<<endl<<flush;
  return contours;
}
