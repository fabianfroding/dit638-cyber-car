/*
 * Copyright (C) 2019  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
using namespace std;

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    std::vector<std::vector<cv::Point>> contours, polygons;
    std::vector<cv::Rect> rectangle;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat img, img_hsv, frame_threshold, detected_edges, blur, drawing;
    cv::Scalar color = cv::Scalar( 255,255,255 );
    int borderType=cv::BORDER_DEFAULT;
    double area=0,perimeter=0;

    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.i420 --width=640 --height=480" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;
	   // std::cout<<"hello!"<<std::flush;
            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
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
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
		    cv::cvtColor(img,img_hsv, CV_BGR2HSV);

                }
                sharedMemory->unlock();

                // TODO: Do something with the frame.
//		double sensitivity=0;
                double low_H=33, high_H=81, low_S=70, high_S=255, low_V=60, high_V=255;
		cv::GaussianBlur(img_hsv,blur,cv::Size(1,1),0,0,borderType);
		cv::inRange(blur, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);
		cv::Canny (frame_threshold, detected_edges, 0, 0, 5,true);
		cv::findContours(detected_edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		polygons.resize(contours.size());
		rectangle.resize(contours.size());

		//approximate the curve of the polygon
		for(size_t k = 0; k < contours.size(); k++){
        		cv::approxPolyDP(contours[k], polygons[k], 3, true);
			rectangle[k]=cv::boundingRect(polygons[k]);
		}
//		drawing=cv::Mat::zeros(contours.size(),CV_8UC3);

		//draw the rectangles which perimeter is long enough over the img frame
		for (size_t i=0; i<contours.size(); i++){
			area=rectangle[i].area();
			perimeter=cv::arcLength(contours[i],true);
			if(perimeter>70){
				cv::rectangle(img, rectangle[i].tl(), rectangle[i].br(), color, 1, 8, 0 );
				std::cout<< "DETECTED! Area= " <<area<< " | Perimeter = " << perimeter << std::endl << std::flush;
			}
		}
		// Display image.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

