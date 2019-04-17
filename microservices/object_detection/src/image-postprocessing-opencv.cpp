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
#include "opencv/cv.h"

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.i420 --width=640 --height=480" << std::endl;
    }
    else
    {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
               /* cv::Mat img;
                cv::Mat Convimg;
                cv::Mat Thimg;
                cv::Mat Eimg;
		cv::Mat Nimg; */
		cv::Mat img;
		cv::Mat frame_HSV;
		cv::Mat edges;
		cv::Mat frame_threshold;

                int low_H = 40, low_S = 120, low_V = 120;
                int high_H = 75, high_S = 255, high_V = 255;

                std::vector<std::vector<cv::Point>> contours;
                std::vector<std::vector<cv::Point>> contoursC;
                cv::Mat curves;
                std::vector<cv::Vec4i> hierarchy;

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
                }
                sharedMemory->unlock();

                // TODO: Do something with the frame.

               
		//convert image (RGB) to HSV                
		cvtColor(img, frame_HSV, CV_RGB2HSV);
		//apply threshold
                cv::inRange(frame_HSV ,cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), frame_threshold);
		//blur to remove tiny unwanted spots
                blur(frame_threshold, frame_threshold, cv::Size(3, 3));
		//detect the edges
                cv::Canny(frame_threshold, edges, 10, 200,3);
		//find contours from edges
                findContours( edges, contoursC, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
		//draw polygon
                approxPolyDP(curves, contours, 1, true);

              
                // Display image.
                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), frame_threshold);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}