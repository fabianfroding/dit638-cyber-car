/*
 * Copyright (C) 2019 Yue Kang
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
#include "messages.hpp"

#include <cstdint>
#include <chrono>
#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>

int32_t main(int32_t argc, char **argv) {

    // Parse the arguments from the command line
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ( (0 == commandlineArguments.count("cid")) 
        || (0 == commandlineArguments.count("name")) 
        || (0 != commandlineArguments.count("help")) )
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> --name=<shared memory area>] ";
        std::cerr << "[--width=<image width>] [--height=<image height>] [--verbose] [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --name=img.i420 --verbose" << std::endl;
        return -1;
    }
    else
    {
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        if (0 == od4.isRunning())
        {
            std::cerr << "ERROR: No OD4Session running!!!" << std::endl;
            return -2;
        }

        const std::string NAME{commandlineArguments["name"]};
        std::unique_ptr<cluon::SharedMemory> shm(new cluon::SharedMemory{NAME});
        if ( (!shm) || (!(shm->valid())) )
        {
            std::cerr << "The shared memory " << NAME << " can not be reachable!" << std::endl;
            return -3;
        }

        //const float FREQ{(commandlineArguments["freq"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["freq"])) : static_cast<float>(-1.0)};

        const uint32_t WIDTH{(commandlineArguments["width"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["width"])) : 640};
        const uint32_t HEIGHT{(commandlineArguments["height"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["height"])) : 480};

        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        /*
         * Equation of RGB to YUV conversion
           Y = 0.299 * R + 0.587 * G + 0.114 * B
           U = -0.169 * R - 0.331 * G + 0.5 * B + 128
           V = 0.5 * R - 0.419 * G - 0.081 * B + 128
         */

        /* Yellow: (255, 255, 0) in RGB */
        const uint8_t Y = (uint8_t)(std::round(0.299 * 255 + 0.587 * 255 + 0.114 * 0));
        const uint8_t U = (uint8_t)(std::round(-0.169 * 255 - 0.331 * 255 + 0.5 * 0 + 128));
        const uint8_t V = (uint8_t)(std::round(0.5 * 255 - 0.419 * 255 - 0.081 * 0 + 128));


        while(od4.isRunning() && shm && shm->valid())
        {
            shm->wait(); // Wait for shared memory notice from source
            shm->lock(); // Freeze the current frame
            uint8_t *dataPtr = reinterpret_cast<uint8_t *>(shm->data()); // Pointer at the first pixel of the image

            // Draw a yellow square directly on the image
            // Left-top corner coordinate: (30, 20)
            // Right-bottom corner coordinate: (80, 60)
            // Coordinate direction: line first, from left to right, from top to bottom
            // Here we are using i420 format, so pay attention to the coordinates and offsets
            for (uint32_t i = 30; i <= 80; ++i)
            {
                // Horizontal line 1
                *(dataPtr + 20 * WIDTH + i) = Y;
                *(dataPtr + WIDTH * HEIGHT + 5 * WIDTH + (i / 2)) = U;
                *(dataPtr + WIDTH * HEIGHT + ((WIDTH * HEIGHT) >> 2) + 5 * WIDTH + (i / 2)) = V;

                // Horizontal line 2
                *(dataPtr + 60 * WIDTH + i) = Y;
                *(dataPtr + WIDTH * HEIGHT + 15 * WIDTH + (i / 2)) = U;
                *(dataPtr + WIDTH * HEIGHT + ((WIDTH * HEIGHT) >> 2) + 15 * WIDTH + (i / 2)) = V;
            }

            for (uint32_t i = 20; i <= 60; ++i)
            {
                // Vertical line 1
                *(dataPtr + i * WIDTH + 30) = Y;
                *(dataPtr + WIDTH * HEIGHT + (i / 4) * WIDTH + 15) = U;
                *(dataPtr + WIDTH * HEIGHT + ((WIDTH * HEIGHT) >> 2) + (i / 4) * WIDTH + 15) = V;

                // Vertical line 2
                *(dataPtr + i * WIDTH + 80) = Y;
                *(dataPtr + WIDTH * HEIGHT + (i / 4) * WIDTH + 40) = U;
                *(dataPtr + WIDTH * HEIGHT + ((WIDTH * HEIGHT) >> 2) + (i / 4) * WIDTH + 40) = V;
            }

            //TODO: Insert your OpenCV-based algo here

            if (VERBOSE)
            {
                std::cout << "Output necessary verbose information of your own here." << std::endl;
            }

            shm->unlock(); // Free the occupancy of the shared memory
            shm->notifyAll(); // Tell othere that they could lock() now
        }
    }
    return 0;
}