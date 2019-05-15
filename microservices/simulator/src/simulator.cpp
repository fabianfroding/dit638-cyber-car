/**
 * Copyright (C) 2019 Carlos, the car
*/

#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include "cluon-complete.hpp"
#include "envelopes.hpp"

int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if (0 != commandlineArguments.count("help"))
    {
        std::cerr << argv[0] << " is a simulator that tests tests how microservices react to events." << std::endl;
        std::cerr << "Usage:" << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--verbose]" << std::endl;
        std::cerr << argv[0] << "[--delay]" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--carlos=113 --verbose --object" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t DELAY{(commandlineArguments.count("delay") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["delay"])) : static_cast<uint16_t>(2000)};

    std::cout << "starting up " << argv[0] << "..." << std::endl;

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session

    if (carlos_session.isRunning())
    {
        std::cout << "Simulator engaged" << std::endl;

        int16_t userInp = 0;

        //vision
        carlos::object::sign sign_tracker;
        carlos::color::intersection intersection_tracker;

        // //acc
        // carlos::acc::collision collision_status;
        // carlos::acc::trigger trigger;

        // //command
        // carlos::cmd::path command;

        //delay
        std::chrono::milliseconds timer(DELAY);

        while (carlos_session.isRunning())
        {
            std::cout << "Welcome to the SIMULATOR" << std::endl
                      << "to simulate the events at the intersection enter:" << std::endl
                      << "[1] for stage 1" << std::endl
                      << "[2] for stage 2" << std::endl
                      << "[3] for stage 3" << std::endl;

            scanf("%hd", &userInp);
            switch (userInp)
            {
            case 1:
                /** stage 1
                * 1. trigger sign detected
                * 2. assign vehicles to the lanes
                */
                std::this_thread::sleep_for(timer);
                std::cout << "Detecting sign" << std::endl;

                sign_tracker.detected(true);
                sign_tracker.reached(false);
                break;
            case 2:
                /** stage 2
                * 1. trigger sign reached
                * 2. remove the vehicles from the lane
                */
                std::this_thread::sleep_for(timer);
                std::cout << "Reaching sign" << std::endl;

                sign_tracker.detected(false);
                sign_tracker.reached(true);

                std::this_thread::sleep_for(timer);
                std::cout << "Assesing lanes" << std::endl;

                intersection_tracker.north(true);
                intersection_tracker.east(false);
                intersection_tracker.west(true);
                break;
            case 3:
                /** stage 3
                * 1. trigger all cars have left
                * 2. prompt user to chose path
                */
                std::this_thread::sleep_for(timer);
                std::cout << "Tracking lanes" << std::endl;

                intersection_tracker.north(false);
                intersection_tracker.east(false);
                intersection_tracker.west(false);
                break;

            default:
                break;
            }
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
