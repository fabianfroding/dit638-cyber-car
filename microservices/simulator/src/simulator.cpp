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
        std::cerr << argv[0] << "[--sign] test sign" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--carlos=113 --verbose" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t DELAY{(commandlineArguments.count("delay") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["delay"])) : static_cast<uint16_t>(2000)};

    std::cout << "starting up " << argv[0] << "..." << std::endl;

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session

    if (carlos_session.isRunning())
    {
        std::cout << "Simulator engaged" << std::endl;

        //command
        carlos::cmd::turn_status direction_sign;
        direction_sign.west_turn(true);
        direction_sign.north_turn(true);
        direction_sign.east_turn(true);
        //sign
        carlos::object::sign sign_tracker;

        //intersection
        carlos::color::intersection intersection_tracker;

        //delay
        std::chrono::milliseconds timer(DELAY);

        while (carlos_session.isRunning())
        {
            std::cout << "Welcome to the SIMULATOR" << std::endl
                      << "enter simulation:" << std::endl
                      << "[1] for stage" << std::endl
                      << "[2] for intersection turns" << std::endl;
            int16_t option = 0;
            scanf("%hd", &option);

            if (option == 1)
            {
                std::cout << "enter stage:" << std::endl
                          << "[1] for stage 1" << std::endl
                          << "[2] for stage 2" << std::endl
                          << "[3] for stage 3" << std::endl;

                int16_t userInp = 0;
                scanf("%hd", &userInp);
                if (userInp != 1 && userInp != 2 && userInp != 3)
                {
                    return 1;
                }
                if (userInp == 1)
                { /** stage 1
                * 1. trigger stop sign detected
                * 2. asses no turn signs 
                * 3. assign vehicles to the lanes
                */
                    sign_tracker.detected(false);
                    sign_tracker.reached(false);

                    std::cout << "Detecting sign" << std::endl;
                    sign_tracker.detected(true);
                    sign_tracker.reached(false);
                    carlos_session.send(sign_tracker);
                    std::this_thread::sleep_for(timer);
                }
                if (userInp == 2)
                { /** stage 2
                * 1. trigger sign reached
                * 2. remove the vehicles from the lane
                */
                    std::cout << "Reaching sign" << std::endl;
                    sign_tracker.detected(false);
                    sign_tracker.reached(true);
                    carlos_session.send(sign_tracker);
                    std::this_thread::sleep_for(timer);

                    std::cout << "Assesing lanes" << std::endl;
                    intersection_tracker.north(true);
                    intersection_tracker.east(false);
                    intersection_tracker.west(true);
                    carlos_session.send(intersection_tracker);
                    std::this_thread::sleep_for(timer);
                }
                if (userInp == 3)
                { /** stage 3
                * 1. trigger all cars have left
                * 2. prompt user to chose path
                */
                    std::cout << "Tracking lanes" << std::endl;
                    intersection_tracker.north(false);
                    intersection_tracker.east(false);
                    intersection_tracker.west(false);
                    carlos_session.send(intersection_tracker);
                    std::this_thread::sleep_for(timer);
                }
            }

            if (option == 2)
            {
                int16_t west_sign = -1, north_sign = -1, east_sign = -1;
                std::cout << "Choose whether the lanes are accesible or not" << std::endl;
                std::cout << "enter [1] to lock or [0] to unlock the west lane" << std::endl;
                scanf("%hd", &west_sign);
                std::cout << "enter [1] to lock or [0] to unlock the north lane" << std::endl;
                scanf("%hd", &north_sign);
                std::cout << "enter [1] to lock or [0] to unlock the east lane" << std::endl;
                scanf("%hd", &east_sign);

                if ((west_sign < 0 && west_sign > 1) || (north_sign < 0 && north_sign > 1) || (east_sign < 0 && east_sign > 1))
                {
                    std::cout << "cannot use those values" << std::endl;
                    return 1;
                }

                if (west_sign == 1)
                {
                    direction_sign.west_turn(false);
                }
                else
                {
                    direction_sign.west_turn(true);
                }

                if (north_sign == 1)
                {
                    direction_sign.north_turn(false);
                }
                else
                {
                    direction_sign.north_turn(true);
                }

                if (east_sign == 1)
                {
                    direction_sign.east_turn(false);
                }
                else
                {
                    direction_sign.east_turn(true);
                }

                carlos_session.send(direction_sign);
                std::this_thread::sleep_for(timer);
            }

            if (option != 1 && option != 2)
            {
                return 1;
            }
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
