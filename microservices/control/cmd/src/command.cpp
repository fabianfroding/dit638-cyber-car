/**
 * Copyright (C) 2019 Carlos, the car
*/

#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include "cluon-complete.hpp"
#include "messages.hpp"



int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if (0 != commandlineArguments.count("help"))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << argv[0] << "[--cid=<ID of KIWI session>]" << std::endl;
        std::cerr << argv[0] << "[--verbose] print information" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--cid=112 --carlos=113 --verbose" << std::endl;
        return -1;
    }
    const uint16_t CID_SESSION{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
    const uint16_t DELAY{(commandlineArguments.count("delay") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["delay"])) : static_cast<uint16_t>(1)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};

    uint16_t dir = 12;

    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
    }

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session car_session{CID_SESSION};

    if (car_session.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "session started..." << std::endl;
        }

        /**
        * set up messages that you might send
        * "no messages will be sent in this microservice"
        */

        //carlos::command cmd;                         //[carlos] turn_type
        opendlv::proxy::GroundSteeringRequest wheel; //[car] groundSteering
        opendlv::proxy::PedalPositionRequest pedal;  //[car] pedal position
      
        while (car_session.isRunning())
        {
            std::cout << "Direction (9, 12, 3):" << std::endl;
            scanf("%d", &dir);

            switch (dir)
            {
                case 9:
                std::cout << "carlos turning left" << std::endl;
                //turn wheel
                wheel.groundSteering(0.14);
                car_session.send(wheel);

                //speed
                pedal.position(0.13);
                car_session.send(pedal);

                //delay
                std::chrono::milliseconds timer(3); // or whatever
                std::this_thread::sleep_for(timer);

                //stop vehicle
                pedal.position(0);
                car_session.send(pedal);

                //straighten wheel
                wheel.groundSteering(0);
                car_session.send(wheel);                
                break;

                case 12:
                std::cout << "carlos turning right" << std::endl;
                //turn wheel
                wheel.groundSteering(0);
                car_session.send(wheel);

                //speed
                pedal.position(0.12);
                car_session.send(pedal);

                //delay
                std::chrono::milliseconds timer(3); // or whatever
                std::this_thread::sleep_for(timer);

                //stop vehicle
                pedal.position(0);
                car_session.send(pedal);

                //straighten wheel
                wheel.groundSteering(0);
                car_session.send(wheel);
                break;

                case 3:
                std::cout << "carlos turning right" << std::endl;
                //turn wheel
                wheel.groundSteering(-0.25);
                car_session.send(wheel);

                //speed
                pedal.position(0.13);
                car_session.send(pedal);

                //delay
                std::chrono::milliseconds timer(3); // or whatever
                std::this_thread::sleep_for(timer);

                //stop vehicle
                pedal.position(0);
                car_session.send(pedal);

                //straighten wheel
                wheel.groundSteering(0);
                car_session.send(wheel);
                break;
            }

            break;
        }
    }
    else
    {
        std::cout << "Carlos Out. (Sarlos Session timed out.)" << std::endl;
    }
    return 0;
}
