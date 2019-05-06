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
        std::cerr << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--cid=<ID of KIWI session>]" << std::endl;
        std::cerr << argv[0] << "[--turn=<turn angle>]" << std::endl;
        std::cerr << argv[0] << "[--verbose] print information" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--cid=112 --carlos=113 --verbose" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t CID_SESSION{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
    const float TURN{(commandlineArguments.count("turn") != 0) ? static_cast<float>(std::stof(commandlineArguments["turn"])) : static_cast<float>(0.2)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};

    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
        std::cout << "turn: [" << TURN << "]" << std::endl;
    }

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session carlos_session{CARLOS_SESSION};
    cluon::OD4Session car_session{CID_SESSION};

    if (car_session.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "session started..." << std::endl;
        }

        /**
        * set up messages that you might send
        */

        carlos::command cmd;                         //[carlos] turn_type
        opendlv::proxy::GroundSteeringRequest wheel; //[car] groundSteering

        bool SEMAPHORE_KEY = true;
        const int16_t left = 2, right = 1, neutral = 0;
        const float turnRight = TURN * -1, turnLeft = TURN, turnStraight = 0;
        int userInp = -1;

        auto semaphore = [VERBOSE, &SEMAPHORE_KEY](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::semaphore::cmd>(std::move(envelope));
            /*store data*/
            SEMAPHORE_KEY = msg.semaphore();

            if (VERBOSE)
            {
                std::cout << "RECIEVED -> SEMAPHORE_KEY [" << SEMAPHORE_KEY << "]" << std::endl;
            }
        };

        carlos_session.dataTrigger(carlos::semaphore::cmd::ID(), semaphore);

        while (car_session.isRunning())
        {
            std::cout << "press: " << std::endl;
            std::cout << "[" << left << "] for left turn" << std::endl;
            std::cout << "[" << right << "] for right turn" << std::endl;
            std::cout << "[" << neutral << "] for neutral" << std::endl;
            //take in input
            scanf("%d", &userInp);

            switch (userInp)
            {
            case left:
                cmd.turn_type(left);
                wheel.groundSteering(turnLeft);
                std::cout << "Wheel is turning [Left]" << std::endl;
                break;
            case right:
                cmd.turn_type(right);
                wheel.groundSteering(turnRight);
                std::cout << "Wheel is turning [Right]" << std::endl;
                break;
            case neutral:
                cmd.turn_type(neutral);
                wheel.groundSteering(turnStraight);
                std::cout << "Wheel is [Straight]" << std::endl;
                break;
            }
            //send data
            if (SEMAPHORE_KEY)
            {
                car_session.send(wheel);
            }

            carlos_session.send(cmd);
        }
    }
    else
    {
        std::cout << "Carlos Out. (Sarlos Session timed out.)" << std::endl;
    }
    return 0;
}