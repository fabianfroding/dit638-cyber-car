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

float autoPedal(float sensor, float safetyDistance, float maxSpeed, float currentSpeed, float neutral, bool VERBOSE);

int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ((0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << " --cid=<CID of your OD4Session>" << std::endl;
        std::cerr << argv[0] << "[--fb=<front/back safety space>]" << std::endl;
        std::cerr << argv[0] << "[--lr=<left/right safety space>]" << std::endl;
        std::cerr << argv[0] << "[--sp=<speed, min is 0.13 and max is 0.8>]" << std::endl;
        std::cerr << argv[0] << "[--tr=< turn, min is 0.00 and max is 0.5>]" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --fb=0.2 --lr=0.2 --sp=013 --tr=0.2" << std::endl;
        return -1;
    }
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(646)};
    const float FB{(commandlineArguments.count("fb") != 0) ? static_cast<float>(std::stof(commandlineArguments["fb"])) : static_cast<float>(0.20)};
    const float MAXSPEED{(commandlineArguments.count("sp") != 0) ? static_cast<float>(std::stof(commandlineArguments["sp"])) : static_cast<float>(0.70)};
    const float TURN{(commandlineArguments.count("tr") != 0) ? static_cast<float>(std::stof(commandlineArguments["tr"])) : static_cast<float>(0.50)};
    // const float LR{(commandlineArguments.count("lr") != 0) ? static_cast<float>(std::stof(commandlineArguments["lr"])) : static_cast<float>(0.20)};

    std::cout << "starting up " << argv[0] << "..." << std::endl;
    std::cout << "speed: [" << MAXSPEED << "], turn angle: [" << TURN << "], front saftey distance: [" << FB << "]" << std::endl;
    std::cout << "oli" << std::endl;
    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session car{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    cluon::OD4Session carlos{CARLOS_SESSION};
    if (car.isRunning() && carlos.isRunning())
    {
        //prepare messages to recieve from carlos session

        //prep messages to send to car session
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
