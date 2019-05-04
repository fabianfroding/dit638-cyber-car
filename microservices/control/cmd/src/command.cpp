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

    if ((0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:" << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--tr=<turn angle>]" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --carlos=646 --sd=0.2 --sp=013" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const float TURN{(commandlineArguments.count("tr") != 0) ? static_cast<float>(std::stof(commandlineArguments["tr"])) : static_cast<float>(0.20)};

    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
        std::cout << "turn: [" << TURN << "]" std::endl;
    }

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session command{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    if (command.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "session started..." << std::endl;
        }

        /**
        * set up messages that you might send
        */
        carlos::command comm;
        const int16_t left = 2, right = 1, neutral = 0;
        int userInp = -1;
        while (command.isRunning())
        {
            std::cout << "press: " << std::endl;
            std::cout << "[" << left << "] for left turn" << std::endl;
            std::cout << "[" << right << "] for right turn" << std::endl;
            std::cout << "[" << neutral << "] for neutral" << std::endl;
            //take in input
            scanf("%d", &userInp);
            comm.type(static_cast<uint16_t>(userInp));
            //send input
            command.send(comm);
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}