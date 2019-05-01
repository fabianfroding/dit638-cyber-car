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
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112" << std::endl;
        return -1;
    }
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
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
            std::cout << "press [" << left << "] for left turn, [" << right << "] for right turn and [" << neutral << "] for neutral wheel position" << std::endl;
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