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
        std::cerr << "Usage:" << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--freq=<frequency>]" << std::endl;
        std::cerr << argv[0] << "[--verbose" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--carlos=113 --freq=30" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const float FREQ{(commandlineArguments.count("freq") != 0) ? static_cast<float>(std::stof(commandlineArguments["freq"])) : static_cast<float>(30)};
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
    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session

    if (carlos_session.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "[delegator] micro-service started..." << std::endl;
        }

        /*global variables*/ //sign vision
        carlos::semaphore::acc acc_semaphore;
        carlos::semaphore::cmd cmd_semaphore;
        carlos::semaphore::vision::color color_semaphore;
        carlos::semaphore::vision::object object_semaphore;
        const bool LOCK = false;
        const bool UNLOCK = true;

        /* prepare messages to recieve from carlos session */
        auto adaptive_cruise_control = [VERBOSE](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            bool acc_danger = msg.safe_to_drive();
            /*do stuff with data*/

            if (acc_danger)
            {
                /*[test] lock wheels*/
                cmd_semaphore.semaphore(LOCK);
                carlos.send(cmd_semaphore);

                if (VERBOSE)
                {
                    std::cout << "LOCKED -> CMD" << std::endl;
                }
            }
        };

        // auto command = [VERBOSE](cluon::data::Envelope &&envelope) {
        //     /** unpack message recieved*/
        //     auto msg = cluon::extractMessage<carlos::command>(std::move(envelope));
        //     /*store turn value from acc microservice*/
        //     int16_t sign_type = msg.type();
        //     /*do stuff with date*/

        //     if (VERBOSE)
        //     {
        //         std::cout << "RECIVED -> wheel set to [" << turn << "]" << std::endl;
        //     }
        // };

        // auto car_detection = [VERBOSE](cluon::data::Envelope &&envelope) {
        //     /** unpack message recieved*/
        //     auto msg = cluon::extractMessage<carlos::vision::car>(std::move(envelope));
        //     /*store data*/
        //     float center_of_car = msg.coc();
        //     float area_of_car = msg.area();
        //     /*do stuff with data*/

        //     if (VERBOSE)
        //     {
        //         std::cout << "RECIVED -> coc (wheel): [" << center_of_car << "], area of car: [" << area_of_car << "]" << std::endl;
        //     }
        // };

        // auto sign_detection = [VERBOSE](cluon::data::Envelope &&envelope) {
        //     /** unpack message recieved*/
        //     auto msg = cluon::extractMessage<carlos::vision::sign>(std::move(envelope));
        //     /*store data*/
        //     float center_of_sign = msg.cos();
        //     float area_of_sign = msg.area();
        //     int16_t sign_type = msg.type();
        //     /*do stuff with data*/

        //     if (VERBOSE)
        //     {
        //         std::cout << "RECIVED -> cos: [" << center_of_sign << "], area of sign: [" << area_of_sign << "]" << std::endl;
        //     }
        // };

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::acc::ID(), adaptive_cruise_control);
        // carlos_session.dataTrigger(carlos::command::ID(), command);
        // carlos_session.dataTrigger(carlos::vision::car::ID(), car_detection);
        // carlos_session.dataTrigger(carlos::vision::sign::ID(), sign_detection);

        while (carlos_session.isRunning())
        {
            /* just run this microservice until ist crashes */
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
