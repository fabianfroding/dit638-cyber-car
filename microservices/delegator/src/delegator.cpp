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

struct micro_services
{
    carlos::acc::status acc;
    carlos::cmd::status cmd;
    carlos::color::status color;
    carlos::object::status object;
};

int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if (0 != commandlineArguments.count("help"))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:" << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--verbose]" << std::endl;
        std::cerr << argv[0] << "[--acc]" << std::endl;
        std::cerr << argv[0] << "[--cmd]" << std::endl;
        std::cerr << argv[0] << "[--color]" << std::endl;
        std::cerr << argv[0] << "[--object]" << std::endl;
        //std::cerr << argv[0] << "[--freq" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--verbose --object" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const bool ACC{commandlineArguments.count("acc") != 0};
    const bool CMD{commandlineArguments.count("cmd") != 0};
    const bool COLOR{commandlineArguments.count("color") != 0};
    const bool OBJECT{commandlineArguments.count("object") != 0};
    //const float FREQ{(commandlineArguments.count("freq") != 0) ? static_cast<float>(std::stof(commandlineArguments["freq"])) : static_cast<float>(30)};

    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
    }

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session

    if (carlos_session.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "[delegator] micro-service started..." << std::endl;
        }

        const bool LOCK = false;
        const bool UNLOCK = true;
        int16_t STAGE = 0;

        /*micro-services*/
        micro_services services;
        services.acc.semaphore(UNLOCK);
        services.cmd.semaphore(UNLOCK);
        services.color.semaphore(UNLOCK);
        services.object.semaphore(UNLOCK);
        services.acc.stage(STAGE);
        services.cmd.stage(STAGE);
        services.color.stage(STAGE);
        services.object.stage(STAGE);

        bool collision_warning = true; //acc service (collision)
        auto acc_collision = [VERBOSE, ACC, &carlos_session, &LOCK, &UNLOCK, &services, &collision_warning](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::collision>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            collision_warning = msg.collision_warning();

            if (collision_warning)
            {
                services.acc.semaphore(LOCK);
                services.color.semaphore(LOCK);
                carlos_session.send(services.acc);
                carlos_session.send(services.object);
            }
            else
            {
                services.acc.semaphore(UNLOCK);
                services.color.semaphore(UNLOCK);
                carlos_session.send(services.acc);
                carlos_session.send(services.object);
            }

            if (VERBOSE || ACC)
            {
                std::cout << "INBOX -> ACC collision warning:[" << collision_warning << "]" << std::endl;
            }
        };

        bool front_trigger = false, left_trigger = false; //acc service (triggers)
        auto acc_trigger = [VERBOSE, ACC, /* &carlos_session,  &LOCK, &UNLOCK,*/ &front_trigger, &left_trigger](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::trigger>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            front_trigger = msg.front_sensor();
            left_trigger = msg.left_sensor();

            if (VERBOSE || ACC)
            {
                std::cout << "INBOX -> ACC intersection triggers. FRONT [" << front_trigger << "], LEFT [" << left_trigger << "]" << std::endl;
            }
        };

        int16_t path = -1; //cmd service (1 = left, 2 = right, 0 = straight)
        auto cmd_path = [VERBOSE, CMD, /* &carlos_session, &LOCK, &UNLOCK, */ &path](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::cmd::path>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            path = msg.turn();

            if (VERBOSE || CMD)
            {
                std::cout << "INBOX -> CMD intersection path [" << path << "]" << std::endl;
            }
        };

        bool sign_detected = false, sign_reached = false; //object service
        auto object_sign = [VERBOSE, OBJECT, &STAGE, /* &carlos_session, &LOCK, &UNLOCK, */ &sign_detected, &sign_reached](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::object::sign>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            sign_detected = msg.detected();
            sign_reached = msg.reached();

            /*STAGE LOGIC*/
            if (sign_detected)
            {
                STAGE = 1;
                if (VERBOSE)
                {
                    std::cout << " STAGE set to [" << STAGE << "]" << std::endl;
                }
            }
            if (sign_reached)
            {
                STAGE = 2;
                if (VERBOSE)
                {
                    std::cout << " STAGE set to [" << STAGE << "]" << std::endl;
                }
            }

            if (VERBOSE || OBJECT)
            {
                std::cout << "INBOX -> Object Detection. SIGN DETECTED [" << sign_detected << "], SIGN REACHED [" << sign_reached << "]" << std::endl;
            }
        };

        float lead_car_coc = -1.0, lead_car_area = -1.0; //color service
        auto color_lead_car = [VERBOSE, COLOR, /* &carlos_session, &LOCK, &UNLOCK, */ &lead_car_coc, &lead_car_area](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::color::lead_car>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            lead_car_coc = msg.coc();
            lead_car_area = msg.area();

            if (VERBOSE || COLOR)
            {
                std::cout << "INBOX -> COLOR Detection. Center Of Car [" << lead_car_coc << "], Area of Car [" << lead_car_area << "]" << std::endl;
            }
        };

        bool north = false, east = false, west = false; //color service
        auto color_intersection = [VERBOSE, COLOR, &STAGE, /* &carlos_session, &LOCK, &UNLOCK, */ &north, &east, &west](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::color::intersection>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            north = msg.north();
            east = msg.east();
            west = msg.west();

            if (!(north && east && west))
            {
                /*if north, east and west flags are false*/
                STAGE = 3;
                if (VERBOSE)
                {
                    std::cout << " STAGE set to [" << STAGE << "]" << std::endl;
                }
            }

            if (VERBOSE || COLOR)
            {
                std::cout << "INBOX -> COLOR Detection. North [" << north << "], EAST [" << east << "]"
                          << "], WEST [" << west << "]" << std::endl;
            }
        };

        /* auto micro_services_announcement{[VERBOSE, STAGE, &carlos_session, &services]() -> bool {
            services.acc.stage(STAGE);
            services.cmd.stage(STAGE);
            services.color.stage(STAGE);
            services.object.stage(STAGE);

            //send messages
            carlos_session.send(services.acc);
            carlos_session.send(services.cmd);
            carlos_session.send(services.color);
            carlos_session.send(services.object);

            if (VERBOSE)
            {
                std::cout << "Pinged all micro-services in this session" << std::endl;
            }
            return true;
        }}; */

        /* if (FREQ > 0)
        {
            carlos_session.timeTrigger(FREQ, micro_services_announcement);
        } */

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::acc::collision::ID(), acc_collision);
        carlos_session.dataTrigger(carlos::acc::trigger::ID(), acc_trigger);
        carlos_session.dataTrigger(carlos::cmd::path::ID(), cmd_path);
        carlos_session.dataTrigger(carlos::object::sign::ID(), object_sign);
        carlos_session.dataTrigger(carlos::color::lead_car::ID(), color_lead_car);
        carlos_session.dataTrigger(carlos::color::intersection::ID(), color_intersection);

        while (carlos_session.isRunning())
        {
            /* just run this microservice until ist crashes */
            //send messages
            carlos_session.send(services.acc);
            carlos_session.send(services.cmd);
            carlos_session.send(services.color);
            carlos_session.send(services.object);
            //delay
            std::chrono::milliseconds timer(1000); // or whatever
            std::this_thread::sleep_for(timer);

            if (VERBOSE)
            {
                std::cout << "Pinged all micro-services in this session" << std::endl;
            }
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
