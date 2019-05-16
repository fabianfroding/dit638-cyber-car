/**
 * Copyright (C) 2019 Carlos, the car
*/

#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>

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
        std::cerr << argv[0] << " is an management tool for Carlos micro-services." << std::endl;
        std::cerr << "Usage:" << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--verbose] see EVERYTHING" << std::endl;
        std::cerr << argv[0] << "[--acc] filter the acc messages" << std::endl;
        std::cerr << argv[0] << "[--cmd] filter the acc messages" << std::endl;
        std::cerr << argv[0] << "[--color] filter the acc messages" << std::endl;
        std::cerr << argv[0] << "[--object] filter the acc messages" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--verbose --object" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const bool ACC{commandlineArguments.count("acc") != 0};
    // const bool CMD{commandlineArguments.count("cmd") != 0};
    const bool COLOR{commandlineArguments.count("color") != 0};
    const bool OBJECT{commandlineArguments.count("object") != 0};

    std::cout << "starting up " << argv[0] << "..." << std::endl;

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session

    if (carlos_session.isRunning())
    {
        std::cout << "[delegator] micro-service started..." << std::endl;

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
        auto acc_collision = [VERBOSE, ACC, STAGE, &carlos_session, &LOCK, &UNLOCK, &services, &collision_warning](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::collision>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            collision_warning = msg.collision_warning();

            if (collision_warning)
            {
                services.acc.semaphore(LOCK);
                services.cmd.semaphore(LOCK);
                carlos_session.send(services.acc);
                carlos_session.send(services.object);
            }
            else
            {
                services.acc.semaphore(UNLOCK);
                services.cmd.semaphore(UNLOCK);
                carlos_session.send(services.acc);
                carlos_session.send(services.object);
            }

            if (VERBOSE || ACC)
            {
                std::cout << "inbox->acc(" + std::to_string(STAGE) + ")[   warning=" + std::to_string(collision_warning) + "]" << std::endl;
            }
        };

        bool front_trigger = false, left_trigger = false; //acc service (triggers)
        auto acc_trigger = [VERBOSE, ACC, STAGE, &front_trigger, &left_trigger](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::trigger>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            front_trigger = msg.front_sensor();
            left_trigger = msg.left_sensor();

            if (VERBOSE || ACC)
            {
                std::cout << "inbox->acc(" + std::to_string(STAGE) + ")[ left trigger=" + std::to_string(left_trigger) + "," + "right trigger=" + std::to_string(front_trigger) + "]" << std::endl;
            }
        };

        bool sign_detected = false, sign_reached = false; //object service
        auto object_sign = [VERBOSE, OBJECT, &STAGE, &carlos_session, &services, &LOCK, &sign_detected, &sign_reached](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::object::sign>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            sign_detected = msg.detected();
            sign_reached = msg.reached();

            /*STAGE LOGIC*/
            if (!(sign_detected == false) && !(sign_reached == false))
            {
                if (sign_detected && sign_reached == false)
                {
                    STAGE = 1;
                    services.acc.stage(STAGE);
                    services.cmd.stage(STAGE);
                    services.color.stage(STAGE);
                    services.object.stage(STAGE);

                    //send messages
                    carlos_session.send(services.acc);
                    carlos_session.send(services.cmd);
                    carlos_session.send(services.color);
                    carlos_session.send(services.object);
                }
                if (sign_reached && sign_detected == false)
                {
                    STAGE = 2;

                    services.acc.stage(STAGE);
                    services.acc.semaphore(LOCK);
                    services.cmd.stage(STAGE);
                    services.color.stage(STAGE);
                    services.object.stage(STAGE);

                    //send messages
                    carlos_session.send(services.acc);
                    carlos_session.send(services.cmd);
                    carlos_session.send(services.color);
                    carlos_session.send(services.object);
                }
            }

            if (VERBOSE || OBJECT)
            {
                std::cout << "inbox->object(" + std::to_string(STAGE) + ")[   detected=" + std::to_string(sign_detected) + ",   reached=" + std::to_string(sign_reached) + "]" << std::endl;
            }
        };

        float lead_car_coc = -1.0, lead_car_area = -1.0; //color service
        auto color_lead_car = [&lead_car_coc, &lead_car_area](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::color::lead_car>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            lead_car_coc = msg.coc();
            lead_car_area = msg.area();
        };

        bool north = false, east = false, west = false; //color service
        auto color_intersection = [VERBOSE, COLOR, &STAGE, &carlos_session, &services, &UNLOCK, &north, &east, &west](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::color::intersection>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            north = msg.north();
            east = msg.east();
            west = msg.west();

            if ((north == false && east == false && west == false) && STAGE == 2)
            {
                /*if north, east and west flags are false*/
                STAGE = 3;
                services.acc.stage(STAGE);
                services.acc.semaphore(UNLOCK);
                services.cmd.stage(STAGE);
                services.color.stage(STAGE);
                services.object.stage(STAGE);

                //send messages
                carlos_session.send(services.acc);
                carlos_session.send(services.cmd);
                carlos_session.send(services.color);
                carlos_session.send(services.object);
            }

            if (VERBOSE || COLOR)
            {
                std::cout << "inbox->color(" + std::to_string(STAGE) + ")[   west=" + std::to_string(west) + ",   north=" + std::to_string(north) + ",   east=" + std::to_string(east) + "]" << std::endl;
            }
        };

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::acc::collision::ID(), acc_collision);
        carlos_session.dataTrigger(carlos::acc::trigger::ID(), acc_trigger);
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
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
