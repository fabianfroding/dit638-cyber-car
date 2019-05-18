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
        std::cerr << argv[0] << "[--cmd] filter the cmd messages" << std::endl;
        std::cerr << argv[0] << "[--color] filter the color detection messages" << std::endl;
        std::cerr << argv[0] << "[--sign] filter the object detection messages" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--verbose --sign" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const bool ACC{commandlineArguments.count("acc") != 0};
    const bool CMD{commandlineArguments.count("cmd") != 0};
    const bool COLOR{commandlineArguments.count("color") != 0};
    const bool SIGN{commandlineArguments.count("sign") != 0};

    std::cout << "starting up " << argv[0] << "..." << std::endl;

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session

    if (carlos_session.isRunning())
    {
        std::cout << "[delegator] micro-service started..." << std::endl;

        const bool LOCK = false;
        const bool UNLOCK = true;
        int16_t STAGE = 0;

        /*micro-services*/
        carlos::status services;
        services.semaphore(true);
        services.stage(0);

        bool collision_warning = true; //acc service (collision)
        auto acc_collision = [VERBOSE, ACC, STAGE, &carlos_session, &LOCK, &UNLOCK, &services, &collision_warning](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::collision>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            collision_warning = msg.collision_warning();

            if (collision_warning)
            {
                services.semaphore(LOCK);
                carlos_session.send(services);
            }
            else
            {
                services.semaphore(UNLOCK);
                carlos_session.send(services);
            }

            if (VERBOSE || ACC)
            {
                std::cout << "inbox->acc(" + std::to_string(STAGE) + ")[warning=" + std::to_string(collision_warning) + "]" << std::endl;
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

        bool sign_detected = false, sign_reached = false; //object service
        auto object_sign = [VERBOSE, SIGN, &STAGE, &carlos_session, &services, &LOCK, &sign_detected, &sign_reached](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::object::sign>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            sign_detected = msg.detected();
            sign_reached = msg.reached();

            /*STAGE LOGIC*/
            if (sign_detected && (sign_reached == false))
            {
                STAGE = 1;
                services.stage(STAGE);

                //send messages
                carlos_session.send(services);
            }
            if (sign_reached && (sign_detected == false))
            {
                STAGE = 2;

                services.stage(STAGE);
                services.semaphore(LOCK);

                //send messages
                carlos_session.send(services);
            }

            if (VERBOSE || SIGN)
            {
                std::cout << "inbox->sign(" + std::to_string(STAGE) + ")[detected=(" + std::to_string(sign_detected) + "),reached=(" + std::to_string(sign_reached) + ")]" << std::endl;
            }
        };

        bool front_trigger = false, left_trigger = false; //acc service (triggers)
        bool north = false, east = false, west = false;   //color service
        auto acc_trigger = [VERBOSE, ACC, STAGE, &front_trigger, &left_trigger, north, east, &west](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::trigger>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            front_trigger = msg.front_sensor();
            left_trigger = msg.left_sensor();

            if (north == true && east == true)
            {
                if ((front_trigger || left_trigger))
                {
                    west = false;
                }
            }

            if (north == true && east == false)
            {
                if ((front_trigger || left_trigger))
                {
                    west = false;
                }
            }
            if (east == true && north == false)
            {
                if ((front_trigger || left_trigger))
                {
                    west = false;
                }
            }

            if (VERBOSE || ACC)
            {
                std::cout << "inbox->acc(" + std::to_string(STAGE) + ")[left trigger=(" + std::to_string(left_trigger) + ")," + "right trigger=(" + std::to_string(front_trigger) + ")]" << std::endl;
            }
        };

        auto color_intersection = [VERBOSE, COLOR, &STAGE, &north, &east, &west](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::color::intersection>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            if (STAGE == 1)
            {
                north = msg.north();
                east = msg.east();
                west = msg.west();
            }

            if (STAGE == 2)
            {
                north = msg.north();
                east = msg.east();
            }

            if ((west == false && north == false && east == false) && STAGE == 2)
            {
                STAGE = 3;
            }

            if (VERBOSE || COLOR)
            {
                std::cout << "inbox->color(" + std::to_string(STAGE) + ")[west=(" + std::to_string(west) + "), north=(" + std::to_string(north) + "), east=(" + std::to_string(east) + ")]" << std::endl;
            }
        };

        auto turn_status = [VERBOSE, CMD, &STAGE](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::cmd::turn_status>(std::move(envelope));
            /*store speed and front_sensor value from acc microservice*/
            bool command_turn_status = msg.complete();

            if (command_turn_status == true)
            {
                STAGE = 0;
            }
            if (VERBOSE || CMD)
            {
                std::cout << "inbox->command(" + std::to_string(STAGE) + ")[Intersetion Complete]" << std::endl;
            }
        };

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::acc::collision::ID(), acc_collision);
        carlos_session.dataTrigger(carlos::acc::trigger::ID(), acc_trigger);
        carlos_session.dataTrigger(carlos::object::sign::ID(), object_sign);
        carlos_session.dataTrigger(carlos::color::lead_car::ID(), color_lead_car);
        carlos_session.dataTrigger(carlos::color::intersection::ID(), color_intersection);
        carlos_session.dataTrigger(carlos::cmd::turn_status::ID(), turn_status);

        while (carlos_session.isRunning())
        {
            /* just run this microservice until ist crashes */
            //send messages
            carlos_session.send(services);
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
