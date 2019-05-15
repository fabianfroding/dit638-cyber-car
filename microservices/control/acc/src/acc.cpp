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
#include "envelopes.hpp"

/*main*/
int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if (0 != commandlineArguments.count("help"))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:" << argv[0] << " --cid=<CID of your OD4Session>" << std::endl;
        std::cerr << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--sd=<front/back safety space>]" << std::endl;
        std::cerr << argv[0] << "[--sp=<speed, min is 0.13 and max is 0.8>]" << std::endl;
        std::cerr << argv[0] << "[--trig=<trigger distance>]" << std::endl;
        std::cerr << argv[0] << "[--verbose]" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --carlos=113 --verbose --sd=0.2 --sp=013" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t CID{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const float SAFE_DISTANCE{(commandlineArguments.count("sd") != 0) ? static_cast<float>(std::stof(commandlineArguments["sd"])) : static_cast<float>(0.30)};
    const float USER_SPEED{(commandlineArguments.count("sp") != 0) ? static_cast<float>(std::stof(commandlineArguments["sp"])) : static_cast<float>(0.15)};
    const float INTERSECTION{(commandlineArguments.count("trig") != 0) ? static_cast<float>(std::stof(commandlineArguments["trig"])) : static_cast<float>(0.15)};

    std::cout << "starting up " << argv[0] << "..." << std::endl;
    std::cout << "speed: [" << USER_SPEED << "], front saftey distance: [" << SAFE_DISTANCE << " meters]" << std::endl;

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session
    cluon::OD4Session kiwi_session{CID};              //needed to recieve data from sensors

    if (kiwi_session.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "[acc] micro-service started..." << std::endl;
        }

        bool SEMAPHORE = true;
        int16_t STAGE = 0;
        float SPEED = 0, PREV_SPEED = SPEED;

        /*callbacks*/
        auto get_status = [VERBOSE, &SEMAPHORE, &STAGE, &kiwi_session](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::status>(std::move(envelope));
            /*store data*/
            SEMAPHORE = msg.semaphore();
            STAGE = msg.stage();

            if (!SEMAPHORE)
            {
                opendlv::proxy::PedalPositionRequest pedal; //kiwi
                pedal.position(0);
                kiwi_session.send(pedal);
            }

            if (VERBOSE)
            {
                if (SEMAPHORE)
                {
                    std::cout << "RECIEVED -> SEMAPHORE [UNLOCKED]" << std::endl;
                }
                else
                {
                    std::cout << "RECIEVED -> SEMAPHORE [LOCKED]" << std::endl;
                }
            }
        };

        opendlv::proxy::PedalPositionRequest pedal; //kiwi
        auto get_sensor_information = [VERBOSE, SAFE_DISTANCE, USER_SPEED, INTERSECTION, &carlos_session, &SPEED, &PREV_SPEED, &STAGE, &pedal](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            /*store sender id*/
            const uint16_t senderStamp = envelope.senderStamp(), front_sensor = 0, left_sensor = 1;
            /*store sensor data*/
            float sensor = msg.distance();

            carlos::acc::collision collision_status; //carlos
            carlos::acc::trigger trigger;            //carlos

            if (senderStamp == front_sensor)
            {
                if (STAGE != 2)
                {
                    if (sensor < SAFE_DISTANCE)
                    {
                        SPEED = 0;
                        pedal.position(SPEED);

                        if (VERBOSE)
                        {
                            std::cout << "Sent stop instructions. Object Detected at [" << sensor << "]" << std::endl;
                        }
                    }
                    else
                    {
                        SPEED = USER_SPEED;
                        pedal.position(SPEED);

                        if (VERBOSE)
                        {
                            std::cout << "Sent move instructions at speed [" << SPEED << "]" << std::endl;
                        }
                    }
                }
                else
                {
                    //intersection
                    if (sensor < INTERSECTION)
                    {
                        trigger.front_sensor(true);
                    }
                    else
                    {
                        trigger.front_sensor(false);
                    }
                    carlos_session.send(trigger);
                }
                collision_status.collision_warning((sensor < SAFE_DISTANCE) ? true : false);
                carlos_session.send(collision_status);
                PREV_SPEED = SPEED;
            }

            if ((senderStamp == left_sensor) && STAGE == 2)
            {
                //intersection
                if (sensor < INTERSECTION)
                {
                    trigger.left_sensor(true);
                }
                else
                {
                    trigger.left_sensor(false);
                }
                carlos_session.send(trigger);
            }
        };

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::acc::status::ID(), get_status);
        kiwi_session.dataTrigger(opendlv::proxy::DistanceReading::ID(), get_sensor_information);

        while (kiwi_session.isRunning())
        {
            /* just run this microservice until ist crashes */
            if (SEMAPHORE && (SPEED != PREV_SPEED))
            {
                kiwi_session.send(pedal);
            }
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
