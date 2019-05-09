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

float autoPedal(float front_sensor, float SAFE_DISTANCE, float MAX_SPEED, bool VERBOSE);

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
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --carlos=113 --sd=0.2 --sp=013" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t CID{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const float SAFE_DISTANCE{(commandlineArguments.count("sd") != 0) ? static_cast<float>(std::stof(commandlineArguments["sd"])) : static_cast<float>(0.30)};
    const float MAX_SPEED{(commandlineArguments.count("sp") != 0) ? static_cast<float>(std::stof(commandlineArguments["sp"])) : static_cast<float>(0.15)};

    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
        std::cout << "speed: [" << MAX_SPEED << "], front saftey distance: [" << SAFE_DISTANCE << " meters]" << std::endl;
    }

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

        /*callbacks*/
        auto get_status = [VERBOSE, &SEMAPHORE, &STAGE](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::acc::status>(std::move(envelope));
            /*store data*/
            SEMAPHORE = msg.semaphore();
            STAGE = msg.stage();

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

        auto get_sensor_information = [VERBOSE, SAFE_DISTANCE, MAX_SPEED, &kiwi_session, &carlos_session, &SEMAPHORE, &STAGE](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            /*store sender id*/
            const uint16_t senderStamp = envelope.senderStamp(), front_sensor = 0, left_sensor = 1;
            /*store sensor data*/
            float sensor = msg.distance();
            /*other variables*/
            float speed = autoPedal(sensor, SAFE_DISTANCE, MAX_SPEED, VERBOSE);
            /*messages to carlos session*/
            carlos::acc::collision collision_status;
            carlos::acc::trigger trigger;
            /*messages to kiwi session*/
            opendlv::proxy::PedalPositionRequest pedal;

            switch (senderStamp)
            {
            case front_sensor:
                /* front sensor */
                switch (STAGE)
                {
                case 1:
                    /* intersection detected */
                    pedal.position(speed);

                    collision_status.collision_warning((sensor < SAFE_DISTANCE) ? true : false);
                    carlos_session.send(collision_status);
                    break;

                case 2:
                    /* at intersection */
                    trigger.front_sensor(sensor);
                    carlos_session.send(trigger);
                    pedal.position(0); //car should never move in stage 2
                    break;

                case 3:
                    collision_status.collision_warning((sensor < SAFE_DISTANCE) ? true : false);
                    carlos_session.send(collision_status);
                    break;

                default:
                    /* no stages have been engaged */
                    pedal.position(speed);
                    collision_status.collision_warning((sensor < SAFE_DISTANCE) ? true : false);
                    carlos_session.send(collision_status);
                    break;
                }
                break;

            case left_sensor:
                if (STAGE == 2)
                { /* at intersection */
                    trigger.left_sensor(sensor);
                    carlos_session.send(trigger);
                    pedal.position(0); //car should never move in stage 2
                }
                break;
            }

            if (SEMAPHORE)
            {
                kiwi_session.send(pedal);
            }
        };

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::acc::status::ID(), get_status);
        kiwi_session.dataTrigger(opendlv::proxy::DistanceReading::ID(), get_sensor_information);

        while (kiwi_session.isRunning())
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

/*functions*/

float autoPedal(float front_sensor, float SAFE_DISTANCE, float MAX_SPEED, bool VERBOSE)
{
    const float neutral = 0.0;
    float result = neutral;

    if (front_sensor <= SAFE_DISTANCE)
    {
        /**
        * front sensor detects something
        * stop car
        * */
        result = neutral;

        if (VERBOSE)
        {
            std::cout << "Sent stop instructions. Object Detected at [" << front_sensor << "]" << std::endl;
        }
    }

    if (front_sensor > SAFE_DISTANCE)
    {
        /**
        * front sensor is clear
        * move car forward
        * */
        result = MAX_SPEED;

        if (VERBOSE)
        {
            std::cout << "Sent move instructions at speed [" << result << "]" << std::endl;
        }
    }
    return result;
}
