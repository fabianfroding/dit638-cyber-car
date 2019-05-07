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
        std::cerr << "example:  " << argv[0] << " --cid=112 --sd=0.2 --sp=013" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t CID{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const bool EXTRA_VERBOSE{commandlineArguments.count("extra_v") != 0};
    const float SAFE_DISTANCE{(commandlineArguments.count("sd") != 0) ? static_cast<float>(std::stof(commandlineArguments["sd"])) : static_cast<float>(0.30)};
    const float MAX_SPEED{(commandlineArguments.count("sp") != 0) ? static_cast<float>(std::stof(commandlineArguments["sp"])) : static_cast<float>(0.15)};

    if (VERBOSE || EXTRA_VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
        std::cout << "speed: [" << MAX_SPEED << "], front saftey distance: [" << SAFE_DISTANCE << " meters]" << std::endl;
    }

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */

    cluon::OD4Session carlos_session{CARLOS_SESSION}; //needed to send messages to carlos session
    cluon::OD4Session car_session{CID};               //needed to recieve data from sensors

    if (car_session.isRunning())
    {
        if (VERBOSE || EXTRA_VERBOSE)
        {
            std::cout << "[acc] micro-service started..." << std::endl;
        }

        /*set up messages to send*/
        carlos::acc acc; //send to carlos session
        float frontSensor = 0.0;

        /*sends front sensor messages to carlos delegator*/
        auto distanceTrigger = [EXTRA_VERBOSE, VERBOSE, SAFE_DISTANCE, MAX_SPEED, &carlos_session, &acc, &frontSensor](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            /*store sender id*/
            const uint16_t senderStamp = envelope.senderStamp();
            /*store sender data*/
            float distance = msg.distance();

            if (senderStamp == 0)
            {
                /**
                 * only get data fron sender 0 (front sensor)
                 * */

                /*get front sensor value*/
                frontSensor = distance;
                /*calculate speed*/
                float speed = autoPedal(frontSensor, SAFE_DISTANCE, MAX_SPEED, VERBOSE);
                /*store speed in carlos object*/
                acc.speed(speed);
                acc.front_sensor_is_safe((frontSensor < SAFE_DISTANCE) ? true : false); //if front sensor is safe, set to true, else set to false
                /*send object to carlos delegator*/
                carlos_session.send(acc);
            }

            if (EXTRA_VERBOSE)
            {
                std::cout << "autoPedal engaged.." << std::endl;
            }
        };

        /*registers callbacks*/
        car_session.dataTrigger(opendlv::proxy::DistanceReading::ID(), distanceTrigger);

        while (car_session.isRunning())
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
