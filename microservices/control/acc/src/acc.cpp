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
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--fb=<front/back safety space>] [--lr=<left/right safety space>] [--sp=<speed>] [--tr=<turn>]  [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --fb=0.2 --lr=0.2" << std::endl;
        return -1;
    }
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const float FB{(commandlineArguments.count("fb") != 0) ? static_cast<float>(std::stof(commandlineArguments["fb"])) : static_cast<float>(0.20)};
    const float MAXSPEED{(commandlineArguments.count("sp") != 0) ? static_cast<float>(std::stof(commandlineArguments["sp"])) : static_cast<float>(0.70)};
    // const float LR{(commandlineArguments.count("lr") != 0) ? static_cast<float>(std::stof(commandlineArguments["lr"])) : static_cast<float>(0.20)};
    // const float TURN{(commandlineArguments.count("tr") != 0) ? static_cast<float>(std::stof(commandlineArguments["lr"])) : static_cast<float>(0.50)};

    std::cout << "starting up " << argv[0] << "..." << std::endl;
    std::cout << "speed: " << MAXSPEED << " saftey distance: " << FB << std::endl;

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session service{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    if (service.isRunning())
    {
        std::cout << "session started..." << std::endl;

        /**
         * create a callbacks that will be triggered if a message of type
         * "xxx" (check the messages.odv file) is recived.
        */
        auto standardMessageTrigger = [VERBOSE](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<standardMessage>(std::move(env));
            if (VERBOSE)
            {
                std::cout << "Text = " << msg.text() << std::endl;
            }
        };

        float dist{-1000.0}, frontSensor{-1000.0} /* , backSensor{-1000.0}, rightSensor{-1000.0}, leftSensor{-1000.0} */;
        auto distanceTrigger = [VERBOSE, &dist, &frontSensor /* , &backSensor, &rightSensor, &leftSensor */](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            const uint16_t senderStamp = envelope.senderStamp();
            dist = msg.distance();

            if (senderStamp == 0)
            {
                frontSensor = dist;
                if (VERBOSE)
                {
                    std::cout << "[Front Sensor: " << frontSensor << "]" << std::endl;
                }
            }
        };

        /**register callback to od4 session*/
        service.dataTrigger(standardMessage::ID(), standardMessageTrigger);
        service.dataTrigger(opendlv::proxy::DistanceReading::ID(), distanceTrigger);

        /**
        * set up messages that you might send
        */
        opendlv::proxy::PedalPositionRequest pedalReq; //pedalReq.position(xxx);
        const float neutral = 0.0;
        float currentSpeed = neutral;

        // opendlv::proxy::GroundSteeringRequest steerReq; //steerReq.groundSteering(xxx);
        // float right{-1 * TURN};
        // float left{TURN};

        /** wait x secs before doing anything*/
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(5s);
        const int16_t delay{1000}; // milliseconds
        while (service.isRunning())
        {
            if (frontSensor > FB)
            {
                /** 
                 * front sensor is clear
                 * move car forward 
                 * */
                if (currentSpeed < MAXSPEED)
                {
                    currentSpeed = currentSpeed + 0.05;
                    pedalReq.position(currentSpeed);
                    service.send(pedalReq);
                    std::cout << "Current Speed is: " << currentSpeed << std::endl;
                }
                else
                {
                    pedalReq.position(currentSpeed);
                    service.send(pedalReq);
                    std::cout << "Reached max speed: " << currentSpeed << std::endl;
                }
            }

            if (frontSensor < FB)
            {
                /** 
                 * front sensor detects something
                 * stop car 
                 * */
                pedalReq.position(neutral);
                service.send(pedalReq);
                currentSpeed = neutral;
                std::cout << "Detected object: " << frontSensor << std::endl;
                std::cout << "stopping.." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            }

            // if (leftSensor > LR)
            // {
            //     steerReq.groundSteering(left);
            //     service.send(steerReq);
            //     std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            // }

            // if (rightSensor > LR)
            // {
            //     steerReq.groundSteering(right);
            //     service.send(steerReq);
            //     std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            // }

            // if (rightSensor < LR && leftSensor < LR)
            // {
            //     steerReq.groundSteering(neutral);
            //     service.send(steerReq);
            //     std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            // }
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}