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

float autoPedal(float sensor, float safetyDistance, float maxSpeed, float currentSpeed, float neutral, bool VERBOSE);

int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ((0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] << " --cid=<CID of your OD4Session>" << std::endl;
        std::cerr << argv[0] << "[--fb=<front/back safety space>]" << std::endl;
        std::cerr << argv[0] << "[--lr=<left/right safety space>]" << std::endl;
        std::cerr << argv[0] << "[--sp=<speed, min is 0.13 and max is 0.8>]" << std::endl;
        std::cerr << argv[0] << "[--tr=< turn, min is 0.00 and max is 0.5>]" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --fb=0.2 --lr=0.2 --sp=013 --tr=0.2" << std::endl;
        return -1;
    }
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const float FB{(commandlineArguments.count("fb") != 0) ? static_cast<float>(std::stof(commandlineArguments["fb"])) : static_cast<float>(0.20)};
    const float MAXSPEED{(commandlineArguments.count("sp") != 0) ? static_cast<float>(std::stof(commandlineArguments["sp"])) : static_cast<float>(0.70)};
    const float TURN{(commandlineArguments.count("tr") != 0) ? static_cast<float>(std::stof(commandlineArguments["tr"])) : static_cast<float>(0.50)};
    // const float LR{(commandlineArguments.count("lr") != 0) ? static_cast<float>(std::stof(commandlineArguments["lr"])) : static_cast<float>(0.20)};

    std::cout << "starting up " << argv[0] << "..." << std::endl;
    std::cout << "speed: [" << MAXSPEED << "], turn angle: [" << TURN << "], front saftey distance: [" << FB << "]" << std::endl;
    std::cout << "oli" << std::endl;
    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session acc{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
    if (acc.isRunning())
    {
        if (VERBOSE)
        {
            std::cout << "session started..." << std::endl;
        }

        const int16_t delay{500}; // milliseconds

        /*set up messages to send*/
        opendlv::proxy::PedalPositionRequest pedalReq; //pedalReq.position(xxx);
        const float neutral = 0.0;
        float currentSpeed = neutral;

        opendlv::proxy::GroundSteeringRequest steerReq; //steerReq.groundSteering(xxx);
        const float right = -1 * TURN;
        const float left = TURN;

        /*set up thread functions*/
        float dist = 0.0, frontSensor = 0.0;
        auto distanceTrigger = [VERBOSE, FB, MAXSPEED, &acc, &pedalReq, &dist, &frontSensor, &currentSpeed, &neutral](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            const uint16_t senderStamp = envelope.senderStamp();
            dist = msg.distance();
            if (senderStamp == 0)
            {
                frontSensor = dist;
                float speed = autoPedal(frontSensor, FB, MAXSPEED, currentSpeed, neutral, VERBOSE);
                if (VERBOSE)
                {
                    if (speed > neutral)
                    {
                        std::cout << "Moving..." << std::endl;
                    }
                    if (speed == neutral)
                    {
                        std::cout << "Object Detected at [" << frontSensor << "]" << std::endl;
                    }
                }

                pedalReq.position(speed);
                acc.send(pedalReq);
                std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            }
        };

        int16_t turn = 0;
        auto turnTrigger = [&acc, &steerReq, &turn, &neutral, &right, &left](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::command>(std::move(envelope));
            turn = msg.type();
            switch (turn)
            {
            case 0:
                steerReq.groundSteering(neutral);
                acc.send(steerReq);
                break;
            case 1:
                steerReq.groundSteering(right);
                acc.send(steerReq);
                break;
            case 2:
                steerReq.groundSteering(left);
                acc.send(steerReq);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        };

        auto followTrigger = [&acc, &steerReq, &left, &right, &neutral](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::vision::car>(std::move(envelope));
            float coc = msg.coc();

            if (0.0 <= coc && coc < 0.4)
            {
                //object is on the left
                steerReq.groundSteering(left);
                acc.send(steerReq);
            }
            if (0.4 <= coc && coc < 0.6)
            {
                steerReq.groundSteering(neutral);
                acc.send(steerReq);
            }
            if (0.6 <= coc && coc <= 1.0)
            {
                //object is on the right
                steerReq.groundSteering(right);
                acc.send(steerReq);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        };

        /*register threads functions*/
        acc.dataTrigger(opendlv::proxy::DistanceReading::ID(), distanceTrigger);
        acc.dataTrigger(carlos::command::ID(), turnTrigger);
        acc.dataTrigger(carlos::vision::car::ID(), followTrigger);

        /* wait 3 secs*/
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(3s);
        while (acc.isRunning())
        {
            // if (frontSensor <= FB)
            // {
            //     /**
            //     * front sensor detects something
            //     * stop car
            //     * */
            //     pedalReq.position(neutral);
            //     acc.send(pedalReq);
            //     currentSpeed = neutral;
            //     if (VERBOSE)
            //     {
            //         std::cout << "Object Detected at [" << frontSensor << "]" << std::endl;
            //     }
            // }

            // if (FB < frontSensor)
            // {
            //     /**
            //      * front sensor is clear
            //      * move car forward
            //      * */
            //     if (currentSpeed < MAXSPEED)
            //     {
            //         currentSpeed = currentSpeed + 0.05;
            //         pedalReq.position(currentSpeed);
            //         acc.send(pedalReq);
            //     }
            //     if (currentSpeed == MAXSPEED)
            //     {
            //         pedalReq.position(currentSpeed);
            //         acc.send(pedalReq);
            //     }
            //     if (VERBOSE)
            //     {
            //         std::cout << "Moving..." << std::endl;
            //     }
            // }
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}

float autoPedal(float frontSensor, float safetyDistance, float maxSpeed, float currentSpeed, float neutral, bool VERBOSE)
{
    if (VERBOSE)
    {
        std::cout << "autoPedal engaged.." << std::endl;
    }
    if (frontSensor <= safetyDistance)
    {
        /**
        * front sensor detects something
        * stop car
        * */
        currentSpeed = neutral;
    }

    if (frontSensor > safetyDistance)
    {
        /**
        * front sensor is clear
        * move car forward
        * */
        if (currentSpeed < maxSpeed)
        {
            currentSpeed = currentSpeed + 0.05;
        }
    }
    return currentSpeed;
}