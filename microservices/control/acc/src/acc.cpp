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
        std::cerr << "Usage:" << argv[0] << "[--cid=<int>] CID of your OD4Session" << std::endl;
        std::cerr << argv[0] << "[--carlos=<int>] ID of carlos microservices" << std::endl;
        std::cerr << argv[0] << "[--sd=<float>] safety distance for vehicle" << std::endl;
        std::cerr << argv[0] << "[--sp=<float, min is 0.13 and max is 0.8>]" << std::endl;
        std::cerr << argv[0] << "[--trig=<float>] tirgger distance for vehicle at intersection" << std::endl;
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
        auto get_status = [&SEMAPHORE, &STAGE](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::status>(std::move(envelope));
            /*store data*/
            SEMAPHORE = msg.semaphore();
            STAGE = msg.stage();
        };

        opendlv::proxy::PedalPositionRequest pedal; //kiwi
        int16_t count = 0;
        float sensor_total = 0;
        auto get_sensor_information = [VERBOSE, SAFE_DISTANCE, USER_SPEED, INTERSECTION, SEMAPHORE, &carlos_session, &SPEED, &PREV_SPEED, &STAGE, &pedal, &count, &sensor_total, &kiwi_session](cluon::data::Envelope &&envelope) {
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
                    if (count == 4)
                    {
                        count = 0;
                        float sensor_average = sensor_total / 5;
                        if (sensor_average < SAFE_DISTANCE)
                        {
                            SPEED = 0;
                            pedal.position(SPEED);

                            if (VERBOSE)
                            {
                                std::cout << "STAGE(" + std::to_string(STAGE) + ")->SEM(" + std::to_string(SEMAPHORE) + "): Object Detected at [" << sensor_average << "]" << std::endl;
                            }
                        }
                        else
                        {
                            SPEED = USER_SPEED;
                            pedal.position(SPEED);

                            if (VERBOSE)
                            {
                                std::cout << "STAGE(" + std::to_string(STAGE) + ")->SEM(" + std::to_string(SEMAPHORE) + "): Sent move instructions at speed [" << SPEED << "]" << std::endl;
                            }
                        }
                        if (SEMAPHORE && (SPEED != PREV_SPEED))
                        {
                            kiwi_session.send(pedal);
                        }

                        collision_status.collision_warning((sensor_average < SAFE_DISTANCE) ? true : false);
                        carlos_session.send(collision_status);
                        PREV_SPEED = SPEED;
                    }
                    else
                    {
                        sensor_total = sensor_total + sensor;
                        count = count + 1;
                    }
                }
                else
                {
                    //intersection
                    if (sensor < INTERSECTION)
                    {
                        trigger.front_sensor(true);

                        if (VERBOSE)
                        {
                            std::cout << "STAGE(" + std::to_string(STAGE) + ")->SEM(" + std::to_string(SEMAPHORE) + "): Front Sensor[True]" << std::endl;
                        }
                    }
                    else
                    {
                        trigger.front_sensor(false);

                        if (VERBOSE)
                        {
                            std::cout << "STAGE(" + std::to_string(STAGE) + ")->SEM(" + std::to_string(SEMAPHORE) + "): Front Sensor[False]" << std::endl;
                        }
                    }
                    carlos_session.send(trigger);
                }
            }

            if ((senderStamp == left_sensor) && STAGE == 2)
            {
                //intersection
                if (sensor < INTERSECTION)
                {
                    trigger.left_sensor(true);

                    if (VERBOSE)
                    {
                        std::cout << "STAGE(" + std::to_string(STAGE) + ")->SEM(" + std::to_string(SEMAPHORE) + "): Left Sensor[True]" << std::endl;
                    }
                }
                else
                {
                    trigger.left_sensor(false);

                    if (VERBOSE)
                    {
                        std::cout << "STAGE(" + std::to_string(STAGE) + ")->SEM(" + std::to_string(SEMAPHORE) + "): Left Sensor[False]" << std::endl;
                    }
                }
                carlos_session.send(trigger);
            }
        };

        /*registers callbacks*/
        carlos_session.dataTrigger(carlos::status::ID(), get_status);
        kiwi_session.dataTrigger(opendlv::proxy::DistanceReading::ID(), get_sensor_information);

        while (kiwi_session.isRunning())
        {
            //keep running
        }
    }
    else
    {
        std::cout << "Carlos Out. (OD4Session timed out.)" << std::endl;
    }
    return 0;
}
