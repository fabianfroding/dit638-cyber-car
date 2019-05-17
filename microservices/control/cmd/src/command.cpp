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

int32_t main(int32_t argc, char **argv)
{
    /**Parse the arguments from the command line*/
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if (0 != commandlineArguments.count("help"))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << argv[0] << "[--carlos=<ID of carlos microservices>]" << std::endl;
        std::cerr << argv[0] << "[--cid=<ID of KIWI session>]" << std::endl;
        std::cerr << argv[0] << "[--turn=<turn angle>]" << std::endl;
        std::cerr << argv[0] << "[--verbose] print information" << std::endl;
        std::cerr << argv[0] << "[--debug] configure turns" << std::endl;
        std::cerr << argv[0] << "[--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << "--cid=112 --carlos=113 --verbose" << std::endl;
        return -1;
    }
    const uint16_t CARLOS_SESSION{(commandlineArguments.count("carlos") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["carlos"])) : static_cast<uint16_t>(113)};
    const uint16_t CID_SESSION{(commandlineArguments.count("cid") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["cid"])) : static_cast<uint16_t>(112)};
    const float TURN{(commandlineArguments.count("turn") != 0) ? static_cast<float>(std::stof(commandlineArguments["turn"])) : static_cast<float>(0.2)};
    const float SP{(commandlineArguments.count("speed") != 0) ? static_cast<float>(std::stof(commandlineArguments["speed"])) : static_cast<float>(0.14)};
    const uint16_t DELAY{(commandlineArguments.count("delay") != 0) ? static_cast<uint16_t>(std::stof(commandlineArguments["delay"])) : static_cast<uint16_t>(1)};
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};

    uint16_t dir = 12;

    if (VERBOSE)
    {
        std::cout << "starting up " << argv[0] << "..." << std::endl;
        std::cout << "turn: [" << TURN << "]" << std::endl;
        std::cout << "speed: [" << SP << "]" << std::endl;
        std::cout << "delay: [" << DELAY << "]" << std::endl;
        std::cout << "turn: [" << TURN << "]" << std::endl;
    }

    cluon::OD4Session carlos_session{CARLOS_SESSION};
    cluon::OD4Session kiwi_session{CID_SESSION};

    if (kiwi_session.isRunning())
    {
        std::cout << "session started..." << std::endl;

        bool SEMAPHORE = true;
        int16_t STAGE = 0;
        auto get_status = [&SEMAPHORE, &STAGE](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::status>(std::move(envelope));
            /*store data*/
            SEMAPHORE = msg.semaphore();
            STAGE = msg.stage();
        };

        bool turn_west = true, turn_north = true, turn_east = true;
        auto turn_status = [VERBOSE, &turn_west, &turn_north, &turn_east](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<carlos::cmd::turn_status>(std::move(envelope));
            /*store data*/
            turn_west = msg.west_turn();
            turn_north = msg.north_turn();
            turn_east = msg.east_turn();

            if (VERBOSE)
            {
                std::cout << "session started..." << std::endl;
            }
        };

        carlos_session.dataTrigger(carlos::status::ID(), get_status);
        carlos_session.dataTrigger(carlos::cmd::turn_status::ID(), turn_status);

        opendlv::proxy::GroundSteeringRequest wheel; //[car] groundSteering
        opendlv::proxy::PedalPositionRequest pedal;  //[car] pedal position

        while (kiwi_session.isRunning())
        {
            std::cout << "STAGE(" + std::to_string(STAGE) + "): West turn: " + std::to_string(turn_west) + ", North turn: " + std::to_string(turn_north) + ",East turn: " + std::to_string(turn_east) << std::endl;

            if (STAGE == 3)
            {
                std::cout << "[STAGE 3 Engaged]: Direction (9, 12, 3):" << std::endl;
                scanf("%hd", &dir);

                switch (dir)
                {
                case 9:
                    if (turn_west)
                    {
                        std::cout << "carlos turning left" << std::endl;
                        //turn wheel
                        wheel.groundSteering(0.14);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(wheel);
                        }

                        //speed
                        pedal.position(0.13);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(pedal);
                        }

                        //delay
                        std::chrono::milliseconds timer(3); // or whatever
                        std::this_thread::sleep_for(timer);

                        //stop vehicle
                        pedal.position(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(pedal);
                        }

                        //straighten wheel
                        wheel.groundSteering(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(wheel);
                        }
                    }
                    else
                    {
                        std::cout << "you are not allowed to turn west" << std::endl;
                    }
                    break;
                case 12:
                    if (turn_north)
                    {
                        std::cout << "carlos turning right" << std::endl;
                        //turn wheel
                        wheel.groundSteering(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(wheel);
                        }

                        //speed
                        pedal.position(0.12);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(pedal);
                        }

                        //delay
                        std::chrono::milliseconds timer(3); // or whatever
                        std::this_thread::sleep_for(timer);

                        //stop vehicle
                        pedal.position(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(pedal);
                        }

                        //straighten wheel
                        wheel.groundSteering(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(wheel);
                        }
                    }
                    else
                    {
                        std::cout << "you are not allowed to turn north" << std::endl;
                    }
                    break;

                case 3:
                    if (turn_east)
                    {
                        std::cout << "carlos turning right" << std::endl;
                        //turn wheel
                        wheel.groundSteering(-0.25);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(wheel);
                        }

                        //speed
                        pedal.position(0.13);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(pedal);
                        }

                        //delay
                        std::chrono::milliseconds timer(3); // or whatever
                        std::this_thread::sleep_for(timer);

                        //stop vehicle
                        pedal.position(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(pedal);
                        }

                        //straighten wheel
                        wheel.groundSteering(0);
                        if (SEMAPHORE)
                        {
                            kiwi_session.send(wheel);
                        }
                    }
                    else
                    {
                        std::cout << "you are not allowed to turn east" << std::endl;
                    }
                    break;
                }
            }
            else
            {
            }
        }
    }
    else
    {
        std::cout << "Carlos Out. (Sarlos Session timed out.)" << std::endl;
    }
    return 0;
}
