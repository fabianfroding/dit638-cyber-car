#include <iostream>
#include <thread>
#include "cluon-complete.hpp"
#include "messages.hpp"

int32_t main(int32_t argc, char **argv)
{
    // Parse the arguments from the command line
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ((0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")))
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--fb=<front/back padding>] [--lb=<left/right padding] [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --fb=0.2 --lb=0.2" << std::endl;
        return -1;
    }

    std::cout << "starting up " << argv[0] << "..." << std::endl;
    const int32_t secret = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));
    const bool VERBOSE{commandlineArguments.count("verbose") != 0};
    const int32_t FB{(commandlineArguments.count("fb") != 0) ? static_cast<float>(std::stof(commandlineArguments["fb"])) : static_cast<float>(0.20)};
    const int32_t LR{(commandlineArguments.count("lr") != 0) ? static_cast<float>(std::stof(commandlineArguments["lr"])) : static_cast<float>(0.20)};
    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session service{secret};

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

        float frontSensor{-100.0};
        auto frontSensorTrigger = [VERBOSE, &frontSensor](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            const uint16_t senderStamp = envelope.senderStamp();
            frontSensor = msg.distance();

            if (VERBOSE && senderStamp == 0)
            {
                std::cout << "Front Sensor: " << frontSensor << std::endl;
            }
        };

        float backSensor{-100.0};
        auto backSensorTrigger = [VERBOSE, &backSensor](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            const uint16_t senderStamp = envelope.senderStamp();
            backSensor = msg.distance();

            if (VERBOSE && senderStamp == 2)
            {
                std::cout << "Back Sensor: " << backSensor << std::endl;
            }
        };

        float rightSensor{-100.0};
        auto rightSensorTrigger = [VERBOSE, &rightSensor](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            const uint16_t senderStamp = envelope.senderStamp();
            rightSensor = msg.distance();

            if (VERBOSE && senderStamp == 3)
            {
                std::cout << "Right Sensor: " << rightSensor << std::endl;
            }
        };

        float leftSensor{-100.0};
        auto leftSensorTrigger = [VERBOSE, &leftSensor](cluon::data::Envelope &&envelope) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            const uint16_t senderStamp = envelope.senderStamp();
            leftSensor = msg.distance();

            if (VERBOSE && senderStamp == 1)
            {
                std::cout << "Left Sensor: " << leftSensor << std::endl;
            }
        };

        /**register callback to od4 session*/
        service.dataTrigger(standardMessage::ID(), standardMessageTrigger);
        service.dataTrigger(opendlv::proxy::DistanceReading::ID(), frontSensorTrigger);
        service.dataTrigger(opendlv::proxy::DistanceReading::ID(), backSensorTrigger);
        service.dataTrigger(opendlv::proxy::DistanceReading::ID(), rightSensorTrigger);
        service.dataTrigger(opendlv::proxy::DistanceReading::ID(), leftSensorTrigger);

        /**
         * set up messages that you might send
        */

        opendlv::proxy::PedalPositionRequest pedalReq;  //pedalReq.position(xxx);
        opendlv::proxy::GroundSteeringRequest steerReq; //steerReq.groundSteering(xxx);
        float right{-0.67};
        float left{0.67};
        float forward{0.50};
        float backward{-0.80};
        float neutral{0};

        /** wait x secs before doing anything*/
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(5s);

        while (service.isRunning())
        {
            if (frontSensor < FB || backSensor < FB)
            {
                pedalReq.position(forward); //move car forward
                service.send(pedalReq);
                std::this_thread::sleep_for(1s);
            }
            else
            {
                pedalReq.position(neutral); //stop car
                service.send(pedalReq);
                std::this_thread::sleep_for(1s);
            }

            if (leftSensor < LR)
            {
                steerReq.groundSteering(left);
                service.send(steerReq);
                std::this_thread::sleep_for(1s);
            }
            else if (rightSensor < LR)
            {
                steerReq.groundSteering(right);
                service.send(steerReq);
                std::this_thread::sleep_for(1s);
            }
            else
            {
                steerReq.groundSteering(neutral);
                service.send(steerReq);
                std::this_thread::sleep_for(1s);
            }
        }
    }
    else
    {
        std::cout << "OD4Session timed out. Carlos." << std::endl;
    }
    return 0;
}