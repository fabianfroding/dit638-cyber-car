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
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--freq=<Frequency>] [--verbose] [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --freq=30 --verbose" << std::endl;
        return -1;
    }

    std::cout << "starting up receiver..." << std::endl;
    int secret = static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]));

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session receiver{secret};

    if (receiver.isRunning())
    {
        std::cout << "starting up receiver..." << std::endl;

        /**
         * create a callbacks that will be triggered if a message of type
         * "xxx" (check the messages.odv file) is recived.
        */

        auto standardMessageTrigger = [](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<standardMessage>(std::move(env));
            std::cout << "Text = " << msg.text() << std::endl;
        };

        int frontSensor = -100;
        auto frontSensorTrigger = [](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
            std::cout << "Text = " << msg.text() << std::endl;
        };

        int backSensor = -100;
        auto backSensorTrigger = [](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
            std::cout << "Text = " << msg.text() << std::endl;
        };

        int rightSensor = -100;
        auto rightSensorTrigger = [](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
            std::cout << "Text = " << msg.text() << std::endl;
        };

        int leftSensor = -100;
        auto leftSensorTrigger = [](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(env));
            std::cout << "Text = " << msg.text() << std::endl;
        };

        /**register callback to od4 session*/
        receiver.dataTrigger(standardMessage::ID(), standardMessageTrigger);

        /** wait 10 secs*/
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(10s);
    }
    return 0;
}