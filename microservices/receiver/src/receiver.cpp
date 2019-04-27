#include <iostream>
#include <thread>
#include "cluon-complete.hpp"
#include "messages.hpp"

int main()
{
    std::cout << "starting up receiver..." << std::endl;
    // char receiver_ip[] = "127.0.0.1";
    // int receiver_port = 1234;
    int secret = 101;

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session receiver{static_cast<uint16_t>(secret)};

    if (receiver.isRunning())
    {
        /**
         * create a callback that will be triggered if a message of type
         * "standardMessage" (check the messages.odv file) is recived.
        */

        auto standardMessageTrigger = [](cluon::data::Envelope &&env) {
            /** unpack message recieved*/
            standardMessage msg = cluon::extractMessage<standardMessage>(std::move(env));
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