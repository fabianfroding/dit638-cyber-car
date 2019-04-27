#include <iostream>
#include <thread>

#include "cluon-complete.hpp"
#include "messages.hpp"

int main()
{
    std::cout << "starting up sender..." << std::endl;
    // char sender_ip[] = "127.0.0.1";
    // int sender_port = 1234;
    int secret = 101;

    /**
     * create a od4session object that will allow all microservices
     * with the same secret number to send and recieve messages from
     * one another
    */
    cluon::OD4Session sender{static_cast<uint16_t>(secret)};

    if (sender.isRunning())
    {
        /** wait 5 secs*/
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(5s);

        /**
        * create a message to send. [*Imprtant* use the templates provided in the messages.odv file]
        */
        standardMessage msg;
        msg.text("earth to Carlos. I, repeat, earth to Carlos");

        /**
        * send message
        */
        sender.send(msg);
    }
    return 0;
}