#include <iostream>
#include <thread>

#include "cluon-complete.hpp"
#include "messages.hpp"

int main()
{
    std::cout << "starting up sender..." << std::endl;
    // char sender_ip[] = "127.0.0.1";
    // int sender_port = 1234;
    int goup_id = 101;

    /**
     * create a sender object that will listen and send on an ip through a specific port
    */
    cluon::UDPSender sender(sender_ip, sender_port);

    /**
     * create a message to send. [*Imprtant* use the templates provided in the messages.odv file]
    */
    standardMessage msg;
    msg.text("earth to Carlos. I, repeat, earth to Carlos");

    /**
     * send message
    */
    sender.send(msg);
    return 0;
}