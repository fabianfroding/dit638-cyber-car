#include <iostream>
#include <thread>

#include "cluon-complete.hpp"
#include "messages.hpp"

int main()
{
    std::cout << "starting up sender..." << std::endl;
    // char reciever_ip[] = "127.0.0.1";
    // int reciever_port = 1234;
    int goup_id = 101;

    /**
     * create a reciever object that will listen on an ip though a specific port
    */
    cluon::UDPReceiver receiver(reciever_ip, reciever_port,
                                /**this "thing" will listen in for: 
                                 * 1. data 
                                 * 2. the sender of the data
                                 * 3. the timestamp for the data... 
                                 * and then print them out*/
                                [](std::string && data, std::string && sender, std::chrono::system_clock::time_point && ts) noexcept {
                                    const auto timestamp(std::chrono::system_clock::to_time_t(ts));
                                    std::cout << "Received " << data.size() << " bytes from " << sender << " at "
                                              << timestamp << "s"
                                              << ", containing '" << data
                                              << "'." << std::endl;
                                });
    return 0;
}