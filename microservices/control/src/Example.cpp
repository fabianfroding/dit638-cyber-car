/*
 * Copyright (C) 2019 Yue Kang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include "cluon-complete.hpp"
#include "messages.hpp"

int32_t main(int32_t argc, char **argv) {

    // Parse the arguments from the command line
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);

    if ( (0 == commandlineArguments.count("cid")) || (0 != commandlineArguments.count("help")) )
    {
        std::cerr << argv[0] << " is an example application for miniature vehicles (Kiwis) of DIT638 course." << std::endl;
        std::cerr << "Usage:  " << argv[0] << " --cid=<CID of your OD4Session> [--freq=<Frequency>] [--verbose] [--help]" << std::endl;
        std::cerr << "example:  " << argv[0] << " --cid=112 --freq=30 --verbose" << std::endl;
        return -1;
    }
    else
    {
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        if (0 == od4.isRunning())
        {
            std::cerr << "ERROR: No OD4Session running!!!" << std::endl;
            return -2;
        }

        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const float FREQ{(commandlineArguments["freq"].size() != 0) ? static_cast<float>(std::stof(commandlineArguments["freq"])) : static_cast<float>(-1.0)};

        // An example of message-receiving function
        // Also an example of data-triggered function
        float tempDistReading{0.0};
        auto onDistanceReading{[VERBOSE, &tempDistReading](cluon::data::Envelope &&envelope)
            // &<variables> will be captured by reference (instead of value only)
            {
                auto msg = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
                const uint16_t senderStamp = envelope.senderStamp(); // Local variables are not available outside the lambda function
                tempDistReading = msg.distance(); // Corresponds to odvd message set
                if (VERBOSE)
                {
                    std::cout << "Received DistanceReading message (senderStamp=" << senderStamp << "): " << tempDistReading << std::endl;
                }
            }
        };
        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);

        // An example of message-sending function
        // Also an example of time-triggered function
        float tempSteering{-0.2};
        auto sendGroundSteeringRequest{[&od4, VERBOSE, tempSteering]() -> bool
            {
                opendlv::proxy::GroundSteeringRequest steerReq;
                steerReq.groundSteering(tempSteering);
                od4.send(steerReq);
                if (VERBOSE)
                {
                    std::cout << "Sent GroundSteeringRequest message: " << tempSteering << std::endl;
                }
                return true;
            }
        };
        if (FREQ > 0)
        {
            od4.timeTrigger(FREQ, sendGroundSteeringRequest);
        }
        else
        {
            std::cerr << "WARNING: No acceptable frequency indicated." << std::endl;
        }

        while(od4.isRunning())
        {
            // An example of single time message-sending function
            const int16_t delay{1000}; // milliseconds

            opendlv::proxy::PedalPositionRequest pedalReq;
            pedalReq.position(0.4);
            od4.send(pedalReq); // This was called only once, hereinafter
            if (VERBOSE) std::cout << "Now move forward ...";
            std::this_thread::sleep_for(std::chrono::milliseconds(2 * delay));

            pedalReq.position(0.0);
            od4.send(pedalReq);
            if (VERBOSE) std::cout << " and stop." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));

            
            pedalReq.position(-0.3);
            od4.send(pedalReq);
            if (VERBOSE) std::cout << "Now go back ...";
            std::this_thread::sleep_for(std::chrono::milliseconds(2 * delay));

            pedalReq.position(0.0);
            od4.send(pedalReq);
            if (VERBOSE) std::cout << " and stop." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
        return 0;
    }
}

