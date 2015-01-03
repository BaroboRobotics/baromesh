// This header defines the barobo::Linkbot class.
#include "baromesh/linkbot.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main (int argc, char** argv) {
    if (argc != 2) {
        auto programName = argv[0];
        std::cerr << "Looks like you forgot to specify a robot ID.\n"
                  << "Usage: " << programName << " <robot ID>\n";
        return 1;
    }

    auto robotId = argv[1];
    try {
        std::cout << "Connecting to " << robotId << "...\n";

        // To connect to a Linkbot, instantiate an object of that type.
        barobo::Linkbot linkbot { robotId };

        std::cout << "Connected.\n";

        // Now we can control the Linkbot with the member functions defined in
        // linkbot.hpp.

        // Move the motors for a few seconds.
        auto allJoints = 0x07; // bits 1, 2, and 3
        linkbot.setJointSpeeds(allJoints, 120.0, 120.0, 120.0);
        linkbot.moveContinuous(allJoints, 1.0, 1.0, 1.0);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        linkbot.stop(allJoints);

        // The Linkbot destructor, run automatically at the end of this block,
        // will disconnect for us.
    }
    catch (std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }

    std::cout << "All done.\n";

    return 0;
}
