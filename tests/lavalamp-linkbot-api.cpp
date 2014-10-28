#include "baromesh/linkbot.hpp"

#include <algorithm>
#include <iostream>
#include <thread>

#include <cassert>

#undef M_PI
#define M_PI 3.14159265358979323846

namespace br = barobo;

void sendNewColor(br::Linkbot& linkbot, double tim) {
    uint32_t red, green, blue;
    red = (sin(tim) + 1) * 127;
    green = (sin(tim + 2 * M_PI / 3) + 1) * 127;
    blue = (sin(tim + 4 * M_PI / 4) + 1) * 127;
    linkbot.setLedColor(red, green, blue);
}

void lavaLamp (std::string serialId) {
    double t = 0;
    br::Linkbot linkbot { serialId };
    try {
        linkbot.connect();
        linkbot.moveContinuous(7, br::MotorDir::FORWARD, br::MotorDir::FORWARD, br::MotorDir::FORWARD);
        while (1) {
            sendNewColor(linkbot, t);
            t += 0.05;
        }
    }
    catch (std::exception& e) {
        std::cout << std::hex;
        // FIXME: This serial ID should be information baked into e.what() in
        // some cases.
        std::cout << "(" << serialId << ") error setting color(" << t << "): "
                  << e.what() << '\n';
    }
}


int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <serial-id> [<serial-id> ...]\n", argv[0]);
        return 1;
    }

    // Get list of serial IDs from command line.
    std::vector<std::string> serialIds { argv + 1, argv + argc };

    // Ensure they all sorta look like serial IDs.
    assert(std::all_of(serialIds.cbegin(), serialIds.cend(),
                [] (const std::string& s) { return 4 == s.size(); }));

    std::vector<std::thread> lavaLampThreads;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (auto s : serialIds) {
        lavaLampThreads.emplace_back(lavaLamp, s);
    }

    for (auto& t: lavaLampThreads) {
        t.join();
    }

    return 0;
}
