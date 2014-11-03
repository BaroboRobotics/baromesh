#include "baromesh/linkbot.hpp"

#include <algorithm>
#include <iostream>
#include <thread>

#include <cassert>

#undef M_PI
#define M_PI 3.14159265358979323846

void testMovement (std::string serialId) {
    double t = 0;
    barobo::Linkbot linkbot { serialId };
    try {
        using std::this_thread::sleep_for;
        using std::chrono::seconds;
        using std::chrono::milliseconds;
        const auto jointMask = 0x04 | 0x01;

        linkbot.connect();

        std::cout << "moving forward quickly for a second\n";
        linkbot.setJointSpeeds(jointMask, 200, 0, 200);
        linkbot.moveContinuous(jointMask, 1, 0, -1);
        sleep_for(seconds(1));
        linkbot.stop();
        sleep_for(milliseconds(500));

        std::cout << "moving backward quickly for a second\n";
        // Note: not setting joint speeds.
        linkbot.moveContinuous(jointMask, -1, 0, 1);
        sleep_for(seconds(1));
        linkbot.stop();
        sleep_for(milliseconds(500));

        std::cout << "moving forward slowly for two seconds\n";
        linkbot.setJointSpeeds(jointMask, 40, 0, 40);
        linkbot.moveContinuous(jointMask, 1, 0, -1);
        sleep_for(seconds(2));
        linkbot.stop();
        sleep_for(milliseconds(500));

        std::cout << "moving backward slowly for two seconds\n";
        // Note: not setting joint speeds.
        linkbot.moveContinuous(jointMask, -1, 0, 1);
        sleep_for(seconds(2));
        linkbot.stop();
        sleep_for(milliseconds(500));

        std::cout << "moving forward slowly 200 degrees\n";
        // Note: not setting joint speeds.
        linkbot.move(0x04 | 0x01, 200, 0, -200);
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

    std::vector<std::thread> testThreads;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (auto s : serialIds) {
        testThreads.emplace_back(testMovement, s);
    }

    for (auto& t: testThreads) {
        t.join();
    }

    return 0;
}
