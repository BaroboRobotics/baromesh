#include "robotproxy.hpp"
#include "dongleproxy.hpp"

#include <boost/unordered_map.hpp>

#include <algorithm>


void sendNewColor(robot::Proxy &robotProxy, double tim) {
    uint32_t red, green, blue;
    red = (sin(tim) + 1) * 127;
    green = (sin(tim + 2 * M_PI / 3) + 1) * 127;
    blue = (sin(tim + 4 * M_PI / 4) + 1) * 127;
    auto future = robotProxy.set(rpc::Attribute<barobo::Robot>::ledColor{red << 16 | green << 8 | blue});
    //printf("sent request\n");
    future.get();
    //printf("got reply\n");
}

void lavaLamp (std::string serialId) {
    double t = 0;
    robot::Proxy robotProxy { serialId };
    try {
        auto serviceInfo = robotProxy.connect().get();

        std::cout << "Local RPC version "
                  << rpc::Version<>::triplet() << '\n';
        std::cout << "Local barobo.Robot interface version "
                  << rpc::Version<barobo::Robot>::triplet() << '\n';

        std::cout << serialId << " RPC version "
                  << serviceInfo.rpcVersion() << '\n';
        std::cout << serialId << " barobo.Robot interface version "
                  << serviceInfo.interfaceVersion() << '\n';

        if (!serviceInfo.connected()) {
            std::cout << serialId << ": connection refused\n";
            return;
        }
        std::cout << serialId << ": connected\n";

        while (1) {
            sendNewColor(robotProxy, t);
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
