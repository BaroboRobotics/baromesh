#include "baromesh/linkbot.hpp"

#include <algorithm>
#include <iostream>
#include <chrono>
#include <thread>

#include <cassert>

#undef M_PI
#define M_PI 3.14159265358979323846

void sendNewColor(barobo::Linkbot& linkbot, double tim) {
    uint32_t red, green, blue;
    red = (sin(tim) + 1) * 127;
    green = (sin(tim + 2 * M_PI / 3) + 1) * 127;
    blue = (sin(tim + 4 * M_PI / 4) + 1) * 127;
    linkbot.setLedColor(red, green, blue);
}

void lavaLamp (std::string host, std::string service) {
    double t = 0;
    try {
        barobo::Linkbot linkbot { host, service };
        std::cout << host << ":" << service << ": connected\n";

        while (1) {
            sendNewColor(linkbot, t);
            t += 0.05;
        }
    }
    catch (std::exception& e) {
        std::cout << std::hex;
        // FIXME: This serial ID should be information baked into e.what() in
        // some cases.
        std::cout << "(" << host << ":" << service << ") error setting color(" << t << "): "
                  << e.what() << '\n';
    }
}


int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Usage: %s <host> <service>\ne.g., %s 127.0.0.1 42010", argv[0], argv[0]);
        return 1;
    }

    lavaLamp(argv[1], argv[2]);
}
