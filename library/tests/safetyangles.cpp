#include "baromesh/linkbot.hpp"

#include <algorithm>
#include <iostream>

#include <cassert>

void testSafetyAngles (std::string serialId) {
    double t = 0;
    try {
        barobo::Linkbot linkbot { serialId };

        std::cout << "Linkbot " << serialId << " constructed\n";

        double a[] = {-0.5, 0.5, 5.75};
        double b[] = {0, 0, 0};

        std::cout << "Setting safety angles: " << a[0] << ", " << a[1] << ", " << a[2] << '\n';
        linkbot.setJointSafetyAngles(0x07, a[0], a[1], a[2]);
        linkbot.getJointSafetyAngles(b[0], b[1], b[2]);
        std::cout << "Safety angles are now: " << b[0] << ", " << b[1] << ", " << b[2] << '\n';

        assert(std::equal(a, a + 3, b));
    }
    catch (std::exception& e) {
        std::cout << std::hex;
        // FIXME: This serial ID should be information baked into e.what() in
        // some cases.
        std::cout << "(" << serialId << ") error running test routine: "
                  << e.what() << '\n';
    }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Usage: %s <serial-id>\n", argv[0]);
        return 1;
    }

    auto serialId = std::string{argv[1]};
    assert(4 == s.size());

    testSafetyAngles(serialId);

    return 0;
}
