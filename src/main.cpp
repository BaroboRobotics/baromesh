#include "robotproxy.hpp"
#include "dongleproxy.hpp"

#include <boost/unordered_map.hpp>

#include <algorithm>


void sendNewColor(RobotProxy &robotProxy, double tim) {
    uint32_t red, green, blue;
    red = (sin(tim) + 1) * 127;
    green = (sin(tim + 2 * M_PI / 3) + 1) * 127;
    blue = (sin(tim + 4 * M_PI / 4) + 1) * 127;
    auto future = robotProxy.set(rpc::Attribute<barobo::Robot>::ledColor{red << 16 | green << 8 | blue});
    printf("sent request\n");
    future.get();
    printf("got reply\n");
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

    dongle::Proxy dongleProxy;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    dongleProxy.subscribe(rpc::Broadcast<barobo::Dongle>::receiveUnicast()).get();

    auto& serialId = serialIds[0];
    RobotProxy robotProxy { serialId, dongleProxy };

    auto robotMessageHandler = [&robotProxy, &serialId] (std::string incomingSerialId,
              const uint8_t* bytes, size_t size) {
          RobotProxy::BufferType buffer;
          assert(incomingSerialId == serialId);
          assert(size <= sizeof(buffer.bytes));
          memcpy(buffer.bytes, bytes, size);
          buffer.size = size;
          robotProxy.receiveServiceBuffer(buffer);
    };
    using Lambda = decltype(robotMessageHandler);

    dongleProxy.robotMessageReceived.connect(
            BIND_MEM_CB(&Lambda::operator(), &robotMessageHandler));

    robotProxy.subscribe(rpc::Broadcast<barobo::Robot>::buttonEvent()).get();
    printf("Subscribed to button events\n");

    double tim = 0;
    while (1) {
        //sendNewColor(robotProxy, tim);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        tim += 0.05;
    }
    return 0;
}
