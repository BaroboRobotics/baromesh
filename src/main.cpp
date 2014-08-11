#include "robotproxy.hpp"
#include "dongleproxy.hpp"
#include "reliableusb.hpp"

#include <boost/unordered_map.hpp>

#include <algorithm>

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

    // Set up the dongle proxy object with its USB/SFP backend.
    DongleProxy dongleProxy;

    ReliableUsb reliableUsb { std::string("/dev/ttyACM0") };
    reliableUsb.messageReceived.connect(
            BIND_MEM_CB(&DongleProxy::deliverMessage, &dongleProxy));

    dongleProxy.bufferPosted.connect(
            BIND_MEM_CB(&ReliableUsb::sendMessage, &reliableUsb));

#if 0
    boost::unordered_map<std::string, RobotProxy> robots;
    auto f = [&robots] (std::string serialId, const uint8_t* bytes, size_t size) {
        auto iter = robots.find(serialId);
        if (iter != robots.end()) {
            RobotProxy::BufferType buffer;
            memcpy(buffer.bytes, bytes, size);
            buffer.size = size;
            iter->second.deliver(buffer);
        }
        else {
            printf("Ain't no such robot named %s\n", serialId.c_str());
        }
    }

    std::atomic<bool> killThreads = {false};
    std::thread t{ [&] () {
        while(!killThreads) {
            uint8_t byte;
            auto bytesread = usb.read(&byte, 1);
            if(bytesread) {
                DongleProxy::BufferType buffer;
                auto ret = sfpDeliverOctet(&ctx, byte, buffer.bytes, sizeof(buffer.bytes), &buffer.size);
                if(ret > 0) {
                    auto status = dongleProxy.deliver(buffer);
                    std::cout << statusToString(status) << "\n";
                    assert(!rpc::hasError(status));
                }
            }
        }
    }};

    dongleProxy.subscribe(rpc::Broadcast<barobo::Dongle>::receiveUnicast()).get();

    bool success;
    decltype(robots)::iterator iter;
    std::tie(iter, success) = robots.emplace(std::piecewise_construct, std::forward_as_tuple(serialId),
        std::forward_as_tuple([&] (const RobotProxy::BufferType& buffer) {
            barobo_Dongle_Address destination;
            strcpy((char*)destination.serialId, serialId.c_str());
            destination.port = 0;

            barobo_Dongle_Payload payload;
            // FIXME check buffer sizes for overflow
            std::copy_n(buffer.bytes, buffer.size, payload.value.bytes);
            payload.value.size = buffer.size;

            printf("sending:");
            for (int i = 0; i < payload.value.size; ++i) {
                printf(" %02x", payload.value.bytes[i]);
            }
            printf("\n");

            dongleProxy.fire(rpc::MethodIn<barobo::Dongle>::transmitUnicast {
                    destination, payload
            }).get();
        })
    );

    assert(success);

    double tim = 0;

    while (1) {
        uint32_t red, green, blue;
        red = (sin(tim) + 1) * 127;
        green = (sin(tim + 2 * M_PI / 3) + 1) * 127;
        blue = (sin(tim + 4 * M_PI / 4) + 1) * 127;
        auto future = robots.at(serialId).set(rpc::Attribute<barobo::Robot>::ledColor{red << 16 | green << 8 | blue});
        printf("sent request\n");
        future.get();
        printf("got reply\n");
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
        tim += 0.05;
    }

    killThreads = true;
    t.join();
#endif
    return 0;
}
