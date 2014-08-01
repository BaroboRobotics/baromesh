#include "robotproxy.hpp"
#include "dongleproxy.hpp"
#include "serial_framing_protocol.h"
#include "serial/serial.h"

#include <boost/unordered_map.hpp>

#include <algorithm>

static int sfp_write(uint8_t *octets, size_t len, size_t *outlen, void *data)
{
    auto &usb = *static_cast<serial::Serial*>(data);
    usb.write(octets, len);
}

static void sfp_lock(void *data)
{
}

static void sfp_unlock(void *data)
{
}

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("Usage: %s <serial-id>\n", argv[0]);
        return 1;
    }

    const std::string serialId { argv[1] };

    serial::Serial usb("/dev/ttyACM0", 230400, serial::Timeout::simpleTimeout(1000));
    SFPcontext ctx;
    sfpInit(&ctx);
    sfpSetWriteCallback(&ctx, SFP_WRITE_MULTIPLE, (void*)sfp_write, &usb);
    sfpSetLockCallback(&ctx, sfp_lock, nullptr);
    sfpSetUnlockCallback(&ctx, sfp_unlock, nullptr);

    sfpConnect(&ctx);
    uint8_t byte;
    while(!sfpIsConnected(&ctx)) {
        auto bytesread = usb.read(&byte, 1);
        if(bytesread) {
            sfpDeliverOctet(&ctx, byte, nullptr, 0, nullptr);
        } else {
            return 1;
        }
    }
    std::cout << "Connected.\n";

    boost::unordered_map<std::string, RobotProxy> robots;

    DongleProxy dongleProxy {
        [&ctx] (const DongleProxy::BufferType& buffer) { 
            sfpWritePacket(&ctx, buffer.bytes, buffer.size, nullptr);
        },
        [&robots] (std::string serialId, const uint8_t* bytes, size_t size) {
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
    };

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
    return 0;
}
