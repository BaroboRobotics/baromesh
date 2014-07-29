#include "robotproxy.hpp"
#include "serial_framing_protocol.h"
#include "serial/serial.h"

#include <list>

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

int main()
{
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
    printf("0x%X\n", rpc::ComponentId<barobo::Robot>::ledColor );
    uint32_t val = rpc::ComponentId<barobo::Robot>::ledColor;
    int i;
    for(i = 0; i < 4; i++) {
        printf("0x%X\n", ((uint8_t*)&val)[i]);
    }
    std::cout << "Connected.\n";

    RobotProxy robotProxy( [&ctx] (const RobotProxy::BufferType& buffer) { 
        printf("Sending buffer...\n");
        int i;
        for(i = 0; i < buffer.size; i++) {
            printf("send 0x%0X\n", buffer.bytes[i]);
        }
        sfpWritePacket(&ctx, buffer.bytes, buffer.size, nullptr);
    } );

    std::atomic<bool> killThreads = {false};
    std::thread t{ [&] () {
        while(!killThreads) {
            uint8_t byte;
            auto bytesread = usb.read(&byte, 1);
            if(bytesread) {
                printf("Received byte: 0x%0X\n", byte);
                RobotProxy::BufferType buffer;
                auto ret = sfpDeliverOctet(&ctx, byte, buffer.bytes, sizeof(buffer.bytes), &buffer.size);
                if(ret > 0) {
                    auto status = robotProxy.deliver(buffer);
                    //std::cout << statusToString(status) << "\n";
                    assert(!rpc::hasError(status));
                }
            }
        }
    }};
    using Attribute = rpc::Attribute<barobo::Robot>;
    auto future = robotProxy.set(Attribute::ledColor{0x80808080});
    std::cout << "Getting future..." << std::endl;
    future.get();
    std::cout << "Future gotten." << std::endl;

    std::list<std::future<void>> colorFutures;
    for (unsigned i = 0; i < 255; ++i) {
        colorFutures.emplace_back(robotProxy.set(Attribute::ledColor{ i << 8 }));
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    for (auto& f : colorFutures) { f.get(); }

    killThreads = true;
    t.join();
    //std::cout << future.get().value << "\n";
    return 0;
}
