#include "dongleproxy.hpp"
#include "dongleproxy.hpp"
#include "serial_framing_protocol.h"
#include "serial/serial.h"

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
    std::cout << "Connected.\n";

    DongleProxy dongleProxy( [&ctx] (const DongleProxy::BufferType& buffer) { 
        sfpWritePacket(&ctx, buffer.bytes, buffer.size, nullptr);
    } );

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
                    //std::cout << statusToString(status) << "\n";
                    assert(!rpc::hasError(status));
                }
            }
        }
    }};

    dongleProxy.subscribe(rpc::Broadcast<barobo::Dongle>::receiveUnicast()).get();

    /* TODO: instantiate a RobotService and give it the contents of this for
     * loop as a post function. */
    for (unsigned i = 0; i < 255; ++i) {
        barobo_Dongle_Address destination;
        strcpy((char*)destination.serialId, "ZRG6");
        destination.port = 0;

        barobo_Dongle_Payload payload;
        auto hello = "Hello, world";
        strcpy((char*)payload.value.bytes, hello);
        payload.value.size = strlen(hello);

        dongleProxy.fire(rpc::MethodIn<barobo::Dongle>::transmitUnicast {
                destination, payload
        }).get();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    killThreads = true;
    t.join();
    return 0;
}
