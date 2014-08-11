#ifndef RELIABLEUSB_HPP
#define RELIABLEUSB_HPP

#define SFP_CONFIG_THREADSAFE
#include "sfp/context.hpp"
#include "serial/serial.h"

constexpr const uint32_t kBaudRate = 230400;
constexpr const uint32_t kSerialTimeout = 1000;

// Encapsulate serial::Serial and sfp::Context to create a reliable,
// message-oriented USB link.
class ReliableUsb {
public:
    // TODO: catch serial::PortNotOpenedException from ctor
    ReliableUsb (std::string devicePath)
            : mUsb ( devicePath
                   , kBaudRate
                   , serial::Timeout::simpleTimeout(kSerialTimeout)) {
        mSfpContext.output.connect(
                BIND_MEM_CB(&ReliableUsb::writeToUsb, this));

        mSfpContext.messageReceived.connect(
                BIND_MEM_CB(&MessageReceived::operator(), &messageReceived));

        mSfpContext.connect();

        uint8_t byte;
        while(!mSfpContext.isConnected()) {
            auto bytesread = mUsb.read(&byte, 1);
            if(bytesread) {
                mSfpContext.input(byte);
            }
            else {
                throw std::runtime_error("libsfp connection failure");
            }
        }

        // Construction complete, start our engine!
        std::thread t { &ReliableUsb::run, this };
        mThread.swap(t);
    }

    ~ReliableUsb () {
        mKillThread = true;
        mThread.join();
    }

    void sendMessage (const uint8_t* data, size_t size) {
        mSfpContext.sendMessage(data, size);
    }

    using MessageReceived = util::Signal<void(const uint8_t*,size_t)>;
    MessageReceived messageReceived;

private:
    void writeToUsb (uint8_t octet) { mUsb.write(&octet, 1); }

    void run () {
        while (!mKillThread) {
            uint8_t byte;
            // Block until kSerialTimeout milliseconds have elapsed.
            auto bytesread = mUsb.read(&byte, 1);
            if (bytesread) {
                mSfpContext.input(byte);
            }
        }
    }

    sfp::Context mSfpContext;
    serial::Serial mUsb;

    std::atomic<bool> mKillThread = { false };
    std::thread mThread;
};

#endif
