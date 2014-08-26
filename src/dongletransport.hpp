#ifndef BAROMESH_DONGLETRANSPORT_HPP
#define BAROMESH_DONGLETRANSPORT_HPP

#define SFP_CONFIG_THREADSAFE
#include "sfp/context.hpp"
#include "serial/serial.h"

#include <boost/scope_exit.hpp>

constexpr const uint32_t kBaudRate = 230400;
constexpr const uint32_t kSerialTimeout = 1000;

// Encapsulate serial::Serial and sfp::Context to create a reliable,
// message-oriented USB link.
class DongleTransport {
public:
    DongleTransport ()
            : mThread(&DongleTransport::threadMain, this) {
        mSfpContext.output.connect(
                BIND_MEM_CB(&DongleTransport::writeToUsb, this));
        mSfpContext.messageReceived.connect(
                BIND_MEM_CB(&MessageReceived::operator(), &messageReceived));
    }

    ~DongleTransport () {
        mKillThread = true;
        mThread.join();
    }

    void sendMessage (const uint8_t* data, size_t size) {
        mSfpContext.sendMessage(data, size);
    }

    using MessageReceived = util::Signal<void(const uint8_t*,size_t)>;
    MessageReceived messageReceived;
    
    util::Signal<void()> linkUp;
    util::Signal<void()> linkDown;

private:
    void writeToUsb (uint8_t octet) {
        // FIXME: this is terribly inefficient to be doing for every single byte
        std::unique_lock<std::timed_mutex> lock {
            mUsbMutex,
            std::chrono::seconds(10)
        };
        if (!lock) {
            throw std::runtime_error("timed out waiting on USB lock");
        }
        if (mUsb) {
            mUsb->write(&octet, 1);
        }
        else {
            throw std::runtime_error("no dongle present");
        }
    }

    void threadMain () {
        while (!mKillThread) {
            // call some function to wait until a dongle is plugged in
            // ensure that this function is written in such a way that you can 
            // check mKillThread now and then, though...
            std::string devicePath { "/dev/ttyACM0" };
            try {
                threadConstructUsb(devicePath); // FIXME what if I throw? linkUp
                                                // was never called
            }
            catch (...) {
                // TODO Figure out what to do with the following cases:
                // perms?
                // read error
                // libsfp failure?
                // break, continue?
            }
            linkUp();
            try {
                threadRun();
                linkDown(); // normally
            }
            catch (...) {
                linkDown(); // exceptionally
            }
        }
    }

    void threadConstructUsb (std::string devicePath) {
        {
            std::unique_lock<std::timed_mutex> lock {
                mUsbMutex,
                std::chrono::seconds(10)
            };
            if (!lock) {
                throw std::runtime_error("timed out waiting on USB lock");
            }
            mUsb.reset(new serial::Serial
               ( devicePath
               , kBaudRate
               , serial::Timeout::simpleTimeout(kSerialTimeout)));
        }
        
        mSfpContext.connect();

        uint8_t byte;
        while (!mKillThread && !mSfpContext.isConnected()) {
            auto bytesread = mUsb->read(&byte, 1);
            if (bytesread) {
                mSfpContext.input(byte);
            }
            else {
                throw std::runtime_error("libsfp connection failure");
            }
        }
    }

    void threadRun () {
        while (!mKillThread) {
            uint8_t byte;
            // Block until kSerialTimeout milliseconds have elapsed.
            // TODO check mUsb.available() and read all the bytes available
            auto bytesread = mUsb->read(&byte, 1);
            if (bytesread) {
                mSfpContext.input(byte);
            }
        }
    }

    sfp::Context mSfpContext;
    std::unique_ptr<serial::Serial> mUsb;
    std::timed_mutex mUsbMutex;

    std::atomic<bool> mKillThread = { false };
    std::thread mThread;
};

#endif
