#ifndef BAROMESH_DONGLETRANSPORT_HPP
#define BAROMESH_DONGLETRANSPORT_HPP

#include "dongledevicepath.hpp"

#define SFP_CONFIG_THREADSAFE
#include "sfp/context.hpp"
#include "serial/serial.h"

#include "dongleexception.hpp"

namespace dongle {

const uint32_t kBaudRate = 230400;
const auto kSerialTimeout = serial::Timeout::simpleTimeout(200);

// SFP timeouts must be greater than or equal to the serial timeout above.
// Think about it.
const auto kSfpConnectionTimeout = std::chrono::milliseconds(5000);
const auto kSfpSettleTimeout = std::chrono::milliseconds(200);

const auto kRetryCooldown = std::chrono::milliseconds(200);

// Encapsulate serial::Serial and sfp::Context to create a reliable,
// message-oriented USB link.
class Transport {
public:


    Transport () {
        mSfpContext.output.connect(
                BIND_MEM_CB(&Transport::writeToUsb, this));
        mSfpContext.messageReceived.connect(
                BIND_MEM_CB(&MessageReceived::operator(), &messageReceived));
    }

    ~Transport () {
        mKillThread = true;
        mThread.join();
    }

    void startReaderThread () {
        try {
            std::thread t { &Transport::threadMain, this };
            mThread.swap(t);
        }
        catch (...) {
            throw ThreadException(std::current_exception());
        }
    }

    void sendMessage (const uint8_t* data, size_t size) {
        mSfpContext.sendMessage(data, size);
    }

    using MessageReceived = util::Signal<void(const uint8_t*,size_t)>;
    MessageReceived messageReceived;
    
    enum class DownReason {
        NORMALLY, EXCEPTIONALLY
    };

    util::Signal<void()> linkUp;
    util::Signal<void(DownReason)> linkDown;

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
            if (threadInitialize()) {
                linkUp();
                try {
                    threadRun();
                    linkDown(DownReason::NORMALLY);
                }
                catch (...) {
                    linkDown(DownReason::EXCEPTIONALLY);
                }
            }
            if (!mKillThread) {
                std::this_thread::sleep_for(kRetryCooldown);
            }
        }
    }

    // True if initialization succeeded, false otherwise.
    bool threadInitialize () {
        std::cerr << "threadInitialize\n";
        // Get the dongle device path, i.e.: /dev/ttyACM0, \\.\COM3, etc.
        char path[64];
        auto status = devicePath(path, sizeof(path));
        if (-1 == status) {
            std::cerr << "threadInitialize: no dongle found\n";
            return false;
        }

        std::cerr << "threadInitialize: found dongle at " << path << "\n";
        try {
            threadConstructUsb(std::string(path));
            threadConnectSfp();
        }
        catch (...) {
            std::cout << "threadConstructUsb threw\n";
            // TODO Figure out what to do with the following cases:
            // perms?
            // read error
            // libsfp failure?
            // break, continue?
            return false;
        }
        return true;
    }

    void threadConstructUsb (std::string path) {
        std::cerr << "threadConstructUsb\n";
        std::unique_lock<std::timed_mutex> lock {
            mUsbMutex,
            std::chrono::seconds(10)
        };
        if (!lock) {
            throw std::runtime_error("timed out waiting on USB lock");
        }
        mUsb.reset(new serial::Serial(path, kBaudRate, kSerialTimeout));
    }

    void readPump(bool breakOnEmptyRead, std::function<bool()> predicate) {
        uint8_t byte;
        // TODO check mUsb.available() and read all the bytes available
        while (predicate()) {
            auto bytesread = mUsb->read(&byte, 1);
            if (bytesread) {
                mSfpContext.input(byte);
            }
            else if(breakOnEmptyRead) {
                break;
            }
        }
    }

    void threadConnectSfp () {
        std::cerr << "threadConnectSfp\n";
        assert(mUsb);

        using Clock = std::chrono::steady_clock;

        mSfpContext.initialize();
        auto timeStop = Clock::now() + kSfpConnectionTimeout;
        while (!mKillThread && !mSfpContext.isConnected()
                && Clock::now() < timeStop) {
            mSfpContext.connect();
            // exception = MEH. That means catching and ignoring
            // ReadPumpException?
            readPump(true, [this] () {
                return !mKillThread && !mSfpContext.isConnected();
            });
        }

        if (!mKillThread && !mSfpContext.isConnected()) {
            throw SfpConnectionException();
        }

        std::cerr << "threadConnectSfp: we think we're connected\n";

        // Although we think we're connected, we don't know if the remote host
        // agrees yet. Wait a little bit for the dust to settle.
        timeStop = Clock::now() + kSfpSettleTimeout;

        readPump(false, [this, timeStop] () {
            return !mKillThread && Clock::now() < timeStop;
        });

        std::cerr << "threadConnectSfp: settle timeout elapsed\n";
    }

    void threadPumpClientData () {
        assert(mUsb);
        std::cerr << "threadPumpClientData\n";
        try {
            readPump(false, [this] () { return !mKillThread; });
        }
        catch (...) {
            throw ReadPumpException(std::current_exception());
        }
    }

    sfp::Context mSfpContext;
    std::unique_ptr<serial::Serial> mUsb;
    std::timed_mutex mUsbMutex;

    std::atomic<bool> mKillThread = { false };
    std::thread mThread;
};

} // namespace dongle

#endif
