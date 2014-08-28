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

// SFP settle timeout must be greater than or equal to the serial timeout above.
// Think about it.
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
                BIND_MEM_CB(&MessageReceived::operator(), &sigMessageReceived));
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
    MessageReceived sigMessageReceived;

    util::Signal<void()> sigNoDongle;
    util::Signal<void()> sigDongleConnecting;
    util::Signal<void()> sigDongleConnected;

private:
    void writeToUsb (uint8_t octet) {
        // FIXME: this is terribly inefficient to be doing for every single byte
        mSerial.write(&octet, 1);
    }

    enum class State {
        noDongle,
        dongleConnecting,
        dongleConnected
    };

    void setState(State s) {
        mState = s;
        switch(s) {
            case State::noDongle:
                sigNoDongle();
                break;
            case State::dongleConnecting:
                sigDongleConnecting();
                break;
            case State::dongleConnected:
                sigDongleConnected();
                break;
            default:
                assert(false);
        }
    }

    void threadMain () {
        while (!mKillThread) {
            try {
                auto path = devicePath();
                setState(State::dongleConnecting); // yellowLight();
                threadOpenSerial(path);
                threadConnectSfp();
                setState(State::dongleConnected); // greenLight();
                threadPumpClientData();
            }
            catch (DongleNotFoundException& exc) {
                // Dongle unplugged or turned off. Non-fatal.
            }
            // Too bad the serial exceptions don't all inherit from
            // SerialException. :|
            catch (serial::SerialException& exc) {
                std::cerr << exc.what() << '\n';
            }
            catch (serial::IOException& exc) {
                std::cerr << exc.what() << '\n';
            }
            catch (serial::PortNotOpenedException& exc) {
                std::cerr << exc.what() << '\n';
            }
            // Let any other exceptions crash the program! They should only
            // be coming from user code called as sfp callbacks, or
            // catastrophic situations.
            // NO: catch (std::exception &e) {}

            if (!mKillThread) {
                setState(State::noDongle); // redLight();
                std::this_thread::sleep_for(kRetryCooldown);
            }
        }
    }

    void threadOpenSerial (std::string path) {
        std::cerr << "threadOpenSerial\n";
        mSerial.close();
        mSerial.setPort(path);
        mSerial.open();
    }

    void readWhile(std::function<bool()> predicate, bool breakOnEmptyRead = false) {
        uint8_t byte;
        // TODO check mSerial.available() and read all the bytes available
        while (predicate()) {
            auto bytesread = mSerial.read(&byte, 1);
            if (bytesread) {
                mSfpContext.input(byte);
            }
            else if (breakOnEmptyRead) {
                break;
            }
        }
    }

    void threadConnectSfp () {
        std::cerr << "threadConnectSfp\n";
        using Clock = std::chrono::steady_clock;

        mSfpContext.initialize();
        while (!mKillThread && !mSfpContext.isConnected()) {
            mSfpContext.connect();
            readWhile([this] () {
                return !mKillThread && !mSfpContext.isConnected();
            }, true);
        }
        std::cerr << "threadConnectSfp: we think we're connected\n";

        // Although we think we're connected, we don't know if the remote host
        // agrees yet. Wait a little bit for the dust to settle.
        auto timeStop = Clock::now() + kSfpSettleTimeout;
        readWhile([this, timeStop] () {
            return !mKillThread && Clock::now() < timeStop;
        });
        std::cerr << "threadConnectSfp: settle timeout elapsed\n";
    }

    void threadPumpClientData () {
        std::cerr << "threadPumpClientData\n";
        readWhile([this] () { return !mKillThread; });
    }

    State mState = State::noDongle;

    sfp::Context mSfpContext;
    serial::Serial mSerial = { "", kBaudRate, kSerialTimeout };

    std::atomic<bool> mKillThread = { false };
    std::thread mThread;
};

} // namespace dongle

#endif
