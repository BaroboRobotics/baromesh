#ifndef BAROMESH_DONGLETRANSPORT_HPP
#define BAROMESH_DONGLETRANSPORT_HPP

#include "dongledevicepath.hpp"

#define SFP_CONFIG_THREADSAFE
#include "sfp/context.hpp"
#include "serial/serial.h"

#include "dongleexception.hpp"

#include <iostream>
#include <atomic>

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

    void startReaderThread();

    void sendMessage (const uint8_t* data, size_t size) {
        mSfpContext.sendMessage(data, size);
    }

    using MessageReceived = util::Signal<void(const uint8_t*,size_t)>;
    MessageReceived sigMessageReceived;

    util::Signal<void()> sigNoDongle;
    util::Signal<void()> sigDongleConnecting;
    util::Signal<void()> sigDongleConnected;

private:
    enum class State {
        noDongle,
        dongleConnecting,
        dongleConnected
    };

    void writeToUsb (uint8_t octet) { mSerial.write(&octet, 1); }
    void threadMain ();
    void threadOpenSerial (std::string path);
    void threadConnectSfp ();
    void threadPumpClientData ();
    void readWhile(std::function<bool()> predicate, bool breakOnEmptyRead = false);
    void setState(State);

    State mState = State::noDongle;
    sfp::Context mSfpContext;
    serial::Serial mSerial = { "", kBaudRate, kSerialTimeout };
    std::atomic<bool> mKillThread = { false };
    std::thread mThread;
};

} // namespace dongle

#endif
