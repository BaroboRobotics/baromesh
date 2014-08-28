#include "dongletransport.hpp"

namespace dongle {

void Transport::startReaderThread () {
    try {
        std::thread t { &Transport::threadMain, this };
        mThread.swap(t);
    }
    catch (...) {
        throw ThreadException(std::current_exception());
    }
}

void Transport::setState(State s) {
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

void Transport::threadMain () {
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

void Transport::threadOpenSerial (std::string path) {
    mSerial.close();
    mSerial.setPort(path);
    mSerial.open();
}

void Transport::threadConnectSfp () {
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

void Transport::threadPumpClientData () {
    readWhile([this] () { return !mKillThread; });
}

void Transport::readWhile(std::function<bool()> predicate, bool breakOnEmptyRead) {
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

} // namespace dongle
