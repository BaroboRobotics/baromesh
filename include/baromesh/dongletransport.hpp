#ifndef BAROMESH_DONGLETRANSPORT_HPP
#define BAROMESH_DONGLETRANSPORT_HPP

#include "dongledevicepath.hpp"
#include "dongleexception.hpp"

#define SFP_CONFIG_THREADSAFE
#include "sfp/context.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/write.hpp>

#include <boost/log/common.hpp>
#include <boost/log/sources/logger.hpp>

#include <iostream>
#include <atomic>

namespace dongle {

// Encapsulate boost::asio::serial_port and sfp::Context to create a reliable,
// message-oriented USB link.
class Transport {
public:
    Transport ()
        : mSerial(mIoService)
        , mSfpTimer(mIoService)
    {
        mSfpContext.output.connect(
                BIND_MEM_CB(&Transport::writeToUsb, this));
        using MessageReceived = decltype(sigMessageReceived);
        mSfpContext.messageReceived.connect(
                BIND_MEM_CB(&MessageReceived::operator(), &sigMessageReceived));
    }

    ~Transport () {
        mKillThread = true;
        mIoService.stop();
        mThread.join();
    }

    void startReaderThread();

    void sendMessage (const uint8_t* data, size_t size);

    util::Signal<void(const uint8_t*, size_t)> sigMessageReceived;
    util::Signal<void()> sigDisconnected;
    util::Signal<void()> sigConnecting;
    util::Signal<void()> sigConnected;

private:
    enum class State {
        disconnected,
        connecting,
        connected
    };

    void setState (State);

    void writeToUsb (uint8_t octet) {
        boost::asio::write(mSerial, boost::asio::buffer(&octet, 1));
    }

    void threadMain ();

    void asyncRead ();
    void readHandler (const boost::system::error_code&, size_t);

    void asyncSfpConnect ();
    void sfpConnectHandler (const boost::system::error_code&);

    void asyncSfpSettle ();
    void sfpSettleHandler (const boost::system::error_code&);

    boost::asio::io_service mIoService;
    boost::asio::serial_port mSerial;
    boost::asio::steady_timer mSfpTimer;

    std::array<uint8_t, 512> mReadBuffer;

    boost::log::sources::logger mLog;

    std::atomic<State> mState = { State::disconnected };
    sfp::Context mSfpContext;
    std::atomic<bool> mKillThread = { false };
    std::thread mThread;
    std::string mDevicePath;
};

} // namespace dongle

#endif
