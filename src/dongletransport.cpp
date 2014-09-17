#include "baromesh/dongletransport.hpp"

namespace {
const uint32_t kBaudRate = 230400;
const auto kSfpConnectTimeout = std::chrono::milliseconds(100);
const auto kSfpSettleTimeout = std::chrono::milliseconds(200);
const auto kRetryCooldown = std::chrono::milliseconds(200);
}

namespace dongle {

// Since this is a separate method, clients can connect to our signals
// before the reader thread starts and possibly fires those signals.
void Transport::startReaderThread () {
    try {
        std::thread t { &Transport::threadMain, this };
        mThread.swap(t);
    }
    catch (...) {
        throw ThreadError(std::current_exception());
    }
}

void Transport::sendMessage (const uint8_t* data, size_t size) {
    BOOST_LOG_NAMED_SCOPE("dongle::Transport::sendMessage");
    if (mState != State::connected) {
        BOOST_LOG(mLog) << "dongle not connected";
        throw NotConnected();
    }
    try {
        BOOST_LOG(mLog) << "sending message through SFP";
        mSfpContext.sendMessage(data, size);
        BOOST_LOG(mLog) << "message sent";
    }
    catch (boost::system::system_error& e) {
        BOOST_LOG(mLog) << "error sending message: " << e.what();
        throw SerialError(std::make_exception_ptr(e));
    }
}

void Transport::setState(State s) {
    BOOST_LOG_NAMED_SCOPE("dongle::Transport::setState");
    if (mState != s) {
        mState = s;
        switch(s) {
            case State::disconnected:
                BOOST_LOG(mLog) << "dongle transport disconnected";
                sigDisconnected();
                break;
            case State::connecting:
                BOOST_LOG(mLog) << "dongle transport connecting";
                sigConnecting();
                break;
            case State::connected:
                BOOST_LOG(mLog) << "dongle transport connected";
                sigConnected();
                break;
            default:
                assert(false);
        }
    }
}

void Transport::threadMain () {
    /* FIXME this thread can be started before the main thread, since it
     * (currently) lives in a static object. This isn't a problem, except that
     * it means calls to BOOST_LOG_NAMED_SCOPE() or BOOST_LOG_FUNCTION() will
     * segfault, if the main thread hasn't initialized the logging core yet.
     * Figure out a solution. */
    while (!mKillThread) {
        try {
            auto path = devicePath();
            BOOST_LOG(mLog) << "Found dongle device at " << path;
            setState(State::connecting);

            // Open the serial port and set up a read pump
            mSerial.close();
            mSerial.open(path);
            mSerial.set_option(boost::asio::serial_port_base::baud_rate(kBaudRate));
            BOOST_LOG(mLog) << path << " opened";
            asyncRead();

            // Initiate the SFP connection and set up a timer to tend it
            mSfpContext.initialize();
            asyncSfpConnect();

            // Run the read pump and SFP connection timer handlers
            mIoService.run();
            // If run() throws, it is permissible to restart it with no
            // intervening reset(), which is why this next line is
            // exception-safe.
            mIoService.reset();
        }
        catch (DongleNotFoundException& e) {
            // Dongle unplugged or turned off. Non-fatal.
        }
        catch (boost::system::system_error& e) {
            // Boost.Asio (i.e., I/O) errors. Non-fatal.
            BOOST_LOG(mLog) << "exception in reader thread: " << e.what();
        }
        // Let any other exceptions crash the program! They should only
        // be coming from catastrophic situations.
        // NO: catch (std::exception &e) {}

        if (!mKillThread) {
            setState(State::disconnected);
            std::this_thread::sleep_for(kRetryCooldown);
        }
    }
}

void Transport::asyncRead () {
    mSerial.async_read_some(boost::asio::buffer(mReadBuffer),
        BIND_MEM_CB(&Transport::readHandler, this));
}

void Transport::readHandler (const boost::system::error_code& ec, size_t nBytesRead) {
    if (ec) {
        BOOST_LOG(mLog) << "read error: " << ec.message();
        // If the read pump isn't operating, then the SFP connection process is
        // moot. Cancel it so mIoService.run() doesn't block needlessly.
        mSfpTimer.cancel();
        return;
    }

    for (size_t i = 0; i < nBytesRead; ++i) {
        mSfpContext.input(mReadBuffer[i]);
    }

    // That tasted great, gimme some more
    asyncRead();
}

void Transport::asyncSfpConnect () {
    mSfpContext.connect();
    mSfpTimer.expires_from_now(kSfpConnectTimeout);
    mSfpTimer.async_wait(BIND_MEM_CB(&Transport::sfpConnectHandler, this));
}

void Transport::sfpConnectHandler (const boost::system::error_code& ec) {
    if (ec) {
        BOOST_LOG(mLog) << "sfp connect timer error: " << ec.message();
        return;
    }

    if (mSfpContext.isConnected()) {
        BOOST_LOG(mLog) << "we think we're connected, settling ...";
        asyncSfpSettle();
    }
    else {
        // Keep trying, buddy
        asyncSfpConnect();
    }
}

void Transport::asyncSfpSettle () {
    mSfpTimer.expires_from_now(kSfpSettleTimeout);
    mSfpTimer.async_wait(BIND_MEM_CB(&Transport::sfpSettleHandler, this));
}

void Transport::sfpSettleHandler (const boost::system::error_code& ec) {
    if (ec) {
        BOOST_LOG(mLog) << "sfp settle timer error: " << ec.message();
        return;
    }

    if (mSfpContext.isConnected()) {
        BOOST_LOG(mLog) << "SFP connection handshake complete";
        mIoService.dispatch([this] () { setState(State::connected); });
    }
    else {
        BOOST_LOG(mLog) << "SFP connection failed to settle";
        mIoService.stop();
    }
}

} // namespace dongle
