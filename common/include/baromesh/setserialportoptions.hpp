#ifndef BAROMESH_SETSERIALPORTOPTIONS_HPP
#define BAROMESH_SETSERIALPORTOPTIONS_HPP

#include <boost/asio/async_result.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include <chrono>
#include <memory>

#ifdef __MACH__
#include <unistd.h>
#endif

namespace baromesh {

// How long we pause after opening the dongle's device path before setting the
// serial line options. Mac serial ports require some strategic timing
// ninjitsu in order to work, adjust this value as necessary.
static const std::chrono::milliseconds kDongleSettleTimeAfterOpen { 500 };

// How many times we should try setting a serial option before throwing an
// error. On OS X 10.11, it sometimes requires dozens of attempts to set serial
// line options. Setting this to zero is the same as setting it to one.
static const int kMaxSerialSetOptionAttempts { 50 };

template <class Option>
void tenaciousSetOption (boost::asio::serial_port& sp, Option value, const int maxAttempts) {
    auto attempts = 0;
    auto ec = boost::system::error_code{};
    do {
        ec = boost::system::error_code{};
        sp.set_option(value, ec);
    } while (maxAttempts > 0
             && ++attempts != maxAttempts
             && ec);
    if (attempts > 1) {
        boost::log::sources::logger lg;
        BOOST_LOG(lg) << "set serial option after " << attempts << " attempts";
    }
    if (ec) {
        throw boost::system::system_error(ec);
    }
}

void setSerialPortOptions (boost::asio::serial_port& sp, int baud) {
    using Option = boost::asio::serial_port_base;
    const auto max = kMaxSerialSetOptionAttempts;
    tenaciousSetOption(sp, Option::baud_rate(baud), max);
    tenaciousSetOption(sp, Option::character_size(8), max);
    tenaciousSetOption(sp, Option::parity(Option::parity::none), max);
    tenaciousSetOption(sp, Option::stop_bits(Option::stop_bits::one), max);
    tenaciousSetOption(sp, Option::flow_control(Option::flow_control::none), max);
#ifdef __MACH__
    auto handle = sp.native_handle();
    ::write(handle, nullptr, 0);
#endif
}


template <class Handler>
class RobustOpenOperation : public std::enable_shared_from_this<RobustOpenOperation<Handler>> {
public:
    RobustOpenOperation (boost::asio::serial_port& sp, std::string path, int baud)
        : mSp(sp)
        , mTimer(sp.get_io_service())
        , mStrand(sp.get_io_service())
        , mPath(path)
        , mBaud(baud)
    {}

    void start (Handler handler) {
        try {
            mSp.open(mPath);
            mTimer.expires_from_now(kDongleSettleTimeAfterOpen);
            mTimer.async_wait(mStrand.wrap(
                std::bind(&RobustOpenOperation::stepOne,
                    this->shared_from_this(), handler, _1)));
        }
        catch (boost::system::system_error& e) {
            mSp.get_io_service().post(std::bind(handler, e.code()));
        }
    }

private:
    void stepOne (Handler handler, boost::system::error_code ec) {
        if (!ec) {
            setSerialPortOptions(mSp, mBaud);
        }
        mSp.get_io_service().post(std::bind(handler, ec));
    }

    boost::asio::serial_port& mSp;
    boost::asio::steady_timer mTimer;
    boost::asio::io_service::strand mStrand;

    std::string mPath;
    int mBaud;
};

typedef void RobustOpenHandlerSignature(boost::system::error_code);

template <class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, RobustOpenHandlerSignature)
asyncRobustOpen (boost::asio::serial_port& sp, std::string path, int baud, Handler&& handler) {
    boost::asio::detail::async_result_init<
        Handler, RobustOpenHandlerSignature
    > init { std::forward<Handler>(handler) };

    using Op = RobustOpenOperation<decltype(init.handler)>;
    std::make_shared<Op>(sp, path, baud)->start(init.handler);

    return init.result.get();
}

} // namespace baromesh

#endif