#ifndef BAROMESH_IOCORE_HPP
#define BAROMESH_IOCORE_HPP

#include <boost/asio/io_service.hpp>

#include <boost/optional.hpp>

#include <boost/log/core/core.hpp>
#include <boost/log/sources/logger.hpp>

#include <future>
#include <memory>
#include <thread>

namespace baromesh {

class IoCore {
    explicit IoCore (boost::optional<bool> enableLogging);
    void maybeEnableLogging (boost::optional<bool> enable);
    void startThread ();

public:
    static std::shared_ptr<IoCore> get (boost::optional<bool> enableLogging = boost::none);
    ~IoCore ();

    boost::asio::io_service& ios () {
        return mIos;
    }

private:
    boost::log::core_ptr mLoggingCore;

    mutable boost::log::sources::logger_mt mLog;

    boost::asio::io_service mIos;
    boost::optional<boost::asio::io_service::work> mWork;
    std::weak_ptr<int> mToken;
    std::thread mThread;
};

} // namespace baromesh

#endif