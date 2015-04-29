#ifndef BAROMESH_IOCORE_HPP
#define BAROMESH_IOCORE_HPP

#include <boost/asio/io_service.hpp>

#include <boost/log/sources/logger.hpp>

#include <boost/optional.hpp>

#include <future>
#include <memory>

namespace baromesh {

class IoCore {
    explicit IoCore ();

public:
    static std::shared_ptr<IoCore> get ();
    ~IoCore ();

    boost::asio::io_service& ios () {
        return mIos;
    }

private:
    mutable boost::log::sources::logger_mt mLog;

    boost::asio::io_service mIos;
    boost::optional<boost::asio::io_service::work> mWork;

    std::future<size_t> mNHandlers;
};

} // namespace baromesh

#endif