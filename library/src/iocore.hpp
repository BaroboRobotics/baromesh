#ifndef BAROMESH_IOCORE_HPP
#define BAROMESH_IOCORE_HPP

#include <boost/asio.hpp>
#include <boost/optional.hpp>

#include <thread>

namespace baromesh {

class IoCore {
    friend IoCore& ioCore ();

    IoCore ();
    void init ();

public:
    ~IoCore ();

    boost::asio::io_service& ios () {
        return mIos;
    }

private:
    boost::asio::io_service mIos;
    boost::optional<boost::asio::io_service::work> mWork;
    std::thread mThread;
};

IoCore& ioCore ();

} // namespace baromesh

#endif