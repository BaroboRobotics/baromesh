#ifndef BAROMESH_WEBSOCKETACCEPTOR_HPP
#define BAROMESH_WEBSOCKETACCEPTOR_HPP

#include <util/producerconsumerqueue.hpp>
#include <util/asio/asynccompletion.hpp>
#include <util/asio/transparentservice.hpp>

#include <baromesh/websocketlogger.hpp>
#include <baromesh/websocketmessagequeue.hpp>

#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/logger.hpp>

#include <websocketpp/server.hpp>
#include <websocketpp/close.hpp>
#include <websocketpp/config/asio_no_tls.hpp>

#include <tuple>
#include <utility>

namespace baromesh { namespace websocket {

class AcceptorImpl : public std::enable_shared_from_this<AcceptorImpl> {
public:
    // Server config with asio transport, TLS disabled, and Boost.Log logging
    struct Config : public ::websocketpp::config::asio {
        typedef Config type;
        typedef ::websocketpp::config::asio base;

        typedef Logger<base::concurrency_type, ::websocketpp::log::elevel> elog_type;
        typedef Logger<base::concurrency_type, ::websocketpp::log::alevel> alog_type;

        struct transport_config : public base::transport_config {
            typedef type::alog_type alog_type;
            typedef type::elog_type elog_type;
        };

        typedef ::websocketpp::transport::asio::endpoint<transport_config> transport_type;
    };

    using Connection = ::websocketpp::connection<Config>;
    using ConnectionPtr = Connection::ptr;
    using MessageQueue = ::baromesh::websocket::MessageQueue<Config>;

    explicit AcceptorImpl (boost::asio::io_service& ios)
        : mContext(ios)
    {
        mWsServer.init_asio(&mContext);
    }

    ~AcceptorImpl () {
        boost::system::error_code ec;
        close(ec);
    }

    void close (boost::system::error_code& ec) {
        ec = {};
        while (mConnectionQueue.depth() < 0) {
            mConnectionQueue.produce(boost::asio::error::operation_aborted, nullptr);
        }
        while (mConnectionQueue.depth() > 0) {
            mConnectionQueue.consume([this](boost::system::error_code ec2, ConnectionPtr ptr) {
                if (!ec2) {
                    BOOST_LOG(mLog) << "Discarding accepted connection from "
                        << ptr->get_uri()->str();
                }
                else {
                    BOOST_LOG(mLog) << "Discarding error message: " << ec2.message();
                }
            });
        }
    }

    void listen (const boost::asio::ip::tcp::endpoint& endpoint) {
        auto self = this->shared_from_this();
        mWsServer.set_open_handler(std::bind(&AcceptorImpl::handleOpen, self, _1));
        mWsServer.set_fail_handler(std::bind(&AcceptorImpl::handleOpen, self, _1));
        mWsServer.set_reuse_addr(true);
        mWsServer.listen(endpoint);
        mWsServer.start_accept();
    }

    boost::asio::ip::tcp::endpoint getLocalEndpoint () {
        boost::system::error_code ec;
        auto ep = mWsServer.get_local_endpoint(ec);
        if (ec) { throw boost::system::system_error(ec); }
        return ep;
    }

    template <class CompletionToken>
    BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, void(boost::system::error_code))
    asyncAccept (MessageQueue& mq, CompletionToken&& token) {
        util::asio::AsyncCompletion<
            CompletionToken, void(boost::system::error_code)
        > init { std::forward<CompletionToken>(token) };

        auto handler = init.handler;
        auto self = this->shared_from_this();
        mContext.post([&mq, handler, self, this]() mutable {
            auto consume = [&mq, handler](boost::system::error_code ec, ConnectionPtr con) mutable {
                if (!ec) {
                    mq.setConnectionPtr(con);
                }
                handler(ec);
            };
            mConnectionQueue.consume(consume);
        });

        return init.result.get();
    }

private:
    void handleOpen (::websocketpp::connection_hdl hdl) {
        auto ec = boost::system::error_code{};
        auto con = mWsServer.get_con_from_hdl(hdl, ec);
        if (!ec) {
            assert(con);
            auto self = this->shared_from_this();
            mContext.post([con, self, this]() mutable {
                // The open and fail handlers hold pointers to the Acceptor oject. Kill them before
                // we let them escape. Gotta do this inside mContext.post() so we don't
                // accidentally destroy our own currently-executing std::function object.
                con->set_open_handler(nullptr);
                con->set_fail_handler(nullptr);
                mConnectionQueue.produce(con->get_transport_ec(), con);
            });
        }
        else {
            BOOST_LOG(mLog) << "Open handler could not get connection pointer: " << ec.message();
        }
    }

    boost::asio::io_service& mContext;
    ::websocketpp::server<Config> mWsServer;
    util::ProducerConsumerQueue<boost::system::error_code, ConnectionPtr> mConnectionQueue;

    mutable boost::log::sources::logger mLog;
};

class Acceptor : public util::asio::TransparentIoObject<AcceptorImpl> {
public:
    explicit Acceptor (boost::asio::io_service& ios)
        : util::asio::TransparentIoObject<AcceptorImpl>(ios)
    {}

    void listen (const boost::asio::ip::tcp::endpoint& ep) {
        this->get_implementation()->listen(ep);
    }

    boost::asio::ip::tcp::endpoint getLocalEndpoint () {
        return this->get_implementation()->getLocalEndpoint();
    }

    using MessageQueue = AcceptorImpl::MessageQueue;
    UTIL_ASIO_DECL_ASYNC_METHOD(asyncAccept)
};

}} // namespace baromesh::websocket

#endif
