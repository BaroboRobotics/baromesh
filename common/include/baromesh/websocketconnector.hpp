#ifndef BAROMESH_WEBSOCKETCONNECTOR_HPP
#define BAROMESH_WEBSOCKETCONNECTOR_HPP

#include <util/index_sequence.hpp>
#include <util/asio/asynccompletion.hpp>
#include <util/asio/iothread.hpp>
#include <util/asio/transparentservice.hpp>

#include <baromesh/websocketconnectorconfig.hpp>
#include <baromesh/websocketmessagequeue.hpp>

#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/logger.hpp>

#include <websocketpp/client.hpp>
#include <websocketpp/close.hpp>

#include <tuple>
#include <utility>

namespace baromesh { namespace websocket {

class ConnectorImpl : public std::enable_shared_from_this<ConnectorImpl> {
public:
    using Config = ConnectorConfig;
    using Connection = ::websocketpp::connection<Config>;
    using ConnectionPtr = Connection::ptr;
    using MessageQueue = MessageQueue<Config>;

    explicit ConnectorImpl (boost::asio::io_service& ios)
        : mContext(ios)
    {
        mWsClient.init_asio(&mContext);
        // We can't set generic open/fail handlers here, because they would need a shared_ptr to
        // this to be safe, and shared_from_this() does not work in constructors.
    }

    ~ConnectorImpl () {
        boost::system::error_code ec;
        close(ec);
    }

    void close (boost::system::error_code& ec) {
        auto self = this->shared_from_this();
        ec = {};
        mContext.post([self, this, ec]() mutable {
            for (auto&& conPair : mNascentConnections) {
                // Don't want first->close() to accidentally call the handler, which we will go
                // ahead and do ourselves.
                conPair.first->set_open_handler(nullptr);
                conPair.first->set_fail_handler(nullptr);
                conPair.first->close(
                    ::websocketpp::close::status::normal, "Closing nascent connection", ec);
                if (ec) {
                    BOOST_LOG(mLog) << "Error closing WebSocket connection: " << ec.message();
                }
                conPair.second.handler(boost::asio::error::operation_aborted);
            }
            mNascentConnections.clear();
        });
    }

    template <class CompletionToken>
    BOOST_ASIO_INITFN_RESULT_TYPE(CompletionToken, void(boost::system::error_code))
    asyncConnect (MessageQueue& mq,
        const std::string& host, const std::string& service,
        CompletionToken&& token)
    {
        util::AsyncCompletion<
            CompletionToken, void(boost::system::error_code)
        > init { std::forward<CompletionToken>(token) };

        auto uri = std::make_shared<::websocketpp::uri>(false, host, service, "");
        auto handler = init.handler;
        auto self = this->shared_from_this();
        mContext.post([&mq, uri, handler, self, this]() mutable {
            auto ec = boost::system::error_code{};
            auto con = mWsClient.get_connection(uri, ec);
            con->set_open_handler(std::bind(&ConnectorImpl::openHandler, self, _1));
            con->set_fail_handler(std::bind(&ConnectorImpl::openHandler, self, _1));
            if (ec) {
                handler(ec);
                return;
            }
            bool success;
            std::tie(std::ignore, success) = mNascentConnections.insert(
                std::make_pair(con, NascentConnectionData{handler, std::ref(mq)}));
            if (success) {
                mWsClient.connect(con);
            }
            else {
                assert(false);
                handler(boost::asio::error::operation_aborted); // FIXME come up with a real error?
            }
        });

        return init.result.get();
    }

private:
    void openHandler (::websocketpp::connection_hdl hdl) {
        auto ec = boost::system::error_code{};
        auto con = mWsClient.get_con_from_hdl(hdl, ec);
        if (!ec) {
            assert(con);
            auto iter = mNascentConnections.find(con);
            if (iter != mNascentConnections.end()) {
                auto& data = iter->second;
                auto handler = data.handler;
                auto& mq = data.mq.get();
                mNascentConnections.erase(iter);
                // The newly opened connection has handlers which contain shared_ptrs to this.
                // Destroy them as soon as possible.
                mContext.post([con] {
                    con->set_open_handler(nullptr);
                    con->set_fail_handler(nullptr);
                });
                mq.setConnection(con);
                handler(con->get_transport_ec());
            }
            else {
                BOOST_LOG(mLog) << "Open handler could not find nascent connection pointer";
            }
        }
        else {
            BOOST_LOG(mLog) << "Open handler could not get connection pointer: " << ec.message();
        }
    }

    boost::asio::io_service& mContext;
    ::websocketpp::client<Config> mWsClient;

    struct NascentConnectionData {
        std::function<void(boost::system::error_code)> handler;
        std::reference_wrapper<MessageQueue> mq;
    };
    std::map<ConnectionPtr, NascentConnectionData> mNascentConnections;

    mutable boost::log::sources::logger mLog;
};

class Connector : public util::TransparentIoObject<ConnectorImpl> {
public:
    explicit Connector (boost::asio::io_service& ios)
        : util::TransparentIoObject<ConnectorImpl>(ios)
    {}

    Connector (const Connector&) = delete;
    Connector& operator= (const Connector&) = delete;

#if 0
    Connector (Connector&&) = default;
    Connector& operator= (Connector&&) = default;
#endif

    void close () {
        boost::system::error_code ec;
        close(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void close (boost::system::error_code& ec) {
        this->get_service().close(this->get_implementation(), ec);
    }

    template <class... Args, class Indices = util::make_index_sequence_t<sizeof...(Args) - 1>>
    auto asyncConnect (Args&&... args)
//#if !_MSC_VER
        -> decltype(std::declval<Connector>().asyncConnectImpl(
            std::forward_as_tuple(std::forward<Args>(args)...), Indices{}))
//#endif
    {
        static_assert(sizeof...(Args) > 0, "Asynchronous operations need at least one argument");
        // Indices is an index_sequence for every argument except the last argument, which is a
        // completion token.
        return asyncConnectImpl(std::forward_as_tuple(std::forward<Args>(args)...), Indices{});
    }

private:
    template <class Tuple, size_t... NMinusOneIndices>
    auto asyncConnectImpl (Tuple&& t, util::index_sequence<NMinusOneIndices...>&&)
//#if !_MSC_VER
        -> decltype(std::declval<Connector>().get_implementation()->asyncConnect(
            std::get<NMinusOneIndices>(t)...,
            std::declval<Connector>().get_service().transformCompletionToken(
                std::get<std::tuple_size<typename std::decay<Tuple>::type>::value - 1>(t))))
//#endif
    {
        return this->get_implementation()->asyncConnect(
            std::get<NMinusOneIndices>(t)...,
            this->get_service().transformCompletionToken(
                std::get<std::tuple_size<typename std::decay<Tuple>::type>::value - 1>(t)));
    }
    using MessageQueue = ConnectorImpl::MessageQueue;
};

}} // namespace baromesh::websocket

#endif
