#ifndef BAROMESH_WEBSOCKETPOLYSERVER_HPP
#define BAROMESH_WEBSOCKETPOLYSERVER_HPP

#include <util/composed.hpp>
#include <util/iothread.hpp>

#include <rpc/asio/waitmultiplecompleter.hpp>
#include <rpc/asio/server.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <boost/asio/async_result.hpp>
#include <boost/asio/basic_io_object.hpp>
#include <boost/asio/error.hpp>
#include <boost/asio/handler_invoke_hook.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace baromesh {

using Tcp = boost::asio::ip::tcp;
using IoService = boost::asio::io_service;
using namespace std::placeholders;

using WebsocketAcceptor = websocketpp::server<websocketpp::config::asio>;
using WebsocketConnectionPtr = WebsocketAcceptor::connection_ptr;

namespace _ {

typedef void ReceiveHandlerSignature(boost::system::error_code, size_t);
typedef void SendHandlerSignature(boost::system::error_code);

struct WebsocketMessageQueue {
public:
    WebsocketMessageQueue (boost::asio::io_service& ios, boost::log::sources::logger log)
        : mIos(ios)
        , mLog(log)
    {
        mLog.add_attribute("Protocol", boost::log::attributes::constant<std::string>("WSQ"));
    }

    ~WebsocketMessageQueue () {
        mPtr->set_message_handler(nullptr);
        boost::system::error_code ec;
        close(ec);
    }

    // noncopyable
    WebsocketMessageQueue (const WebsocketMessageQueue&) = delete;
    WebsocketMessageQueue& operator= (const WebsocketMessageQueue&) = delete;

    void close () {
        boost::system::error_code ec;
        close(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void close (boost::system::error_code& ec) {
        voidReceives(boost::asio::error::operation_aborted);

        enum CloseCode {
            NORMAL_CLOSURE = 1000
        };
        if (mPtr) {
            mPtr->close(NORMAL_CLOSURE, "See ya bro", ec);
        }
    }

    boost::asio::io_service& get_io_service () {
        return mIos;
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, SendHandlerSignature)
    asyncSend (boost::asio::const_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, SendHandlerSignature
        > init { std::forward<Handler>(handler) };

        assert(mPtr);
        auto ec = mPtr->send(boost::asio::buffer_cast<void const*>(buffer),
                boost::asio::buffer_size(buffer));
        mIos.post(std::bind(init.handler, ec));

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ReceiveHandlerSignature)
    asyncReceive (boost::asio::mutable_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ReceiveHandlerSignature
        > init { std::forward<Handler>(handler) };

        auto ec = mPtr->get_transport_ec();
        if (!ec) {
            auto work = boost::asio::io_service::work{mIos};
            mReceives.emplace(ReceiveData{work, buffer, init.handler});
            postReceives();
        }
        else {
            mIos.post(std::bind(init.handler, ec, 0));
        }

        return init.result.get();
    }

    void setWebsocketConnection (WebsocketConnectionPtr ptr) {
        mPtr = ptr;
        mPtr->set_message_handler(std::bind(&WebsocketMessageQueue::handleMessage, this, _1, _2));
    }

private:
    void handleMessage (websocketpp::connection_hdl, WebsocketAcceptor::connection_type::message_ptr msg) {
        BOOST_LOG(mLog) << "Received " << msg->get_payload().size();
        mInbox.emplace(msg);
        postReceives();
    }

	void postReceives () {
		while (mInbox.size() && mReceives.size()) {
			auto& receive = mReceives.front();
			auto nCopied = boost::asio::buffer_copy(receive.buffer,
				boost::asio::buffer(mInbox.front()->get_payload()));

			auto ec = nCopied == mInbox.front()->get_payload().size()
					  ? boost::system::error_code()
					  : make_error_code(boost::asio::error::message_size);

			auto& ios = receive.work.get_io_service();
			ios.post(std::bind(receive.handler, ec, nCopied));
			mInbox.pop();
			mReceives.pop();
		}
	}

	void voidReceives (boost::system::error_code ec) {
		while (mReceives.size()) {
			auto& receive = mReceives.front();
			auto& ios = receive.work.get_io_service();
			ios.post(std::bind(receive.handler, ec, 0));
			mReceives.pop();
		}
	}

    using ReceiveHandler = std::function<ReceiveHandlerSignature>;
    struct ReceiveData {
        boost::asio::io_service::work work;
        boost::asio::mutable_buffer buffer;
        ReceiveHandler handler;
    };

    boost::asio::io_service& mIos;
    WebsocketConnectionPtr mPtr;
    std::queue<ReceiveData> mReceives;
    std::queue<WebsocketAcceptor::connection_type::message_ptr> mInbox;

    mutable boost::log::sources::logger mLog;
};

using WebsocketServer = rpc::asio::Server<WebsocketMessageQueue>;

#include <boost/asio/yield.hpp>

// TODO refactor to combine this with RunServerOperation in server.hpp?
template <class S, class RequestFunc>
struct RunSubServerOp {
    using RequestId = typename S::RequestId;
    using RequestPair = typename S::RequestPair;

    RunSubServerOp (std::shared_ptr<S> subServer, RequestFunc requestFunc)
        : ios_(subServer->get_io_service())
        , subServer_(subServer)
        , requestFunc_(requestFunc)
    {}

    boost::asio::io_service& ios_;
    std::shared_ptr<S> subServer_;
    RequestFunc requestFunc_;
    boost::log::sources::logger log_;

    boost::system::error_code rc_ = boost::asio::error::operation_aborted;

    std::tuple<boost::system::error_code> result () const {
        return std::make_tuple(rc_);
    }

    template <class Op>
    void operator() (Op&& op, boost::system::error_code ec = {}, RequestPair rp = {}) {
        if (!ec) reenter(op) {
            BOOST_LOG(log_) << "Waiting for connection.";
            yield asyncWaitForConnection(*subServer_, std::move(op));
            BOOST_LOG(log_) << "Received connection.";
            while (1) {
                if (barobo_rpc_Request_Type_DISCONNECT == rp.second.type) {
                    BOOST_LOG(log_) << "disconnecting";
                    // Don't forward the request to mRequestFunc, as we may not be
                    // the last subserver remaining. Our caller can figure out if
                    // we are the last one and emit any last DISCONNECT request as
                    // necessary.
                    yield asyncReply(*subServer_, rp.first, rpc::Status::OK, std::move(op));
                    rc_ = Status::OK;
                    return;
                }
                else {
                    if (barobo_rpc_Request_Type_CONNECT == rp.second.type) {
                        BOOST_LOG(log_) << "connection received";
                    }
                    requestFunc_(rp.first, rp.second);
                    yield subServer_->asyncReceiveRequest(std::move(op));
                }
            }
        }
        else if (ec != boost::asio::error::operation_aborted) {
            BOOST_LOG(log_) << "subserver run interrupted: " << ec.message();
            rc_ = ec;
        }
    }

};

#include <boost/asio/unyield.hpp>

template <class S, class RequestFunc, class Handler>
BOOST_ASIO_INITFN_RESULT_TYPE(Handler, void(boost::system::error_code))
asyncRunSubServer (std::shared_ptr<S> server, RequestFunc&& requestFunc, Handler&& handler) {
    boost::asio::detail::async_result_init<
        Handler, void(boost::system::error_code)
    > init { std::forward<Handler>(handler) };

    using Op = RunSubServerOp<S, RequestFunc>;
    util::composed::makeOperation<Op>(std::move(init.handler),
            server, std::forward<RequestFunc>(requestFunc))();

    return init.result.get();
}

} // namespace _

class WebsocketPolyServerImpl : public std::enable_shared_from_this<WebsocketPolyServerImpl> {
public:
    using SubServer = _::WebsocketServer;

    using RequestId = std::pair<std::string, typename SubServer::RequestId>;
    using RequestPair = std::pair<RequestId, barobo_rpc_Request>;

    typedef void RequestHandlerSignature(boost::system::error_code, RequestPair);
    using RequestHandler = std::function<RequestHandlerSignature>;

    typedef void ReplyHandlerSignature(boost::system::error_code);
    using ReplyHandler = std::function<ReplyHandlerSignature>;

    typedef void BroadcastHandlerSignature(boost::system::error_code);
    using BroadcastHandler = std::function<BroadcastHandlerSignature>;

    explicit WebsocketPolyServerImpl (IoService& ioService)
        : mStrand(ioService)
        , mAcceptor()
    {
        mAcceptor.init_asio(&ioService);
    }

    void destroy () {
        boost::system::error_code ec;
        close(ec);
    }

    void close (boost::system::error_code& ec) {
        mStrand.post(std::bind(&WebsocketPolyServerImpl::closeImpl,
            this->shared_from_this()));
        ec = boost::system::error_code{};
    }

    void init (Tcp::endpoint endpoint, boost::log::sources::logger log) {
        mLogPrototype = mLog = log;
        mLog.add_attribute("Protocol", boost::log::attributes::constant<std::string>("RB-PS"));

        mDesiredEndpoint = endpoint;
        initAcceptor();
    }

    Tcp::endpoint endpoint () {
        // This may be different from mDesiredEndpoint, which may use service (port) "0".
        boost::system::error_code ec;
        auto ep = mAcceptor.get_local_endpoint(ec);
        if(ec) {
            throw boost::system::system_error(ec);
        }
        return ep;
    }

    boost::log::sources::logger log () const {
        return mLog;
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        RequestHandlerSignature)
    asyncReceiveRequest (IoService::work work, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, RequestHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        mStrand.post(std::bind(&WebsocketPolyServerImpl::asyncReceiveRequestImpl,
            this->shared_from_this(), work, realHandler));

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        ReplyHandlerSignature)
    asyncSendReply (IoService::work work, RequestId requestId, barobo_rpc_Reply reply, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ReplyHandlerSignature
        > init { std::forward<Handler>(handler) };

        mStrand.post(std::bind(&WebsocketPolyServerImpl::asyncSendReplyImpl,
            this->shared_from_this(), work, requestId, reply, init.handler));

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        BroadcastHandlerSignature)
    asyncSendBroadcast (IoService::work work, barobo_rpc_Broadcast broadcast, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, BroadcastHandlerSignature
        > init { std::forward<Handler>(handler) };

        mStrand.post(std::bind(&WebsocketPolyServerImpl::asyncSendBroadcastImpl,
            this->shared_from_this(), work, broadcast, init.handler));

        return init.result.get();
    }

private:
    void initAcceptor () {
        if (mAcceptor.is_listening()) {
            boost::system::error_code ec;
            mAcceptor.stop_listening(ec);
            if (ec) {
                BOOST_LOG(mLog) << "Error closing acceptor for reinitialization: " << ec.message();
            }
        }

        mAcceptor.set_open_handler(std::bind(&WebsocketPolyServerImpl::handleAccept,
                this->shared_from_this(), _1));

        auto mkHandler = [this](std::string message) {
            return [this, message](websocketpp::connection_hdl hdl) mutable {
                boost::system::error_code ec;
                auto con = mAcceptor.get_con_from_hdl(hdl);
                if (!ec) {
                    message += " from ";
                    message += con->get_uri()->str();
                }
                BOOST_LOG(mLog) << "websocketpp server handler: " << message;
            };
        };

        auto mkValidateHandler = [this](std::string message) {
            return [this, message](websocketpp::connection_hdl hdl) mutable {
                boost::system::error_code ec;
                auto con = mAcceptor.get_con_from_hdl(hdl);
                if (!ec) {
                    message += " from ";
                    message += con->get_uri()->str();
                }
                BOOST_LOG(mLog) << "websocketpp server handler: " << message;
                return true;
            };
        };

        /*
        mAcceptor.set_close_handler(mkHandler("close"));
        mAcceptor.set_fail_handler(mkHandler("fail"));
        mAcceptor.set_http_handler(mkHandler("http"));
        mAcceptor.set_interrupt_handler(mkHandler("interrupt"));
        mAcceptor.set_validate_handler(mkValidateHandler("validate"));
        */


        mAcceptor.set_reuse_addr(true);
        mAcceptor.listen(mDesiredEndpoint);
        mAcceptor.start_accept();
    }

    void handleAccept (websocketpp::connection_hdl hdl) {
        auto ec = boost::system::error_code{};
        auto con = mAcceptor.get_con_from_hdl(hdl, ec);
        if (!ec) {
            decltype(mSubServers)::iterator iter;
            bool success;
            auto peer = con->get_remote_endpoint();
            auto subServer = std::make_shared<SubServer>(mStrand.get_io_service(), mLogPrototype);
            subServer->messageQueue().setWebsocketConnection(con);
            std::tie(iter, success) = mSubServers.insert(std::make_pair(peer, subServer));
            assert(success);

            BOOST_LOG(mLog) << "inserted subserver for " << peer;

            _::asyncRunSubServer(subServer,
                std::bind(&WebsocketPolyServerImpl::pushRequest, this->shared_from_this(), peer, _1, _2),
                mStrand.wrap(std::bind(&WebsocketPolyServerImpl::handleSubServerFinished,
                    this->shared_from_this(), peer, _1)));
        }
    }

    void closeImpl () {
        boost::system::error_code ec;
        mAcceptor.stop_listening(ec);
        if (ec) {
            BOOST_LOG(mLog) << "Error closing acceptor: " << ec.message();
        }
        for (auto& kv : mSubServers) {
            kv.second->close(ec);
            if (ec) {
                BOOST_LOG(mLog) << "Error closing subserver "
                                << kv.first << ": " << ec.message();
            }
        }
        voidReceives(boost::asio::error::operation_aborted);
    }

    void pushRequest (std::string peer, SubServer::RequestId requestId, barobo_rpc_Request request) {
        mInbox.push(std::make_pair(std::make_pair(peer, requestId), request));
        postReceives();
    }

    void handleSubServerFinished (std::string peer, boost::system::error_code ec) {
        BOOST_LOG(mLog) << "Subserver " << peer << " finished with " << ec.message();
        auto iter = mSubServers.find(peer);
        if (iter != mSubServers.end()) {
            ec = boost::system::error_code{};
            iter->second->close(ec); // ignore error
            mSubServers.erase(iter);
            BOOST_LOG(mLog) << "" << peer << " erased; " << mSubServers.size() << " subservers remaining";
        }
        if (!mSubServers.size()) {
            BOOST_LOG(mLog) << "Emitting disconnect";
            barobo_rpc_Request request;
            request = decltype(request)();
            request.type = barobo_rpc_Request_Type_DISCONNECT;
            pushRequest("", nextRequestId(), request);
        }
    }

    void asyncReceiveRequestImpl (IoService::work work, RequestHandler handler) {
        mReceives.emplace(std::piecewise_construct,
            std::forward_as_tuple(work), std::forward_as_tuple(handler));
        postReceives();
    }

    void asyncSendReplyImpl (IoService::work work, RequestId requestId, barobo_rpc_Reply reply, ReplyHandler handler) {
        // If no subserver exists for this request ID, or if there is an error
        // replying to the subserver, ignore this attempt at a reply. We don't
        // want to post an error, because the situation is similar to a remote
        // client ignoring spurious, or expired, replies. The WebsocketPolyServer is
        // still functional.
        auto iter = mSubServers.find(requestId.first);
        if (iter != mSubServers.end()) {
            auto log = mLog;
            iter->second->asyncSendReply(requestId.second, reply,
                [log] (boost::system::error_code ec) mutable {
                    if (ec) {
                        // Replies cannot fail
                        BOOST_LOG(log) << "ignoring subserver reply failure: " << ec.message();
                    }
                });
        }
        else if (requestId.first == "") {
            BOOST_LOG(mLog) << "Reply to inaddr_any endpoint in polyserver, probably to our DISCONNECT, ignoring";
        }
        auto& ios = work.get_io_service();
        ios.post(std::bind(handler, boost::system::error_code()));
    }

    void asyncSendBroadcastImpl (IoService::work work, barobo_rpc_Broadcast broadcast, BroadcastHandler handler) {
        auto& ios = work.get_io_service();
        rpc::asio::WaitMultipleCompleter<BroadcastHandler> completer { ios, handler };
        for (auto& kv : mSubServers) {
            BOOST_LOG(mLog) << "Broadcasting to " << kv.first;
            kv.second->asyncSendBroadcast(broadcast, completer);
        }
    }

    void postReceives () {
        while (mInbox.size() && mReceives.size()) {
            auto request = mInbox.front();
            auto op = mReceives.front();
            auto& ios = op.first.get_io_service();
            auto& handler = op.second;

            mInbox.pop();
            mReceives.pop();

            ios.post(std::bind(handler, boost::system::error_code(), request));
        }
    }

    void voidReceives (boost::system::error_code ec) {
        while (mReceives.size()) {
            auto op = mReceives.front();
            auto& ios = op.first.get_io_service();
            auto& handler = op.second;

            mReceives.pop();

            ios.post(std::bind(handler, ec, RequestPair()));
        }
    }

    SubServer::RequestId nextRequestId () {
        return mNextRequestId++;
    }

    IoService::strand mStrand;

    SubServer::RequestId mNextRequestId = 0;

    Tcp::endpoint mDesiredEndpoint;
    WebsocketAcceptor mAcceptor;
    std::map<std::string, std::shared_ptr<SubServer>> mSubServers;

    std::queue<std::pair<RequestId, barobo_rpc_Request>> mInbox;
    std::queue<std::pair<IoService::work, RequestHandler>> mReceives;

    boost::log::sources::logger mLogPrototype;
    mutable boost::log::sources::logger mLog;
};

template <class Impl = WebsocketPolyServerImpl>
class WebsocketPolyServerService : public IoService::service {
public:
    static IoService::id id;

    using RequestId = typename Impl::RequestId;
    using RequestPair = typename Impl::RequestPair;

    using RequestHandlerSignature = typename Impl::RequestHandlerSignature;
    using RequestHandler = typename Impl::RequestHandler;

    explicit WebsocketPolyServerService (IoService& ioService)
        : IoService::service(ioService)
    {}

    ~WebsocketPolyServerService () {
        mIoThread.join();
    }

    using implementation_type = std::shared_ptr<Impl>;

    void construct (implementation_type& impl) {
        impl.reset(new Impl(mIoThread.context()));
    }

    void destroy (implementation_type& impl) {
        impl->destroy();
        impl.reset();
    }

    void close (implementation_type& impl, boost::system::error_code& ec) {
        impl->close(ec);
    }

    void init (implementation_type& impl, Tcp::endpoint endpoint, boost::log::sources::logger log) {
        impl->init(endpoint, log);
    }

    Tcp::endpoint endpoint (const implementation_type& impl) const {
        return impl->endpoint();
    }

    boost::log::sources::logger log (const implementation_type& impl) const {
        return impl->log();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        void(boost::system::error_code, RequestPair))
    asyncReceiveRequest (implementation_type& impl, Handler&& handler) {
        IoService::work work { this->get_io_service() };
        return impl->asyncReceiveRequest(work, std::forward<Handler>(handler));
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        void(boost::system::error_code))
    asyncSendReply (implementation_type& impl,
            RequestId requestId, barobo_rpc_Reply reply, Handler&& handler) {
        IoService::work work { this->get_io_service() };
        return impl->asyncSendReply(work, requestId, reply, std::forward<Handler>(handler));
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        void(boost::system::error_code))
    asyncSendBroadcast (implementation_type& impl,
            barobo_rpc_Broadcast broadcast, Handler&& handler) {
        IoService::work work { this->get_io_service() };
        return impl->asyncSendBroadcast(work, broadcast, std::forward<Handler>(handler));
    }

private:
    void shutdown_service () {}
    util::IoThread mIoThread;
};

template <class Impl>
boost::asio::io_service::id WebsocketPolyServerService<Impl>::id;

template <class Service = WebsocketPolyServerService<>>
class BasicWebsocketPolyServer : public boost::asio::basic_io_object<Service> {
public:
    using RequestId = typename Service::RequestId;
    using RequestPair = typename Service::RequestPair;

    using RequestHandlerSignature = typename Service::RequestHandlerSignature;
    using RequestHandler = typename Service::RequestHandler;

    BasicWebsocketPolyServer (IoService& ioService, Tcp::endpoint endpoint, boost::log::sources::logger log)
        : boost::asio::basic_io_object<Service>(ioService)
    {
        this->get_service().init(this->get_implementation(), endpoint, log);
    }

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

    Tcp::endpoint endpoint () const {
        return this->get_service().endpoint(this->get_implementation());
    }

    boost::log::sources::logger log () const {
        return this->get_service().log(this->get_implementation());
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        void(boost::system::error_code, RequestPair))
    asyncReceiveRequest (Handler&& handler) {
        return this->get_service().asyncReceiveRequest(this->get_implementation(),
            std::forward<Handler>(handler));
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        void(boost::system::error_code))
    asyncSendReply (RequestId requestId, barobo_rpc_Reply reply, Handler&& handler) {
        return this->get_service().asyncSendReply(this->get_implementation(),
            requestId, reply, std::forward<Handler>(handler));
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler,
        void(boost::system::error_code))
    asyncSendBroadcast (barobo_rpc_Broadcast broadcast, Handler&& handler) {
        return this->get_service().asyncSendBroadcast(this->get_implementation(),
            broadcast, std::forward<Handler>(handler));
    }
};

using WebsocketPolyServer = BasicWebsocketPolyServer<>;

} // namespace baromesh

#endif
