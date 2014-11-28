#ifndef BAROMESH_DAEMON_BASICDONGLE_HPP
#define BAROMESH_DAEMON_BASICDONGLE_HPP

#include "gen-dongle.pb.hpp"

#include "baromesh/system_error.hpp"

#include "rpc/asio/client.hpp"

#include <boost/asio/spawn.hpp>

#include <functional>
#include <memory>
#include <queue>
#include <mutex>
#include <vector>

namespace baromesh {

using SendHandlerSignature = void(boost::system::error_code);
using ReceiveHandlerSignature = void(boost::system::error_code, size_t);
using ReceiveHandler = std::function<ReceiveHandlerSignature>;

using namespace std::placeholders;

template <class RpcClient>
class DongleImpl : public std::enable_shared_from_this<DongleImpl<RpcClient>> {
    struct ReceiveData {
        ReceiveData (boost::log::sources::logger lg) : log(lg) {}
        std::queue<std::pair<boost::asio::mutable_buffer, ReceiveHandler>> ops;
        std::queue<std::vector<uint8_t>> inbox;
        boost::log::sources::logger log;
    };

    using MethodIn = rpc::MethodIn<barobo::Dongle>;
    using MethodResult = rpc::MethodResult<barobo::Dongle>;
    using Broadcast = rpc::Broadcast<barobo::Dongle>;

public:
    DongleImpl (boost::asio::io_service& ios, boost::log::sources::logger log)
        : mClient(ios, log)
        , mStrand(mClient.get_io_service())
        , mLog(log)
    {}

    void cancel (boost::system::error_code& ec) {
        voidHandlers(boost::asio::error::operation_aborted);
        mClient.cancel(ec);
    }

    void cancelMessageQueue (std::string serialId) {
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        auto iter = mReceiveData.find(serialId);
        if (iter != mReceiveData.end()) {
            auto& data = iter->second;
            // FIXME code duplication with voidHandlers
            while (data.ops.size()) {
                auto op = data.ops.front();
                auto& handler = op.second;
                data.ops.pop();
                mClient.get_io_service().post(
                    std::bind(handler, boost::asio::error::operation_aborted, 0));
            }
        }
    }

    RpcClient& client () { return mClient; }

    bool registerMessageQueue (std::string serialId, boost::log::sources::logger log) {
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        return mReceiveData.insert(std::make_pair(serialId, ReceiveData(log))).second;
    }

    bool unregisterMessageQueue (std::string serialId) {
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        return !!mReceiveData.erase(serialId);
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, SendHandlerSignature)
    asyncSendTo (std::string serialId, boost::asio::const_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, SendHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        assert(4 == serialId.size());
        MethodIn::transmitUnicast args = decltype(args)();

        strncpy(args.serialId.value, serialId.data(), 4);
        args.serialId.value[4] = 0;

        auto& payload = args.payload.value;
        payload.size = boost::asio::buffer_copy(
            boost::asio::buffer(payload.bytes),
            buffer);
        assert(payload.size == boost::asio::buffer_size(buffer));

        auto self = this->shared_from_this();
        asyncFire(mClient, args, std::chrono::seconds(60),
            [self, this, realHandler] (boost::system::error_code ec, MethodResult::transmitUnicast) {
                mClient.get_io_service().post(std::bind(realHandler, ec));
            });

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ReceiveHandlerSignature)
    asyncReceiveFrom (std::string serialId, boost::asio::mutable_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ReceiveHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        //BOOST_LOG(mLog) << "Shitting on " << serialId << "'s receive request";
        //mClient.get_io_service().post(std::bind(realHandler, Status::DONGLE_NOT_FOUND, 0));
        auto self = this->shared_from_this();
        mStrand.post([self, this, serialId, buffer, realHandler] () {
            {
                std::lock_guard<std::mutex> lock { mReceiveDataMutex };
                auto iter = mReceiveData.find(serialId);
                assert(iter != mReceiveData.end());
                iter->second.ops.push(std::make_pair(buffer, realHandler));
            }
            postReceives();
            startReceiveCoroutine();
        });

        return init.result.get();
    }

    template <class B>
    void broadcast (B&& args) {
        onBroadcast(std::forward<B>(args));
    }

    void onBroadcast (Broadcast::receiveUnicast broadcast) {
        using std::get;
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        auto serialId = std::string(broadcast.serialId.value);
        auto iter = mReceiveData.find(serialId);
        if (iter != mReceiveData.end()) {
            BOOST_LOG(mLog) << "received message destined for " << serialId;
            auto& inbox = iter->second.inbox;
            auto& payload = broadcast.payload.value;
            inbox.push(std::vector<uint8_t>(payload.bytes, payload.bytes + payload.size));
        }
        else {
            BOOST_LOG(mLog) << "discarding message destined for unregistered serial ID "
                            << serialId;
        }
    }

private:
    void postReceives () {
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        for (auto& kv : mReceiveData) {
            auto& data = kv.second;
            BOOST_LOG(data.log) << "posting ready receive operations (ops: "
                                << data.ops.size() << ", inbox: " << data.inbox.size() << ")";
            while (data.inbox.size() && data.ops.size()) {
                auto op = data.ops.front();
                data.ops.pop();

                auto& buffer = op.first;
                auto& handler = op.second;

                auto nCopied = boost::asio::buffer_copy(
                    buffer,
                    boost::asio::buffer(data.inbox.front()));

                auto ec = nCopied == data.inbox.front().size()
                      ? boost::system::error_code()
                      : make_error_code(boost::asio::error::message_size);
                data.inbox.pop();

                BOOST_LOG(data.log) << "copied " << nCopied << " bytes";
                mClient.get_io_service().post(std::bind(handler, ec, nCopied));
            }
        }
    }

    void startReceiveCoroutine () {
        if (mReceiveCoroutineRunning) {
            return;
        }
        mReceiveCoroutineRunning = true;
        boost::asio::spawn(mStrand, std::bind(&DongleImpl::receiveCoroutine,
            this->shared_from_this(), _1));
    }

    bool thereArePendingOperations () {
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        using KeyValue = typename decltype(mReceiveData)::value_type;
        return std::any_of(mReceiveData.begin(), mReceiveData.end(),
            [] (KeyValue& kv) {
                auto& data = kv.second;
                return !!data.ops.size();
            });
    }

    void receiveCoroutine (boost::asio::yield_context yield) {
        try {
            BOOST_LOG(mLog) << "receive coroutine starting";
            rpc::ComponentBroadcastUnion<barobo::Dongle> argument;
            while (thereArePendingOperations()) {
                auto broadcast = mClient.asyncReceiveBroadcast(yield);
                auto status = decodeBroadcastPayload(argument, broadcast.id, broadcast.payload);
                if (!hasError(status)) {
                    BOOST_LOG(mLog) << "received broadcast";
                    status = invokeBroadcast(*this, argument, broadcast.id);
                    postReceives();
                }
                if (hasError(status)) {
                    throw rpc::Error(status);
                }
            }
        }
        catch (boost::system::system_error& e) {
            BOOST_LOG(mLog) << "Error in Dongle receive coroutine: " << e.what();
            voidHandlers(e.code());
        }
        mReceiveCoroutineRunning = false;
    }

    void voidHandlers (boost::system::error_code ec) {
        using std::get;
        std::lock_guard<std::mutex> lock { mReceiveDataMutex };
        for (auto& kv : mReceiveData) {
            auto& data = kv.second;
            while (data.ops.size()) {
                auto op = data.ops.front();
                data.ops.pop();
                auto& handler = op.second;
                mClient.get_io_service().post(std::bind(handler, ec, 0));
            }
        }
    }

    bool mReceiveCoroutineRunning = false;

    RpcClient mClient;
    boost::asio::io_service::strand mStrand;

    std::map<std::string, ReceiveData> mReceiveData;
    std::mutex mReceiveDataMutex;

    mutable boost::log::sources::logger mLog;
};

template <class D>
class DongleMessageQueue {
public:
    DongleMessageQueue (boost::asio::io_service& ios, boost::log::sources::logger log)
        : mIos(ios)
        , mLog(log)
    {
        mLog.add_attribute("Protocol", boost::log::attributes::constant<std::string>("DMQ"));
    }

    ~DongleMessageQueue () {
        boost::system::error_code ec;
        cancel(ec);
        auto dongle = mDongle.lock();
        if (dongle) {
            auto success = dongle->unregisterMessageQueue(mSerialId);
            assert(success);
            (void)success;
        }
    }

    // noncopyable
    DongleMessageQueue (const DongleMessageQueue&) = delete;
    DongleMessageQueue& operator= (const DongleMessageQueue&) = delete;

    void cancel () {
        boost::system::error_code ec;
        cancel(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void cancel (boost::system::error_code&) {
        auto dongle = mDongle.lock();
        if (dongle) {
            dongle->cancelMessageQueue(mSerialId);
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
        auto& realHandler = init.handler;

        auto dongle = mDongle.lock();
        if (dongle) {
            dongle->asyncSendTo(mSerialId, buffer, realHandler);
        }
        else {
            mIos.post(std::bind(realHandler, Status::DONGLE_NOT_FOUND));
        }

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ReceiveHandlerSignature)
    asyncReceive (boost::asio::mutable_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ReceiveHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        auto dongle = mDongle.lock();
        if (dongle) {
            dongle->asyncReceiveFrom(mSerialId, buffer, realHandler);
        }
        else {
            mIos.post(std::bind(realHandler, Status::DONGLE_NOT_FOUND, 0));
        }

        return init.result.get();
    }

    void setRoute (D& dongle, std::string serialId) {
        mDongle = dongle.impl();
        auto d = mDongle.lock();
        assert(d);
        auto success = d->registerMessageQueue(serialId, mLog);
        assert(success);
        (void)success;

        mSerialId = serialId;
    }

private:
    std::weak_ptr<typename D::Impl> mDongle;
    std::string mSerialId;
    boost::asio::io_service& mIos;

    mutable boost::log::sources::logger mLog;
};
    
template <class C>
class BasicDongle {
public:
    using Client = C;
    using Impl = DongleImpl<Client>;
    using MessageQueue = DongleMessageQueue<BasicDongle>;
    friend MessageQueue;

    template <class... Args>
    explicit BasicDongle (Args&&... args)
        : mImpl(std::make_shared<Impl>(std::forward<Args>(args)...))
    {}

    ~BasicDongle () {
        boost::system::error_code ec;
        cancel(ec);
    }

    void cancel () {
        boost::system::error_code ec;
        cancel(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void cancel (boost::system::error_code& ec) {
        mImpl->cancel(ec);
    }

    // noncopyable
    BasicDongle (const BasicDongle&) = delete;
    BasicDongle& operator= (const BasicDongle&) = delete;

    Client& client () { return mImpl->client(); }
    boost::asio::io_service& get_io_service () { return client().get_io_service(); }

private:
    std::shared_ptr<Impl> impl () {
        return mImpl;
    }

    std::shared_ptr<Impl> mImpl;
};

} // namespace baromesh

#endif