#ifndef BAROMESH_DAEMON_BASICDONGLE_HPP
#define BAROMESH_DAEMON_BASICDONGLE_HPP

#include "gen-dongle.pb.hpp"

#include "baromesh/system_error.hpp"

#include "rpc/asio/client.hpp"

#include "util/benchmarkedlock.hpp"

#include <functional>
#include <memory>
#include <queue>
#include <mutex>
#include <sstream>
#include <vector>

#include <cstring>

namespace baromesh {

// This *HandlerSignature typedef is all over the place in our libraries, but
// its syntax represents a portability challenge.
// using T = void(boost::system::error_code) results in a parse error in VS 2013
// using T = void(mutable boost::system::error_code) results in a parse error in gcc
// using T = void(const boost::system::error_code&) results in a run-time error at sfp/asio/messagequeue.hpp:219
// typedef void T(boost::system::error_code) works
typedef void SendHandlerSignature(boost::system::error_code);
typedef void ReceiveHandlerSignature(boost::system::error_code, size_t);
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

    void close (boost::system::error_code& ec) {
        mClient.close(ec);
    }

    RpcClient& client () { return mClient; }

    // Tell the Dongle object to maintain an incoming messages buffer for the
    // given serial ID, and store any messages destined thereto therein (by
    // default, the Dongle object will ignore any messages destined for a
    // "closed" serial ID). Return false if the buffer already exists, true
    // otherwise.
    bool openMessageQueue (std::string serialId, boost::log::sources::logger log) {
        auto lock = util::BenchmarkedLock{mReceiveDataMutex};
        return mUnicastReceiveData.insert(std::make_pair(serialId, ReceiveData(log))).second;
    }

    // Remove the incoming messages buffer for the given serial ID, if one exists.
    void closeMessageQueue (std::string serialId) {
        auto lock = util::BenchmarkedLock{mReceiveDataMutex};
        auto iter = mUnicastReceiveData.find(serialId);
        if (iter != mUnicastReceiveData.end()) {
            auto& data = iter->second;
            // FIXME code duplication with voidHandlers
            while (data.ops.size()) {
                auto op = data.ops.front();
                auto& handler = op.second;
                data.ops.pop();
                mClient.get_io_service().post(
                    std::bind(handler, boost::asio::error::operation_aborted, 0));
            }
            mUnicastReceiveData.erase(iter);
        }
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, SendHandlerSignature)
    asyncTransmitUnicast (std::string serialId, boost::asio::const_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, SendHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        if (4 != serialId.size()) {
            throw boost::system::system_error(Status::INVALID_SERIALID);
        }

        MethodIn::transmitUnicast args = decltype(args)();

        std::strncpy(args.serialId.value, serialId.data(), 4);
        args.serialId.value[4] = 0;

        auto& payload = args.payload.value;
        payload.size = boost::asio::buffer_copy(
            boost::asio::buffer(payload.bytes),
            buffer);
        assert(payload.size == boost::asio::buffer_size(buffer));

        auto self = this->shared_from_this();
        asyncFire(mClient, args, std::chrono::seconds(60),
            [self, this, realHandler] (boost::system::error_code ec, MethodResult::transmitUnicast) {
                this->mClient.get_io_service().post(std::bind(realHandler, ec));
            });

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ReceiveHandlerSignature)
    asyncReceiveUnicast (std::string serialId, boost::asio::mutable_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ReceiveHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        auto self = this->shared_from_this();
        mStrand.post([self, this, serialId, buffer, realHandler] () {
            {
                auto lock = util::BenchmarkedLock{this->mReceiveDataMutex};
                auto iter = this->mUnicastReceiveData.find(serialId);
                if (this->mUnicastReceiveData.end() == iter) {
                    this->mClient.get_io_service().post(std::bind(realHandler, Status::UNREGISTERED_SERIALID, 0));
                }
                else {
                    BOOST_LOG(this->mLog) << "Pushing a receive handler for " << serialId;
                    iter->second.ops.push(std::make_pair(buffer, realHandler));
                }
            }
            this->postReceives();
            this->startReceivePump();
        });

        return init.result.get();
    }

    template <class Stringclass Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, SendHandlerSignature)
    asyncTransmitRadioBroadcast (boost::asio::const_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, SendHandlerSignature
        > init { std::forward<Handler>(handler) };

        MethodIn::transmitRadioBroadcast args = decltype(args)();

        auto& payload = args.payload.value;
        payload.size = boost::asio::buffer_copy(
            boost::asio::buffer(payload.bytes),
            buffer);
        assert(payload.size == boost::asio::buffer_size(buffer));

        auto self = this->shared_from_this();
        asyncFire(mClient, args, std::chrono::seconds(60),
            [self, this, realHandler] (boost::system::error_code ec, MethodResult::transmitRadioBroadcast) {
                this->mClient.get_io_service().post(std::bind(realHandler, ec));
            });

        return init.result.get();
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, ReceiveHandlerSignature)
    asyncReceiveRadioBroadcast (boost::asio::mutable_buffer buffer, Handler&& handler) {
        boost::asio::detail::async_result_init<
            Handler, ReceiveHandlerSignature
        > init { std::forward<Handler>(handler) };
        auto& realHandler = init.handler;

        auto self = this->shared_from_this();
        mStrand.post([self, this, buffer, realHandler] () {
            {
                auto lock = util::BenchmarkedLock{this->mReceiveDataMutex};
                BOOST_LOG(this->mLog) << "Pushing a radio broadcast receive handler";
                this->mRadioBroadcastReceiveData.ops.push(std::make_pair(buffer, realHandler));
            }
            this->postReceives();
            this->startReceivePump();
        });

        return init.result.get();
    }

    void onBroadcast (Broadcast::receiveUnicast broadcast) {
        {
            auto lock = util::BenchmarkedLock{mReceiveDataMutex};
            auto serialId = std::string(broadcast.serialId.value);
            auto iter = mUnicastReceiveData.find(serialId);
            if (iter != mUnicastReceiveData.end()) {
                BOOST_LOG(mLog) << "received message from " << serialId;
                auto& inbox = iter->second.inbox;
                auto& payload = broadcast.payload.value;
                inbox.push(std::vector<uint8_t>(payload.bytes, payload.bytes + payload.size));
            }
            else {
                BOOST_LOG(mLog) << "discarding message from unregistered serial ID "
                                << serialId;
            }
        }
        postReceives();
    }

    void onBroadcast (Broadcast::receiveRadioBroadcast broadcast) {
        {
#if 0
            std::stringstream ss {};
            ss << broadcast.serialId.value << " is " << (broadcast.RadioBroadcast.poweredOn ? "ON" : "OFF");

            if (broadcast.RadioBroadcast.has_ledColor) {
                ss << ", and is connected to the dongle (color: "
                   << std::setfill('0') << std::setw(6) << std::hex << broadcast.RadioBroadcast.ledColor << ")";
            }
            BOOST_LOG(mLog) << "robot event: " << ss.str();
#endif
            auto lock = util::BenchmarkedLock{mReceiveDataMutex};
            auto& inbox = mRadioBroadcastReceiveData.inbox;
            auto& payload = broadcast.payload.value;
            inbox.push(std::vector<uint8_t>(payload.bytes, payloads.bytes + payload.size));
        }
        postReceives();

    }

private:
    void postReceives () {
        auto lock = util::BenchmarkedLock{mReceiveDataMutex};

        static auto postReceiveImpl = [=] (ReceiveData& data) {
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
        };

        for (auto& kv : mUnicastReceiveData) {
            postReceivesImpl(kv.second);
        }
        postReceivesImpl(mRadioBroadcastReceiveData);
    }

    void startReceivePump () {
        if (mReceivePumpRunning) {
            return;
        }
        mReceivePumpRunning = true;
        receivePump();
    }

    bool thereArePendingOperations () {
        auto lock = util::BenchmarkedLock{mReceiveDataMutex};
        using KeyValue = typename decltype(mUnicastReceiveData)::value_type;
        return !!mRadioBroadcastReceiveData.ops.size() ||
               std::any_of(mUnicastReceiveData.begin(), mUnicastReceiveData.end(),
            [] (KeyValue& kv) {
                auto& data = kv.second;
                return !!data.ops.size();
            });
    }

    // FIXME the receive pump could probably be replaced with a call to
    // asyncRunClient.
    void receivePump () {
        if (thereArePendingOperations()) {
            //BOOST_LOG(mLog) << "Receive pump: calling asyncReceiveBroadcast";
            mClient.asyncReceiveBroadcast(mStrand.wrap(
                std::bind(&DongleImpl::handleReceive,
                    this->shared_from_this(), _1, _2)));
        }
    }

    void handleReceive (boost::system::error_code ec, barobo_rpc_Broadcast broadcast) {
        try {
            if (ec) {
                BOOST_LOG(mLog) << "Dongle receive broadcast error: " << ec.message();
                throw boost::system::system_error(ec);
            }

            rpc::BroadcastUnion<barobo::Dongle> b;
            rpc::Status status;
            b.invoke(*this, broadcast.id, broadcast.payload, status);
            if (hasError(status)) {
                ec = status;
                BOOST_LOG(mLog) << "Dongle broadcast invocation error: " << ec.message();
                throw boost::system::system_error(ec);
            }

            receivePump();
        }
        catch (boost::system::system_error& e) {
            voidHandlers(e.code());
            mReceivePumpRunning = false;
        }
    }

    void voidHandlers (boost::system::error_code ec) {
        auto lock = util::BenchmarkedLock{mReceiveDataMutex};
        static auto voidHandlersImpl = [=] (ReceiveData& data) {
            while (data.ops.size()) {
                auto op = data.ops.front();
                data.ops.pop();
                auto& handler = op.second;
                mClient.get_io_service().post(std::bind(handler, ec, 0));
            }
        };

        for (auto& kv : mUnicastReceiveData) {
            voidHandlersImpl(kv.second);
        }
        voidHandlersImpl(mRadioBroadcastReceiveData);
    }

    bool mReceivePumpRunning = false;

    RpcClient mClient;
    boost::asio::io_service::strand mStrand;

    std::map<std::string, ReceiveData> mUnicastReceiveData;
    ReceiveData mRadioBroadcastReceiveData;
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
        close(ec);
    }

    // noncopyable
    DongleMessageQueue (const DongleMessageQueue&) = delete;
    DongleMessageQueue& operator= (const DongleMessageQueue&) = delete;

    void close () {
        boost::system::error_code ec;
        close(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void close (boost::system::error_code&) {
        BOOST_LOG(mLog) << "Closing message queue for " << mSerialId;
        auto dongle = mDongle.lock();
        if (dongle) {
            dongle->closeMessageQueue(mSerialId);
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
            BOOST_LOG(mLog) << "Calling dongle->asyncReceiveFrom(" << mSerialId << ")";
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
        auto success = d->openMessageQueue(serialId, mLog);
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
        if (mImpl) {
            boost::system::error_code ec;
            close(ec);
        }
    }

    friend void swap (BasicDongle& lhs, BasicDongle& rhs) BOOST_NOEXCEPT {
        using std::swap;
        swap(lhs.mImpl, rhs.mImpl);
    }

    BasicDongle (BasicDongle&& other) BOOST_NOEXCEPT {
        swap(*this, other);
    }

    BasicDongle& operator= (BasicDongle&& other) BOOST_NOEXCEPT {
        swap(*this, other);
        return *this;
    }

    void close () {
        boost::system::error_code ec;
        close(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void close (boost::system::error_code& ec) {
        mImpl->close(ec);
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
