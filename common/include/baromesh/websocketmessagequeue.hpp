#ifndef BAROMESH_WEBSOCKETMESSAGEQUEUE_HPP
#define BAROMESH_WEBSOCKETMESSAGEQUEUE_HPP

#include <util/asio/asynccompletion.hpp>

#include <websocketpp/connection.hpp>
#include <websocketpp/close.hpp>

#include <boost/asio/io_service.hpp>

#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/record_ostream.hpp>

#include <functional>
#include <queue>

using namespace std::placeholders;

namespace baromesh { namespace websocket {

typedef void ReceiveHandlerSignature(boost::system::error_code, size_t);
typedef void SendHandlerSignature(boost::system::error_code);

template <class Config>
struct MessageQueue {
public:
    using Connection = websocketpp::connection<Config>;
    using ConnectionPtr = typename Connection::ptr;
    using MessagePtr = typename Connection::message_ptr;

    MessageQueue (boost::asio::io_service& ios, boost::log::sources::logger log)
        : mIos(ios)
        , mLog(log)
    {
        mLog.add_attribute("Protocol", boost::log::attributes::constant<std::string>("WSQ"));
    }

    ~MessageQueue () {
        if (mPtr) {
            mPtr->set_message_handler(nullptr);
            mPtr->set_close_handler(nullptr);
        }
        boost::system::error_code ec;
        close(ec);
    }

    // noncopyable
    MessageQueue (const MessageQueue&) = delete;
    MessageQueue& operator= (const MessageQueue&) = delete;

    void close () {
        boost::system::error_code ec;
        close(ec);
        if (ec) {
            throw boost::system::system_error(ec);
        }
    }

    void close (boost::system::error_code& ec) {
        voidReceives(boost::asio::error::operation_aborted);

        if (mPtr) {
            mPtr->close(websocketpp::close::status::normal, "See ya bro", ec);
        }
    }

    boost::asio::io_service& get_io_service () {
        return mIos;
    }

    template <class Handler>
    BOOST_ASIO_INITFN_RESULT_TYPE(Handler, SendHandlerSignature)
    asyncSend (boost::asio::const_buffer buffer, Handler&& handler) {
        util::AsyncCompletion<
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
        util::AsyncCompletion<
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

    void setConnection (ConnectionPtr ptr) {
        mPtr = ptr;
        mPtr->set_message_handler(std::bind(&MessageQueue::handleMessage, this, _1, _2));
        mPtr->set_close_handler(std::bind(&MessageQueue::handleClose, this, _1));
    }

private:
    void handleMessage (websocketpp::connection_hdl, MessagePtr msg) {
        BOOST_LOG(mLog) << "Received " << msg->get_payload().size();
        mInbox.emplace(msg);
        postReceives();
    }

    void handleClose (websocketpp::connection_hdl) {
        voidReceives(mPtr->get_transport_ec());
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
    ConnectionPtr mPtr;
    std::queue<ReceiveData> mReceives;
    std::queue<MessagePtr> mInbox;

    mutable boost::log::sources::logger mLog;
};

}} // namespace baromesh::websocket

#endif
