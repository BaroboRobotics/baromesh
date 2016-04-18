#ifndef BAROMESH_WEBSOCKETCONNECTORCONFIG_HPP
#define BAROMESH_WEBSOCKETCONNECTORCONFIG_HPP

#include <baromesh/websocketlogger.hpp>

#include <websocketpp/config/asio_no_tls_client.hpp>

namespace baromesh { namespace websocket {

// Client config with asio transport, TLS disabled, and Boost.Log logging
struct ConnectorConfig : public ::websocketpp::config::asio_client {
    typedef ConnectorConfig type;
    typedef ::websocketpp::config::asio_client base;

    typedef Logger<base::concurrency_type, ::websocketpp::log::elevel> elog_type;
    typedef Logger<base::concurrency_type, ::websocketpp::log::alevel> alog_type;

    struct transport_config : public base::transport_config {
        typedef type::alog_type alog_type;
        typedef type::elog_type elog_type;
    };

    typedef ::websocketpp::transport::asio::endpoint<transport_config> transport_type;
};

}} // namespace baromesh::websocket

#endif // BAROMESH_WEBSOCKETCONNECTORCONFIG_HPP
