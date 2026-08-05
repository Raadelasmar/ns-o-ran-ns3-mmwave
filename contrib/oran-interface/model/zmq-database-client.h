#ifndef ZMQ_DATABASE_CLIENT_H
#define ZMQ_DATABASE_CLIENT_H

#include <iostream>
#include <string>
#include <zmq.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace ns3 {

class ZmqDatabaseClient {
public:
    ZmqDatabaseClient(const std::string& endpoint = "tcp://localhost:5555") 
        : m_endpoint(endpoint), m_connected(false) {}

    ~ZmqDatabaseClient() {
        Disconnect();
    }

    void Connect() {
        if (!m_connected) {
            m_context = zmq::context_t(1);
            m_socket = zmq::socket_t(m_context, ZMQ_REQ);
            m_socket.connect(m_endpoint);
            m_connected = true;
            std::cout << "[ZmqDatabaseClient] Connected to Python Gym at " << m_endpoint << std::endl;
        }
    }

    void Disconnect() {
        if (m_connected) {
            m_socket.close();
            m_context.close();
            m_connected = false;
            std::cout << "[ZmqDatabaseClient] Disconnected." << std::endl;
        }
    }

    /**
     * Sends the cell KPI JSON payload over ZeroMQ and blocks until 
     * the Python environment responds with the unified control action payload.
     */
    json StepSync(const json& kpi_payload) {
        if (!m_connected) {
            std::cerr << "[ZmqDatabaseClient] Error: Socket not connected!" << std::endl;
            return json({});
        }

        // 1. Serialize and send KPI observation to Python
        std::string request_str = kpi_payload.dump();
        zmq::message_t request(request_str.size());
        memcpy(request.data(), request_str.c_str(), request_str.size());
        m_socket.send(request, zmq::send_flags::none);

        // 2. Block and wait for Python action response
        zmq::message_t reply;
        m_socket.recv(reply, zmq::recv_flags::none);

        std::string reply_str(static_cast<char*>(reply.data()), reply.size());
        return json::parse(reply_str);
    }

private:
    std::string m_endpoint;
    bool m_connected;
    zmq::context_t m_context;
    zmq::socket_t m_socket;
};

} // namespace ns3

#endif // ZMQ_DATABASE_CLIENT_H
