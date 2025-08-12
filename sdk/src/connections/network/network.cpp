/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "network.h"

#include <functional>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <iostream>

std::unique_ptr<zmq::socket_t> frame_socket;
std::unique_ptr<zmq::context_t> zmq_context;
std::string zmq_ip;

static std::atomic<bool> running[MAX_CAMERA_NUM];
static std::atomic<bool> threadObj_started{false};

#define RX_BUFFER_BYTES (20996420)
#define MAX_RETRY_CNT 3

enum protocols { PROTOCOL_0 = 0, PROTOCOL_COUNT };

using namespace google::protobuf::io;
using namespace payload;
using namespace std;

struct clientData {
    bool hasFragments;
    std::vector<char> data;
};

int nBytes = 0;          /*no of bytes sent*/
int recv_data_error = 0; /*flag for recv data*/
char server_msg[] = "Connection Allowed";

/*Declare static members*/
std::vector<std::unique_ptr<zmq::socket_t>> Network::command_socket;
std::vector<std::unique_ptr<zmq::socket_t>> Network::monitor_sockets;
std::vector<std::unique_ptr<zmq::context_t>> Network::contexts;
ClientRequest Network::send_buff[MAX_CAMERA_NUM];
ServerResponse Network::recv_buff[MAX_CAMERA_NUM];
recursive_mutex Network::m_mutex[MAX_CAMERA_NUM];
mutex Network::mutex_recv[MAX_CAMERA_NUM];
condition_variable_any Network::Cond_Var[MAX_CAMERA_NUM];
condition_variable Network::thread_Cond_Var[MAX_CAMERA_NUM];
static const int FRAME_PREPADDING_BYTES = 2;
std::atomic<bool> Network::Send_Successful[MAX_CAMERA_NUM];
bool Network::Data_Received[MAX_CAMERA_NUM];
bool Network::Server_Connected[MAX_CAMERA_NUM];
bool Network::Thread_Detached[MAX_CAMERA_NUM];
bool Network::InterruptDetected[MAX_CAMERA_NUM];

void *Network::rawPayloads[MAX_CAMERA_NUM];
std::vector<std::string> m_connectionList;

/*
* isServer_Connected(): checks if server is connected
* Parameters:        none
* returns:           true  - if server is connected
                     false - if no server is connected
* Desription:   This function checks if server is connected and returns
Server_Connected flag value.
*/
bool Network::isServer_Connected() {
    return Network::Server_Connected[m_connectionId];
}

/*
* isThread_Running(): check if thread created is running
* Parameters:        none
* returns:           true  - if thread is alive
                     false - if thread is not running
* Desription:   This function returns Thread_Running flag value to check if
thread is running.
*/
bool Network::isThread_Running() {
    /*Return true if thread has completed*/
    if (Network::Thread_Running[m_connectionId] == 2) {
        return true;
    } else {
        return false;
    }
}

/*
* isData_Received(): check if data is sent to server and returns Send_Successful
flag value
* Parameters:        none
* returns:           true  - if data has been sent succesfully
                     false - if error in data sending
* Desription:   This function returns Send_Successful flag value
*/
bool Network::isSend_Successful() {
    return Network::Send_Successful[m_connectionId];
}

/*
* isData_Received(): check if data received from server and returns
Data_Received flag value
* Parameters:        none
* returns:           true  - if data received successfully
                     false - if error in data receiving
* Desription:   This function is used to check if any data received from server
*               returns Data_Received flag value.
*/
bool Network::isData_Received() {
    return Network::Data_Received[m_connectionId];
}

/*
* ServerConnect():  intializes the zmq sockets and connects to server
* Parameters:       ip - the ip address of the server to connect to
* returns:          0 - on success
                   -1 - on error
* Desription:   This function initializes the zmq sockets and connects to server.
*/
int Network::ServerConnect(const std::string &ip) {

    uint8_t numTry = 0;
    zmq_ip = ip;
    // Set up ZeroMQ context and socket
    std::unique_ptr<zmq::context_t> m_context;
    std::unique_ptr<zmq::socket_t> m_command;
    std::unique_ptr<zmq::socket_t> m_monitor;
    m_context = std::make_unique<zmq::context_t>(2);
    contexts.at(m_connectionId) = std::move(m_context);
    m_command =
        std::make_unique<zmq::socket_t>(*contexts.at(m_connectionId), ZMQ_REQ);
    m_monitor =
        std::make_unique<zmq::socket_t>(*contexts.at(m_connectionId), ZMQ_PAIR);

    if (!m_connectionId) {

        command_socket.clear();
        command_socket.emplace_back(std::move(m_command));

        monitor_sockets.clear();
        monitor_sockets.emplace_back(std::move(m_monitor));

    } else {

        for (size_t i = 0; i < command_socket.size(); i++) {
            if (command_socket.at(i) == nullptr) {
                command_socket.erase(command_socket.begin() + i);
                i--;
            }
        }
        command_socket.push_back(std::move(m_command));

        for (size_t i = 0; i < monitor_sockets.size(); i++) {
            if (monitor_sockets.at(i) == nullptr) {
                monitor_sockets.erase(monitor_sockets.begin() + i);
                i--;
            }
        }
        monitor_sockets.push_back(std::move(m_monitor));
    }

    std::string monitor_endpoint =
        "inproc://monitor-client" + std::to_string(m_connectionId);
    zmq_socket_monitor(command_socket[m_connectionId]->handle(),
                       monitor_endpoint.c_str(), ZMQ_EVENT_ALL);

    // Connect the monitor socket
    monitor_sockets[m_connectionId]->connect(monitor_endpoint);

    // Start a new thread to service any pending events on ZeroMQ socket
    threadObj[m_connectionId] =
        std::thread(&Network::call_zmq_service, this, std::ref(ip));

    while (!threadObj_started) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    Network::Thread_Detached[m_connectionId] = true;
    threadObj[m_connectionId].detach();

    while (numTry < 3) { // Try 3 times to connect

        if (!Server_Connected[m_connectionId]) {
            LOG(INFO) << "Attempting to connect server... ";
            try {

                command_socket[m_connectionId]->connect("tcp://" + ip +
                                                        ":5556");
            } catch (const zmq::error_t &e) {
                LOG(ERROR) << "Error Connecting to server" << e.what();
                if (e.num() == EHOSTUNREACH) {
                    LOG(ERROR) << "Host is unreachable";
                }
            }
        }
        numTry++;

        std::this_thread::sleep_for(std::chrono::milliseconds(20 * numTry));
    }

    /*Wait for thread to be ready and server is connected*/

    std::unique_lock<std::recursive_mutex> mlock(m_mutex[m_connectionId]);

    /*Wait till server is connected or timeout of 3 sec*/
    if (Cond_Var[m_connectionId].wait_for(
            mlock, std::chrono::seconds(3),
            std::bind(&Network::isServer_Connected, this)) == false) {
        Server_Connected[m_connectionId] = false;
        return -1;
    } else if (command_socket.at(m_connectionId) != NULL) {
        /*Wait for Server message to check another client is connected already
         * or not*/
        send_buff[m_connectionId].set_func_name("ServerConnect");
        send_buff[m_connectionId].set_expect_reply(true);
        if (SendCommand() != 0) {
            LOG(ERROR) << "Send Command Failed";
            Server_Connected[m_connectionId] = false;
            return -1;
        }
        if (recv_server_data() == 0) {
            /*Data received correctly*/
            if (strcmp(recv_buff[m_connectionId].message().c_str(),
                       server_msg) == 0) {
                /*Server is connected successfully*/
                cout << "Conn established" << endl;

                return 0;
            } else {
                /*Another client is connected already*/
                cout << "Server Message :: "
                     << recv_buff[m_connectionId].message() << endl;
                Server_Connected[m_connectionId] = false;
                return -1;
            }
        } else {
            /*No message received from Server*/
            Server_Connected[m_connectionId] = false;
            return -1;
        }
    } else if (command_socket.at(m_connectionId) == NULL) {
        Server_Connected[m_connectionId] = false;
        return -1;
    }

    return -1;
}

/*
* sendCommand(): send data to server
* Parameters:    none
* returns:       0 - on success
                -1 -  on error
* Desription:    This function is used to send the data to connected server.
*/
int Network::SendCommand(void *rawPayload) {
    int status = -1;
    uint8_t numRetry = 0;
    int siz = send_buff[m_connectionId].ByteSize();
    unsigned char *pkt = new unsigned char[siz];

    google::protobuf::io::ArrayOutputStream aos(pkt, siz);
    CodedOutputStream *coded_output = new CodedOutputStream(&aos);
    send_buff[m_connectionId].SerializeToCodedStream(coded_output);

    recv_buff[m_connectionId].Clear();

    while (numRetry++ < MAX_RETRY_CNT && Server_Connected[m_connectionId]) {

        zmq::message_t request(pkt, siz);
        if (command_socket[m_connectionId]->send(request,
                                                 zmq::send_flags::none)) {
            status = 0;
            Send_Successful[m_connectionId] = true;
            Cond_Var[m_connectionId].notify_all();
            return status;
        } else {
            status = -1; // Timeout occurred
            break;
#ifdef NW_DEBUG
            std::cout << "Send Timeout error" << std::endl;
#endif
        }
    }

    if (!Server_Connected[m_connectionId]) {
        status = -2;
    }

    m_latestActivityTimestamp = std::chrono::steady_clock::now();

    return status;
}

/*
* recv_server_data():  receive data from server
* Parameters:   None
* returns:      0  - on success
                -1 -  on error
* Desription:   This function is used to receive the data from connected server
*/
int Network::recv_server_data() {
    int status = -1;
    uint8_t numRetry = 0;

    while (numRetry++ < MAX_RETRY_CNT &&
           Server_Connected[m_connectionId] != false) {

        /*Acquire the lock*/
        std::unique_lock<std::mutex> mlock(mutex_recv[m_connectionId]);
        if (Cond_Var[m_connectionId].wait_for(
                mlock, std::chrono::seconds(10),
                std::bind(&Network::isSend_Successful, this)) == true) {
            /*reset the flag value to receive again*/
            Data_Received[m_connectionId] = false;

            zmq::message_t message;

            if (command_socket[m_connectionId]->recv(message,
                                                     zmq::recv_flags::none)) {
                // Check if data is received correctly
                if (message.size() >= 0) {
                    google::protobuf::io::ArrayInputStream ais(
                        static_cast<char *>(message.data()),
                        static_cast<int>(message.size()));
                    CodedInputStream coded_input(&ais);
                    recv_buff[m_connectionId].ParseFromCodedStream(
                        &coded_input);

                    recv_data_error = 0;
                    Data_Received[m_connectionId] = true;

                    if (recv_buff[m_connectionId].interrupt_occured()) {
                        InterruptDetected[m_connectionId] = true;
                    }

                    /*Notify the host SDK that data is received from server*/
                    Cond_Var[m_connectionId].notify_all();
                    status = 0;
                    break;
                }
            } else {
                /*No data received, retry sending command */
                LOG(INFO)
                    << "Data not received from server going to send again";
                if (SendCommand() != 0) {
                    status = -1;
                    break;
                }
            }
        } else {
            /*No data received till timeout, retry sending command */
            if (SendCommand() != 0) {
                status = -1;
                break;
            }
        }
    }

    if (Server_Connected[m_connectionId] == false) {
        status = -2;
    }

    send_buff[m_connectionId].Clear();

    if (status == 0) {
        if (InterruptDetected[m_connectionId]) {
            InterruptDetected[m_connectionId] = false;
            if (m_intNotifCb) {
                m_intNotifCb();
            }
        }
    }

    m_latestActivityTimestamp = std::chrono::steady_clock::now();

    return status;
}

/*
 * call_zmq_service():  calls zmq service zmq_event_t
 * Parameters:   IP - the ip address of the server to connect to
 * returns:      None
 * Desription:   This function calls zmq_event_t to monitor any event
 * pending zmq activity
 */

void Network::call_zmq_service(const std::string &ip) {
    threadObj_started = true;
    while (running[m_connectionId]) {

        zmq_event_t event;
        zmq::message_t msg;
        try {
            zmq::message_t event;
            if (monitor_sockets[m_connectionId]) {
                monitor_sockets[m_connectionId]->recv(msg);
            } else {
                break; // socket closed exiting the flag
            }
        } catch (const zmq::error_t &e) {
            if (monitor_sockets[m_connectionId]) {
                monitor_sockets[m_connectionId]->close();
            }
        }

        memcpy(&event, msg.data(), sizeof(event));

        callback_function(command_socket.at(m_connectionId), event);
    }
}

/*
* callback_function():  Handles the zmq events
* Parameters:           stx - socket to monitor
                        reasons - zmq event occurred for stx instance
* returns:              0
* Desription:           This function handles the zmq events and take
appropriate action
*/
int Network::callback_function(std::unique_ptr<zmq::socket_t> &stx,
                               const zmq_event_t &reason) {

    int connectionId = 0;
    auto status = std::find(command_socket.begin(), command_socket.end(), stx);
    if (status != command_socket.end()) {
        connectionId =
            static_cast<int>(std::distance(command_socket.begin(), status));
    }

    // Handle the event based on the connection ID
    switch (reason.event) {
    case ZMQ_EVENT_CONNECTED:
        LOG(INFO) << "Connected to server";
        Server_Connected[connectionId] = true;
        Cond_Var[connectionId].notify_all();
        break;
    case ZMQ_EVENT_CLOSED:
        LOG(INFO) << "Closed connection with connection ID: " << connectionId;
        /*Set a flag to indicate server connection is closed abruptly*/
        {
            std::lock_guard<std::recursive_mutex> guard(m_mutex[connectionId]);
            Server_Connected[connectionId] = false;
            running[connectionId] = false;
            command_socket.at(connectionId)->close();
            monitor_sockets.at(connectionId)->close();
            contexts.at(connectionId)->close();
            command_socket.at(connectionId) = NULL;
            monitor_sockets.at(connectionId) = NULL;
            contexts.at(connectionId) = NULL;
        }
        break;
    case ZMQ_EVENT_CONNECT_RETRIED:
        LOG(INFO) << "Connection retried to with connection ID: "
                  << connectionId;
        break;
    case ZMQ_EVENT_DISCONNECTED:
        LOG(INFO) << "Disconnected from server at with connection ID: "
                  << connectionId;
        /*Set a flag to indicate server connection is closed abruptly*/
        {
            std::lock_guard<std::recursive_mutex> guard(m_mutex[connectionId]);
            Server_Connected[connectionId] = false;
            running[connectionId] = false;
            command_socket.at(connectionId)->close();
            monitor_sockets.at(connectionId)->close();
            contexts.at(connectionId)->close();
            command_socket.at(connectionId) = NULL;
            monitor_sockets.at(connectionId) = NULL;
            contexts.at(connectionId) = NULL;
        }
        break;
    case ZMQ_EVENT_CONNECT_DELAYED:
        LOG(INFO) << "Event: CONNECT_DELAYED - Connection attempt delayed, "
                     "server might be unavailable.";
        {
            std::lock_guard<std::recursive_mutex> guard(m_mutex[connectionId]);
            Server_Connected[connectionId] = false;
        }
        break;
    default:
#ifdef NW_DEBUG
        LOG(INFO) << "Event: " << event.event
                  << " on with connection ID: " << connectionId;
#endif
        break;
    }

    return 0;
}

void Network::registerInterruptCallback(InterruptNotificationCallback &cb) {
    m_intNotifCb = cb;
}

std::chrono::steady_clock::time_point Network::getLatestActivityTimestamp() {
    return m_latestActivityTimestamp;
}

/*
 * Network():    Initializes the network parameters
 * Parameters:   None
 * Desription:   This function initializes the network parameters
 */
Network::Network(int connectionId)
    : m_intNotifCb(nullptr), m_latestActivityTimestamp{} {

    /*Initialize the static flags*/
    Network::Send_Successful[connectionId] = false;
    Network::Data_Received[connectionId] = false;
    Network::Thread_Running[connectionId] = 0;
    Network::Server_Connected[connectionId] = false;
    Network::Thread_Detached[connectionId] = false;
    Network::InterruptDetected[connectionId] = false;
    running[connectionId] = true;

    m_connectionId = connectionId;
    while (contexts.size() <= m_connectionId)
        contexts.emplace_back(nullptr);
}

/*
 * ~Network():   Destructor for network class
 * Parameters:   None
 * Desription:   Destructor for network class
 */
Network::~Network() {
    if (contexts.at(m_connectionId) != NULL &&
        Thread_Detached[m_connectionId]) {

        /*set a flag to complete the thread */

        Thread_Detached[m_connectionId] = false;
        {
            std::lock_guard<std::recursive_mutex> guard(
                m_mutex[m_connectionId]);
            Server_Connected[m_connectionId] = false;
            running[m_connectionId] = false;
            command_socket.at(m_connectionId)->close();
            monitor_sockets.at(m_connectionId)->close();
            contexts.at(m_connectionId)->close();
            command_socket.at(m_connectionId) = NULL;
            monitor_sockets.at(m_connectionId) = NULL;
            contexts.at(m_connectionId) = NULL;
        }
    }
}

int32_t Network::getFrame(uint16_t *buffer, uint32_t buf_size) {
    zmq::message_t message;
    if (frame_socket != nullptr) {
        frame_socket->recv(message, zmq::recv_flags::none);
    }

    if (buf_size == message.size()) {
        memcpy(buffer, message.data(), message.size());
    } else {
        LOG(ERROR) << "Received message of size " << message.size()
                   << " bytes . Expected message size " << buf_size
                   << " bytes , dropping the frame.";
    }

    return 0;
}

void Network::closeConnectionFrameSocket() {

    if (frame_socket) {
        frame_socket->close(); // Close the socket
        frame_socket.reset();  // Release the unique pointer
    }

    if (zmq_context) {
        zmq_context.reset(); // Release the ZMQ context
    }

    LOG(INFO) << "Frame socket connection closed.";
}

void Network::FrameSocketConnection(std::string &ip) {

    // Create a ZMQ connection for capturning frame in Async
    frame_context = std::make_unique<zmq::context_t>(1);
    frame_socket =
        std::make_unique<zmq::socket_t>(*frame_context, zmq::socket_type::pull);
    frame_socket->setsockopt(ZMQ_RCVTIMEO,
                             1100); // TODO: Base ZMQ_RCVTIMEO on the frame rate
    frame_socket->setsockopt(ZMQ_RCVHWM, (int *)&max_buffer_size,
                             sizeof(max_buffer_size));
    std::string zmq_address = "tcp://" + ip + ":5555";
    frame_socket->connect(zmq_address);
    LOG(INFO) << "Frame Client Connection established.";
}
