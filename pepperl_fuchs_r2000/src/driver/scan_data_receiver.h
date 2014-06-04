// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Use, modification, and distribution is subject to the
// 3-clause BSD license ("Revised BSD License",
// "New BSD License", or "Modified BSD License")
// You should have received a copy of this license
// in a file named COPYING or LICENSE.

#ifndef SCAN_DATA_RECEIVER_H
#define SCAN_DATA_RECEIVER_H

#define BOOST_CB_DISABLE_DEBUG
#include <string>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <pepperl_fuchs_r2000/packet_structure.h>

namespace pepperl_fuchs {

//! \class ScanDataReceiver
//! \brief Receives data of the laser range finder via IP socket
//! Receives the scanner data with asynchronous functions of the Boost::Asio library
class ScanDataReceiver
{
public:
    //! Connect synchronously to the given IP and TCP port and start reading asynchronously
    ScanDataReceiver(const std::string hostname, const int tcp_port);

    //! Open an UDP port and listen on it
    ScanDataReceiver();

    //! Disconnect cleanly
    ~ScanDataReceiver();

    //! Get open and receiving UDP port
    int getUDPPort() const { return udp_port_; }

    //! Return connection status
    bool isConnected() const { return is_connected_; }

    //! Disconnect and cleanup
    void disconnect();

    //! Pop a single scan out of the internal FIFO queue
    //! CAUTION: Returns also unfinished scans for which a full rotation is not received yet
    //! Call getFullScansAvailable() first to see how many full scans are available
    //! @returns A ScanData struct with distance and amplitude data as well as the packet headers belonging to the data
    ScanData getScan();

    //! Pop a single full scan out of the internal FIFO queue if there is any
    //! If no full scan is available yet, blocks until a full scan is available
    //! @returns A ScanData struct with distance and amplitude data as well as the packet headers belonging to the data
    ScanData getFullScan();

    //! Get the total number of laserscans available (even scans which are not fully reveived yet)
    std::size_t getScansAvailable() const { return scan_data_.size(); }

    //! Get the total number of fully received laserscans available
    std::size_t getFullScansAvailable() const;

private:
    //! Asynchronous callback function, called if data has been reveived by the TCP socket
    void handleSocketRead(const boost::system::error_code& error);

    //! Asynchronous callback function, called if data has been reveived by the UDP socket
    void handleSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred);

    //! Try to read and parse next packet from the internal ring buffer
    //! @returns True if a packet has been parsed, false otherwise
    bool handleNextPacket();

    //! Search for magic header bytes in the internal ring buffer
    //! @returns Position of possible packet start, which normally should be zero
    int findPacketStart();

    //! Try to read a packet from the internal ring buffer
    //! @returns True on success, False otherwise
    bool retrievePacket( std::size_t start, PacketTypeC* p );

    //! Checks if the connection is alive
    //! @returns True if connection is alive, false otherwise
    bool checkConnection();

    //! Read fast from the front of the internal ring buffer
    //! @param dst Destination buffer
    //! @numbytes Number of bytes to read
    void readBufferFront(char* dst, std::size_t numbytes );

    //! Write fast at the back of the internal ring buffer
    //! @param src Source buffer
    //! @numbytes Number of bytes to write
    void writeBufferBack(char* src, std::size_t numbytes );

    //! Data (UDP) port at local side
    int udp_port_;

    //! Internal connection state
    bool is_connected_;

    //! Event handler thread
    boost::thread io_service_thread_;
    boost::asio::io_service io_service_;

    //! Boost::Asio streambuffer
    boost::asio::streambuf inbuf_;

    //! Input stream
    std::istream instream_;

    //! Receiving socket
    boost::asio::ip::tcp::socket* tcp_socket_;

    //! Receiving socket
    boost::asio::ip::udp::socket* udp_socket_;

    //! Endpoint in case of UDP receiver
    boost::asio::ip::udp::endpoint udp_endpoint_;

    //! Buffer in case of UDP receiver
    std::array< char, 65536 > udp_buffer_;

    //! Internal ringbuffer for temporarily storing reveived data
    boost::circular_buffer<char> ring_buffer_;

    //! Protection against data races between ROS and IO threads
    std::mutex data_mutex_;

    //! Data notification condition variable
    std::condition_variable data_notifier_;

    //! Double ended queue with sucessfully received and parsed data, organized as single complete scans
    std::deque<ScanData> scan_data_;

    //! time in seconds since epoch, when last data was received
    double last_data_time_;
};

}
#endif // SCAN_DATA_RECEIVER_H
