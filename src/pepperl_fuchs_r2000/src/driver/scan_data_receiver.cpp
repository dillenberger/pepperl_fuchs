// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Use, modification, and distribution is subject to the
// 3-clause BSD license ("Revised BSD License",
// "New BSD License", or "Modified BSD License")
// You should have received a copy of this license
// in a file named COPYING or LICENSE.

#include "scan_data_receiver.h"
#include <chrono>
#include <ctime>

namespace pepperl_fuchs {

ScanDataReceiver::ScanDataReceiver(const std::string hostname, const int tcp_port):inbuf_(4096),instream_(&inbuf_),ring_buffer_(65536),scan_data_()
{
    last_data_time_ = std::time(0);
    tcp_socket_ = 0;
    udp_socket_ = 0;
    udp_port_ = -1;
    is_connected_ = false;

    std::cout << "Connecting to TCP data channel at " << hostname << ":" << tcp_port << " ... ";
    try
    {
        // Resolve hostname/ip
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::query query(hostname, std::to_string(tcp_port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::ip::tcp::resolver::iterator end;

        tcp_socket_ = new boost::asio::ip::tcp::socket(io_service_);
        boost::system::error_code error = boost::asio::error::host_not_found;

        // Iterate over endpoints and etablish connection
        while (error && endpoint_iterator != end)
        {
            tcp_socket_->close();
            tcp_socket_->connect(*endpoint_iterator++, error);
        }
        if (error)
            throw boost::system::system_error(error);

        // Start async reading
        boost::asio::async_read(*tcp_socket_, inbuf_, boost::bind(&ScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        is_connected_ = true;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

//-----------------------------------------------------------------------------
ScanDataReceiver::ScanDataReceiver():inbuf_(4096),instream_(&inbuf_),ring_buffer_(65536),scan_data_()
{
    tcp_socket_ = 0;
    udp_socket_ = 0;
    udp_port_ = -1;
    is_connected_ = false;


    try
    {
        udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
        udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));
        udp_port_ = udp_socket_->local_endpoint().port();
        // Start async reading
        udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                                        boost::bind(&ScanDataReceiver::handleSocketRead, this,
                                                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        is_connected_ = true;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
    std::cout << "Receiving scanner data at local UDP port " << udp_port_ << " ... ";

}

//-----------------------------------------------------------------------------
ScanDataReceiver::~ScanDataReceiver()
{
    disconnect();
    delete udp_socket_;
    delete tcp_socket_;
}

//-----------------------------------------------------------------------------
void ScanDataReceiver::handleSocketRead(const boost::system::error_code& error)
{
    if (!error )
    {
        // Read all received data and write it to the internal ring buffer
        instream_.clear();
        while(!instream_.eof())
        {
            char buf[4096];
            instream_.read(buf,4096);
            int bytes_read = instream_.gcount();
            writeBufferBack(buf,bytes_read);
        }

        // Handle (read and parse) packets stored in the internal ring buffer
        while( handleNextPacket() ) {}

        // Read data asynchronously
        boost::asio::async_read(*tcp_socket_, inbuf_, boost::bind(&ScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error));
    }
    else
    {
        if( error.value() != 995 )
            std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        disconnect();
    }
    last_data_time_ = std::time(0);
}

//-----------------------------------------------------------------------------
void ScanDataReceiver::handleSocketRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    if (!error )
    {
        // Read all received data and write it to the internal ring buffer
        writeBufferBack(&udp_buffer_[0],bytes_transferred);

        // Handle (read and parse) packets stored in the internal ring buffer
        while( handleNextPacket() ) {}

        // Read data asynchronously
        udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0],udp_buffer_.size()), udp_endpoint_,
                                        boost::bind(&ScanDataReceiver::handleSocketRead, this,
                                                    boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        if( error.value() != 995 )
            std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        disconnect();
    }
    last_data_time_ = std::time(0);
}

//-----------------------------------------------------------------------------
bool ScanDataReceiver::handleNextPacket()
{
    // Search for a packet
    int packet_start = findPacketStart();
    if( packet_start<0 )
        return false;

    // Try to retrieve packet
    char buf[65536];
    PacketTypeC* p = (PacketTypeC*) buf;
    if( !retrievePacket(packet_start,p) )
        return false;

    // Lock internal outgoing data queue, automatically unlocks at end of function
    std::unique_lock<std::mutex> lock(data_mutex_);

    // Create new scan container if necessary
    if( p->header.packet_number == 1 || scan_data_.empty() )
    {
        scan_data_.emplace_back();
        if( scan_data_.size()>100 )
        {
            scan_data_.pop_front();
            std::cerr << "Too many scans in receiver queue: Dropping scans!" << std::endl;
        }
        data_notifier_.notify_one();
    }
    ScanData& scandata = scan_data_.back();

    // Parse payload of packet
    std::uint32_t* p_scan_data = (std::uint32_t*) &buf[p->header.header_size];
    int num_scan_points = p->header.num_points_packet;

    for( int i=0; i<num_scan_points; i++ )
    {
        unsigned int data = p_scan_data[i];
        unsigned int distance = (data & 0x000FFFFF);
        unsigned int amplitude = (data & 0xFFFFF000) >> 20;

        scandata.distance_data.push_back(distance);
        scandata.amplitude_data.push_back(amplitude);
    }

    // Save header
    scandata.headers.push_back(p->header);

    return true;
}

//-----------------------------------------------------------------------------
int ScanDataReceiver::findPacketStart()
{
    if( ring_buffer_.size()<60 )
        return -1;
    for( std::size_t i=0; i<ring_buffer_.size()-4; i++)
    {
        if(   ((unsigned char) ring_buffer_[i])   == 0x5c
           && ((unsigned char) ring_buffer_[i+1]) == 0xa2
           && ((unsigned char) ring_buffer_[i+2]) == 0x43
           && ((unsigned char) ring_buffer_[i+3]) == 0x00 )
        {
            return i;
        }
    }
    return -2;
}

//-----------------------------------------------------------------------------
bool ScanDataReceiver::retrievePacket(std::size_t start, PacketTypeC *p)
{
    if( ring_buffer_.size()<60 )
        return false;

    // Erase preceding bytes
    ring_buffer_.erase_begin(start);

    char* pp = (char*) p;
    // Read header
    readBufferFront(pp,60);

    if( ring_buffer_.size() < p->header.packet_size )
        return false;

    // Read header+payload data
    readBufferFront(pp,p->header.packet_size);

    // Erase packet from ring buffer
    ring_buffer_.erase_begin(p->header.packet_size);
    return true;
}

//-----------------------------------------------------------------------------
void ScanDataReceiver::disconnect()
{
    is_connected_ = false;
    try
    {
        if( tcp_socket_ )
            tcp_socket_->close();
        if( udp_socket_ )
            udp_socket_->close();
        io_service_.stop();
        if( boost::this_thread::get_id() != io_service_thread_.get_id() )
            io_service_thread_.join();
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

//-----------------------------------------------------------------------------
bool ScanDataReceiver::checkConnection()
{
    if( !isConnected() )
        return false;
    if( (std::time(0)-last_data_time_) > 2 )
    {
        disconnect();
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
ScanData ScanDataReceiver::getScan()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    ScanData data(std::move(scan_data_.front()));
    scan_data_.pop_front();
    return data;
}

//-----------------------------------------------------------------------------
ScanData ScanDataReceiver::getFullScan()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    while( checkConnection() && isConnected() && scan_data_.size()<2 )
    {
        data_notifier_.wait_for(lock, std::chrono::seconds(1));
    }
    ScanData data;
    if( scan_data_.size() >= 2 && isConnected() )
    {
        data = ScanData(std::move(scan_data_.front()));
        scan_data_.pop_front();
    }
    return data;
}

//-----------------------------------------------------------------------------
std::size_t ScanDataReceiver::getFullScansAvailable() const
{
    if( scan_data_.size() == 0 )
        return 0;
    else
        return scan_data_.size()-1;
}
//-----------------------------------------------------------------------------
void ScanDataReceiver::writeBufferBack(char *src, std::size_t numbytes)
{
    if( ring_buffer_.size()+numbytes > ring_buffer_.capacity() )
        throw std::exception();
    ring_buffer_.resize(ring_buffer_.size()+numbytes);
    char* pone = ring_buffer_.array_one().first;
    std::size_t pone_size = ring_buffer_.array_one().second;
    char* ptwo = ring_buffer_.array_two().first;
    std::size_t ptwo_size = ring_buffer_.array_two().second;

    if( ptwo_size >= numbytes )
    {
        std::memcpy(ptwo+ptwo_size-numbytes, src, numbytes);
    }
    else
    {
        std::memcpy(pone+pone_size+ptwo_size-numbytes,
                    src,
                    numbytes-ptwo_size );
        std::memcpy(ptwo,
                    src+numbytes-ptwo_size,
                    ptwo_size );
    }
}

//-----------------------------------------------------------------------------
void ScanDataReceiver::readBufferFront(char *dst, std::size_t numbytes)
{
    if( ring_buffer_.size() < numbytes )
        throw std::exception();
    char* pone = ring_buffer_.array_one().first;
    std::size_t pone_size = ring_buffer_.array_one().second;
    char* ptwo = ring_buffer_.array_two().first;
    //std::size_t ptwo_size = ring_buffer_.array_two().second;

    if( pone_size >= numbytes )
    {
        std::memcpy( dst, pone, numbytes );
    }
    else
    {
        std::memcpy( dst, pone, pone_size );
        std::memcpy( dst+pone_size, ptwo, numbytes-pone_size);
    }
}

//-----------------------------------------------------------------------------
}
