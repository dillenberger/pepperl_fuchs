// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Use, modification, and distribution is subject to the
// 3-clause BSD license ("Revised BSD License",
// "New BSD License", or "Modified BSD License")
// You should have received a copy of this license
// in a file named COPYING or LICENSE.

#include <pepperl_fuchs_r2000/http_command_interface.h>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pepperl_fuchs {

//-----------------------------------------------------------------------------
HttpCommandInterface::HttpCommandInterface(const std::string &http_host, int http_port)
{
    http_host_ = http_host;
    http_port_ = http_port;
    http_status_code_ = 0;
}

//-----------------------------------------------------------------------------
int HttpCommandInterface::httpGet(const std::string request_path, std::string &header, std::string &content)
{
    header = "";
    content = "";
    using boost::asio::ip::tcp;
    try
    {
        boost::asio::io_service io_service;

        // Lookup endpoint
        tcp::resolver resolver(io_service);
        tcp::resolver::query query(http_host_, std::to_string(http_port_));
        tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        tcp::resolver::iterator end;

        // Create socket
        tcp::socket socket(io_service);
        boost::system::error_code error = boost::asio::error::host_not_found;

        // Iterate over endpoints and etablish connection
        while (error && endpoint_iterator != end)
        {
            socket.close();
            socket.connect(*endpoint_iterator++, error);
        }
        if (error)
            throw boost::system::system_error(error);

        // Prepare request
        boost::asio::streambuf request;
        std::ostream request_stream(&request);
        request_stream << "GET " << request_path << " HTTP/1.0\r\n\r\n";

        boost::asio::write(socket, request);

        // Read the response status line. The response streambuf will automatically
        // grow to accommodate the entire line. The growth may be limited by passing
        // a maximum size to the streambuf constructor.
        boost::asio::streambuf response;
        boost::asio::read_until(socket, response, "\r\n");

        // Check that response is OK.
        std::istream response_stream(&response);
        std::string http_version;
        response_stream >> http_version;
        unsigned int status_code;
        response_stream >> status_code;
        std::string status_message;
        std::getline(response_stream, status_message);
        if (!response_stream || http_version.substr(0, 5) != "HTTP/")
        {
            std::cout << "Invalid response\n";
            return 0;
        }

        // Read the response headers, which are terminated by a blank line.
        boost::asio::read_until(socket, response, "\r\n\r\n");

        // Process the response headers.
        std::string tmp;
        while (std::getline(response_stream, tmp) && tmp != "\r")
            header += tmp+"\n";

        // Write whatever content we already have to output.
        while (std::getline(response_stream, tmp))
            content += tmp;

        // Read until EOF, writing data to output as we go.
        while (boost::asio::read(socket, response, boost::asio::transfer_at_least(1), error))
        {
            response_stream.clear();
            while (std::getline(response_stream, tmp))
                content += tmp;
        }

        if (error != boost::asio::error::eof)
            throw boost::system::system_error(error);

        // Substitute CRs by a space
        for( std::size_t i=0; i<header.size(); i++ )
            if( header[i] == '\r' )
                header[i] = ' ';

        for( std::size_t i=0; i<content.size(); i++ )
            if( content[i] == '\r' )
                content[i] = ' ';

        return status_code;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
        return 0;
    }
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::sendHttpCommand(const std::string cmd, const std::map<std::string, std::string> param_values)
{
    // Build request string
    std::string request_str = "/cmd/" + cmd + "?";
    for( auto& kv : param_values )
        request_str += kv.first + "=" + kv.second + "&";
    if(request_str.back() == '&' )
        request_str = request_str.substr(0,request_str.size()-1);

    // Do HTTP request
    std::string header, content;
    http_status_code_ = httpGet(request_str,header,content);

    // Try to parse JSON response
    try
    {
        std::stringstream ss(content);
        boost::property_tree::json_parser::read_json(ss,pt_);
    }
    catch (std::exception& e)
    {
        std::cerr << "ERROR: Exception: " <<  e.what() << std::endl;
        return false;
    }

    // Check HTTP-status code
    if( http_status_code_ != 200 )
        return false;
    else
        return true;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::sendHttpCommand(const std::string cmd, const std::string param, const std::string value)
{
    std::map<std::string, std::string> param_values;
    if( param != "" )
        param_values[param] = value;
    return sendHttpCommand(cmd,param_values);
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::setParameter(const std::string name, const std::string value)
{
    return sendHttpCommand("set_parameter",name,value) && checkErrorCode();
}

//-----------------------------------------------------------------------------
boost::optional< std::string > HttpCommandInterface::getParameter(const std::string name)
{
    if( !sendHttpCommand("get_parameter","list",name) || ! checkErrorCode()  )
        return boost::optional<std::string>();
    return pt_.get_optional<std::string>(name);
}

//-----------------------------------------------------------------------------
std::map< std::string, std::string > HttpCommandInterface::getParameters(const std::vector<std::string> &names)
{
    // Build request string
    std::map< std::string, std::string > key_values;
    std::string namelist;
    for( const auto& s: names )
        namelist += (s + ";");
    namelist.substr(0,namelist.size()-1);

    // Read parameter values via HTTP/JSON request/response
    if( !sendHttpCommand("get_parameter","list",namelist) || ! checkErrorCode()  )
        return key_values;

    // Extract values from JSON property_tree
    for( const auto& s: names )
    {
        auto ov = pt_.get_optional<std::string>(s);
        if( ov )
            key_values[s] = *ov;
        else
            key_values[s] = "--COULD NOT RETRIEVE VALUE--";
    }

    return key_values;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::checkErrorCode()
{
    // Check the JSON response if error_code == 0 && error_text == success
    boost::optional<int> error_code = pt_.get_optional<int>("error_code");
    boost::optional<std::string> error_text = pt_.get_optional<std::string>("error_text");
    if( !error_code || (*error_code) != 0 || !error_text || (*error_text) != "success" )
    {
        if( error_text )
            std::cerr << "ERROR: scanner replied: " << *error_text << std::endl;
        return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
boost::optional<ProtocolInfo> HttpCommandInterface::getProtocolInfo()
{
    // Read protocol info via HTTP/JSON request/response
    if( !sendHttpCommand("get_protocol_info") || !checkErrorCode() )
        return boost::optional<ProtocolInfo>();

    // Read and set protocol info
    boost::optional<std::string> protocol_name = pt_.get_optional<std::string>("protocol_name");
    boost::optional<int> version_major = pt_.get_optional<int>("version_major");
    boost::optional<int> version_minor = pt_.get_optional<int>("version_minor");
    auto ocommands = pt_.get_child_optional("commands");
    if( !protocol_name || !version_major || !version_minor || !ocommands )
        return boost::optional<ProtocolInfo>();

    ProtocolInfo pi;
    pi.protocol_name = *protocol_name;
    pi.version_major = *version_major;
    pi.version_minor = *version_minor;

    // Read available commands of the protocol
    boost::property_tree::ptree commands = *ocommands;
    for( auto i= commands.begin(); i!=commands.end(); i++ )
    {
        std::string cmd = i->second.get<std::string>("");
        pi.commands.push_back(cmd);
    }

    return pi;
}

//-----------------------------------------------------------------------------
std::vector< std::string > HttpCommandInterface::getParameterList()
{
    // Read available parameters via HTTP/JSON request/response
    std::vector< std::string > parameter_list;
    if( !sendHttpCommand("list_parameters") || !checkErrorCode() )
        return parameter_list;

    // Check if JSON contains the key "parameters"
    auto oparameters = pt_.get_child_optional("parameters");
    if( !oparameters )
        return parameter_list;

    // Extract parameter names from JSON
    boost::property_tree::ptree parameters = *oparameters;
    for( auto i= parameters.begin(); i!=parameters.end(); i++ )
    {
        std::string param = i->second.get<std::string>("");
        parameter_list.push_back(param);
    }

    return parameter_list;

}

//-----------------------------------------------------------------------------
boost::optional<HandleInfo> HttpCommandInterface::requestHandleTCP(int start_angle)
{
    // Prepare HTTP request
    std::map< std::string, std::string > params;
    params["packet_type"] = "C";
    params["start_angle"] = std::to_string(start_angle);

    // Request handle via HTTP/JSON request/response
    if( !sendHttpCommand("request_handle_tcp", params) || !checkErrorCode() )
        return boost::optional<HandleInfo>();

    // Extract handle info from JSON response
    boost::optional<int> port = pt_.get_optional<int>("port");
    boost::optional<std::string> handle = pt_.get_optional<std::string>("handle");
    if(!port || !handle)
        return boost::optional<HandleInfo>();

    // Prepare return value
    HandleInfo hi;
    hi.handle_type = HandleInfo::HANDLE_TYPE_TCP;
    hi.handle = *handle;
    hi.hostname = http_host_;
    hi.port = *port;
    hi.packet_type = 'C';
    hi.start_angle = start_angle;
    hi.watchdog_enabled = true;
    hi.watchdog_timeout = 60000;
    return hi;
}

//-----------------------------------------------------------------------------
boost::optional<HandleInfo> HttpCommandInterface::requestHandleUDP(int port, std::string hostname, int start_angle)
{
    // Prepare HTTP request
    if( hostname == "" )
        hostname = discoverLocalIP();
    std::map< std::string, std::string > params;
    params["packet_type"] = "C";
    params["start_angle"] = std::to_string(start_angle);
    params["port"] = std::to_string(port);
    params["address"] = hostname;

    // Request handle via HTTP/JSON request/response
    if( !sendHttpCommand("request_handle_udp", params) || !checkErrorCode() )
        return boost::optional<HandleInfo>();

    // Extract handle info from JSON response
    boost::optional<std::string> handle = pt_.get_optional<std::string>("handle");
    if(!handle)
        return boost::optional<HandleInfo>();

    // Prepare return value
    HandleInfo hi;
    hi.handle_type = HandleInfo::HANDLE_TYPE_UDP;
    hi.handle = *handle;
    hi.hostname = hostname;
    hi.port = port;
    hi.packet_type = 'C';
    hi.start_angle = start_angle;
    hi.watchdog_enabled = true;
    hi.watchdog_timeout = 60000;
    return hi;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::releaseHandle(const std::string& handle)
{
    if( !sendHttpCommand("release_handle", "handle", handle) || !checkErrorCode() )
        return false;
    return true;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::startScanOutput(const std::string& handle)
{
    if( !sendHttpCommand("start_scanoutput", "handle", handle) || !checkErrorCode() )
        return false;
    return true;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::stopScanOutput(const std::string& handle)
{
    if( !sendHttpCommand("stop_scanoutput", "handle", handle) || !checkErrorCode() )
        return false;
    return true;
}
//-----------------------------------------------------------------------------
bool HttpCommandInterface::feedWatchdog(const std::string &handle)
{
    if( !sendHttpCommand("feed_watchdog", "handle", handle) || !checkErrorCode() )
        return false;
    return true;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::rebootDevice()
{
    if( !sendHttpCommand("reboot_device") || !checkErrorCode() )
        return false;
    return true;
}

//-----------------------------------------------------------------------------
bool HttpCommandInterface::resetParameters(const std::vector<std::string> &names)
{
    // Prepare HTTP request
    std::string namelist;
    for( const auto& s: names )
        namelist += (s + ";");
    namelist.substr(0,namelist.size()-1);

    if( !sendHttpCommand("reset_parameter","list",namelist) || ! checkErrorCode()  )
        return false;

    return true;
}

//-----------------------------------------------------------------------------
std::string HttpCommandInterface::discoverLocalIP()
{
    std::string local_ip;
    try
    {
        using boost::asio::ip::udp;
        boost::asio::io_service netService;
        udp::resolver resolver(netService);
        udp::resolver::query query(udp::v4(), http_host_ , "");
        udp::resolver::iterator endpoints = resolver.resolve(query);
        udp::endpoint ep = *endpoints;
        udp::socket socket(netService);
        socket.connect(ep);
        boost::asio::ip::address addr = socket.local_endpoint().address();
        local_ip = addr.to_string();
    }
    catch (std::exception& e)
    {
        std::cerr << "Could not deal with socket-exception: " << e.what() << std::endl;
    }

    return local_ip;
}

//-----------------------------------------------------------------------------
}
