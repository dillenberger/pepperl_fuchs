// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Use, modification, and distribution is subject to the
// 3-clause BSD license ("Revised BSD License",
// "New BSD License", or "Modified BSD License")
// You should have received a copy of this license
// in a file named COPYING or LICENSE.

#ifndef HTTP_COMMAND_INTERFACE_H
#define HTTP_COMMAND_INTERFACE_H
#include <string>
#include <map>
#include <boost/property_tree/ptree.hpp>
#include <pepperl_fuchs_r2000/protocol_info.h>

namespace pepperl_fuchs {

//! \class HttpCommandInterface
//! \brief Allows accessing the HTTP/JSON interface of the Pepperl+Fuchs Laserscanner R2000
class HttpCommandInterface
{
public:
    //! Setup a new HTTP command interface
    //! @param http_ip IP or DNS name of sensor
    //! @param http_port HTTP/TCP port of sensor
    HttpCommandInterface(const std::string& http_host, int http_port=80);

    //! Get the HTTP hostname/IP of the scanner
    const std::string& getHttpHost() const { return http_host_; }

    //! Set sensor parameter
    //! @param name Name
    //! @param value Value
    //! @returns True on success, false otherwise
    bool setParameter(const std::string name, const std::string value);

    //! Get sensor parameter
    //! @param name Parameter name
    //! @returns Optional string value with value of given parameter name
    boost::optional<std::string> getParameter(const std::string name);

    //! Get multiple sensor parameters
    //! @param names Parameter names
    //! @returns vector with string values with the values of the given parameter names
    std::map< std::string, std::string > getParameters( const std::vector< std::string >& names );

    //! List available ro/rw parameters
    //! @returns A vector with the names of all available parameters
    std::vector< std::string > getParameterList();

    //! Get protocol info (protocol_name, version, commands)
    //! @returns A struct with the requested data
    boost::optional<ProtocolInfo> getProtocolInfo();

    //! Request TCP handle
    //! @param start_angle Set start angle for scans in the range [0,3600000] (1/10000°)
    //! @returns A valid HandleInfo on success, an empty boost::optional<HandleInfo> container otherwise
    boost::optional<HandleInfo> requestHandleTCP(int start_angle=-1800000);

    //! Request UDP handle
    //! @param port Set UDP port where scanner data should be sent to
    //! @param hostname Optional: Set hostname/IP where scanner data should be sent to, local IP is determined automatically if not specified
    //! @param start_angle Optional: Set start angle for scans in the range [0,3600000] (1/10000°), defaults to -1800000
    //! @returns A valid HandleInfo on success, an empty boost::optional<HandleInfo> container otherwise
    boost::optional<HandleInfo> requestHandleUDP(int port, std::string hostname = std::string(""), int start_angle=-1800000);

    //! Release handle
    bool releaseHandle( const std::string& handle );

    //! Initiate output of scan data
    bool startScanOutput( const std::string& handle );

    //! Terminate output of scan data
    bool stopScanOutput( const std::string& handle );

    //! Feed the watchdog to keep the handle alive
    bool feedWatchdog( const std::string& handle );

    //! Reboot laserscanner
    bool rebootDevice();

    //! Reset laserscanner parameters to factory default
    //! @param names Names of parameters to reset
    bool resetParameters(const std::vector< std::string >& names);

    //! Discovers the local IP of the NIC which talks to the laser range finder
    //! @returns The local IP as a string, an empty string otherwise
    std::string discoverLocalIP();

private:

    //! Send a HTTP-GET request to http_ip_ at http_port_
    //! @param requestStr The last part of an URL with a slash leading
    //! @param header  The response header returned as std::string, empty string in case of an error
    //! @param content The response content returned as std::string, empty string in case of an error
    //! @returns The HTTP status code or 0 in case of an error
    int httpGet(const std::string request_path, std::string& header, std::string& content);

    //! Send a sensor specific HTTP-Command
    //! @param cmd command name
    //! @param keys_values parameter->value map, which is encoded in the GET-request: ?p1=v1&p2=v2
    bool sendHttpCommand(const std::string cmd, const std::map< std::string, std::string > param_values );

    //! Send a sensor specific HTTP-Command with a single parameter
    //! @param cmd Command name
    //! @param param Parameter
    //! @param value Value
    bool sendHttpCommand(const std::string cmd, const std::string param = "", const std::string value = "" );

    //! Check the error code and text of the returned JSON by the last request
    //! @returns False in case of an error, True otherwise
    bool checkErrorCode();

    //! Scanner IP
    std::string http_host_;

    //! Port of HTTP-Interface
    int http_port_;

    //! Returned JSON as property_tree
    boost::property_tree::ptree pt_;

    //! HTTP-Status code of last request
    int http_status_code_;

};
}

#endif // HTTP_COMMAND_INTERFACE_H
