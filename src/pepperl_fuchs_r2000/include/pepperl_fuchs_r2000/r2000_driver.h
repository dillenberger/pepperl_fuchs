// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
// * Neither the name of Pepperl+Fuchs nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef R2000_DRIVER_H
#define R2000_DRIVER_H

#include <string>
#include <vector>
#include <map>
#include <boost/optional.hpp>
#include <pepperl_fuchs_r2000/protocol_info.h>
#include <pepperl_fuchs_r2000/packet_structure.h>

namespace pepperl_fuchs {

class HttpCommandInterface;
class ScanDataReceiver;

//! \class R2000Driver
//! \brief Driver for the laserscanner R2000 of Pepperl+Fuchs
class R2000Driver
{
public:
    //! Initialize driver
    R2000Driver();

    //! Cleanly disconnect in case of destruction
    ~R2000Driver();

    //! Connects to a given laserscanner, gets and checks protocol info of scanner, retrieves values of all parameters
    //! @param ip IP or hostname of laserscanner
    //! @param port Port to use for HTTP-Interface (defaults to 80)
    bool connect(const std::string hostname, int port=80);

    //! Disconnect from the laserscanner and reset internal state
    void disconnect();

    //! Return connection status
    bool isConnected() {return is_connected_; }

    //! Start capturing laserdata: Requests a handle and begin retrieving data from the scanner
    //! @returns True in case of success, False otherwise
    bool startCapturingTCP();

    //! Start capturing laserdata: Requests a handle and begin retrieving data from the scanner
    //! @returns True in case of success, False otherwise
    bool startCapturingUDP();

    //! Stop capturing laserdata: Release handle and stop retrieving data from the scanner
    //! @returns True in case of success, False otherwise
    bool stopCapturing();

    //! Return capture status
    //! @returns True if a capture is running, False otherwise
    bool isCapturing();

    //! Actively check connection to laserscanner
    //! @returns True if connection is alive, false otherwise
    bool checkConnection();

    //! Retrieve Protocol information of the scanner
    //! @returns A struct containing name, version and available commands of the protocol
    const ProtocolInfo& getProtocolInfo() { return protocol_info_; }

    //! Read all parameter values from the scanner
    //! @returns A key->value map with parametername->value
    const std::map< std::string, std::string >& getParameters();

    //! Get cached parameter values of the scanner
    //! @returns A key->value map with parametername->value
    const std::map< std::string, std::string >& getParametersCached() const {return parameters_;}

    //! Pop a single scan out of the driver's interal FIFO queue
    //! CAUTION: Returns also unfinished scans for which a full rotation is not received yet
    //! Call getFullScansAvailable() first to see how many full scans are available
    //! @returns A ScanData struct with distance and amplitude data as well as the packet headers belonging to the data
    ScanData getScan();

    //! Pop a single full scan out of the driver's internal FIFO queue if there is any
    //! If no full scan is available yet, blocks until a full scan is available
    //! @returns A ScanData struct with distance and amplitude data as well as the packet headers belonging to the data
    ScanData getFullScan();

    //! Get the total number of laserscans available (even scans which are not fully reveived yet)
    std::size_t getScansAvailable() const;

    //! Get the total number of fully received laserscans available
    std::size_t getFullScansAvailable() const;

    //! Set scan frequency (rotation speed of scanner head)
    //! @param frequency Frequency in Hz
    bool setScanFrequency( unsigned int frequency );

    //! Set number of samples per scan/rotation
    //! @param samples Only certain values are allowed (see Manual). Examples are 72, 360, 720, 1440, 1800, 3600, 7200, 10080, 25200
    //! @returns True on success, False otherwise
    bool setSamplesPerScan( unsigned int samples );

    //! Reboot Laserscanner (Takes ~60s)
    //! @returns True if command was successfully received, False otherwise
    bool rebootDevice();

    //! Reset certain parameters to factory default
    //! @param names Names of parameters to reset
    //! @returns True if successfull, False otherwise
    bool resetParameters( const std::vector<std::string>& names );

    //! Set a parameter by name and value
    //! @returns True if successfull, False otherwise
    bool setParameter( const std::string& name, const std::string& value );

    //! Feed the watchdog with the current handle ID, to keep the data connection alive
    void feedWatchdog(bool feed_always = false);

private:
    //! HTTP/JSON interface of the scanner
    HttpCommandInterface* command_interface_;

    //! Asynchronous data receiver
    ScanDataReceiver* data_receiver_;

    //! Internal connection state
    bool is_connected_;

    //! Internal capturing state
    bool is_capturing_;

    //! Last time the watchdog was fed
    double watchdog_feed_time_;

    //! Feeding interval (in seconds)
    double food_timeout_;

    //! Handle information about data connection
    boost::optional<HandleInfo> handle_info_;

    //! Cached version of the protocol info
    ProtocolInfo protocol_info_;

    //! Cached version of all parameter values
    std::map< std::string, std::string > parameters_;
};

} // NS pepperl_fuchs

#endif // R2000_DRIVER_H
