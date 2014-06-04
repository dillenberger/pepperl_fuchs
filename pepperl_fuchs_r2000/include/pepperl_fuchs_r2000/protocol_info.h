// Copyright (c) 2014, Pepperl+Fuchs GmbH, Mannheim
// Copyright (c) 2014, Denis Dillenberger
// All rights reserved.
//
// Use, modification, and distribution is subject to the
// 3-clause BSD license ("Revised BSD License",
// "New BSD License", or "Modified BSD License")
// You should have received a copy of this license
// in a file named COPYING or LICENSE.

#ifndef PROTOCOL_INFO_H
#define PROTOCOL_INFO_H
#include <vector>

namespace pepperl_fuchs {

//! \class ProtocolInfo
//! \brief Information about the HTTP/JSON protocol
struct ProtocolInfo
{
    //! protocol name, defaults to "pfsdp"
    std::string protocol_name;

    //! Major version of protocol
    int version_major;

    //! Minor version of protocol
    int version_minor;

    //! List of available commands
    std::vector< std::string > commands;
};

//! \class HandleInfo
//! \brief Encapsulates data about an etablished data connection
struct HandleInfo
{
    static const int HANDLE_TYPE_TCP = 0;
    static const int HANDLE_TYPE_UDP = 1;

    //! Handle type:
    //! 0: TCP
    //! 1: UDP
    int handle_type;

    //! UDP Handle: IP at client side where scanner data is sent to
    std::string hostname;

    //! Meaning depends on handle type (TCP or UDP)
    //! TCP port at scanner side where data can be retreived -or-
    //! UDP port at client side where scanner data is sent to
    int port;

    //! Handle ID
    std::string handle;

    //! Packet type, possible values are A,B or C
    char packet_type;

    //! Start angle of scan in 1/10000°, defaults to -1800000
    int start_angle;

    //! If watchdog is enabled, it has to be fed otherwise scanner closes the connection after timeout
    bool watchdog_enabled;

    //! Watchdog timeout in ms
    int watchdog_timeout;
};
}

#endif // PROTOCOL_INFO_H
