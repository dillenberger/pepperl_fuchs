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

#ifndef R2000_NODE_H
#define R2000_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace pepperl_fuchs {
class R2000Driver;

//! \class R2000Node
//! \brief ROS driver node for the Pepperl+Fuchs R2000 laser range finder
class R2000Node
{
public:
    //! Initialize and connect to laser range finder
    R2000Node();

    //! Callback function for control commands
    void cmdMsgCallback( const std_msgs::StringConstPtr& msg );

private:
    //! Connect to the laser range finder
    //! @returns True on success, false otherwise
    bool connect();

    //! Time callback function for getting data from the driver and sending them out
    void getScanData( const ros::TimerEvent& e);

    //! Internal ROS node handle
    ros::NodeHandle nh_;

    //! Callback timer for getScanData(...)
    ros::Timer get_scan_data_timer_;

    //! ROS publisher for publishing scan data
    ros::Publisher scan_publisher_;

    //! ROS subscriber for receiving control commands
    ros::Subscriber cmd_subscriber_;

    //! frame_id of sensor_msgs/Laserscan messages
    std::string frame_id_;

    //! IP or hostname of laser range finder
    std::string scanner_ip_;

    //! scan_frequency parameter
    int scan_frequency_;

    //! samples_per_scan parameter
    int samples_per_scan_;

    //! Pointer to driver
    R2000Driver* driver_;
};
}

#endif // R2000_NODE_H
