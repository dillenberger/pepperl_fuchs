Pepperl+Fuchs R2000 Driver
===============================

This is the documentation of a driver for the Pepperl+Fuchs OMD10M-R2000-B23 laser range finder.

The driver is based upon the widespread boost asio library (<http://www.boost.org>)

The driver comes as a library, which contains the actual driver, and has additionally a ROS-Node interface to the Robot Operating System (<http://www.ros.org>), which can be used optionally.


Sensor information
-------------------------------

![](https://raw.githubusercontent.com/dillenberger/pepperl_fuchs/master/pepperl_fuchs_r2000/doxygen/r2000.jpg)


_"The new 2-dimensional laser range sensor operates using tried-and-tested Pulse Ranging Technology and boasts a range of unique features. For example, the device features a laser emitter with a visible red light. Users can see where the scanning level is and can align the device accordingly. Thanks to its classification in laser class 1, the device can be used in all workspaces, without posing a health hazard to people. The device has a measuring angle of 360° and boasts a measuring frequency of up to 250,000 individual measurements per second. Combined with a scanning frequency of up to 50 Hz, the device is extremely well suited to highspeed applications. Another feature is the display integrated in the lens aperture."_ (from the official datasheet with permission from Pepperl+Fuchs)

Official Website: http://www.pepperl-fuchs.com/global/en/classid_53.htm?view=productdetails&prodid=43828#overview

Datasheet (en): http://files.pepperl-fuchs.com/selector_files/navi/productInfo/edb/232934_eng.pdf


Usage with ROS
---------------------------
The ROS package `pepperl_fuchs_r2000` consists of the driver library and a node named `r2000_node`, which is linked to the library. This is the actual driver node. The `dummy_slam_broadcaster` is only needed if you want to display the sensors data using the QuickStart method mentioned below.

### Published Topics

- `scan` (sensor_msgs/Laserscan) A standard ROS Laserscan message containing the measured data. The message rate per second depends on the `scan_frequency` parameter.

### Parameters

- `frame_id` The frame-ID in the Header of the published `sensor_msgs/Laserscan2` messages
- `scanner_ip` IP or hostname of the laserscanner
- `scan_frequency` The scan frequency (rotation speed of scanner head) in Hz in the range [10;50]
- `samples_per_scan` The number of distances measured per scan/rotation in the range [72;25200]. Only certain values are allowed (see Manual). Examples are 72, 360, 720, 1440, 1800, 3600, 7200, 10080, 25200.

### Quick Start

Copy the driver in your ROS workspace and compile it.
Set the IP-Address of the scanner in `pepperl_fuchs_r2000/launch/gui_example.launch` and run the following command:

    roslaunch pepperl_fuchs_r2000 gui_example.launch

This starts `RViz` (http://wiki.ros.org/rviz) and the driver and you should see the measuring output of the scanner.

Basic usage without ROS
---------------------------
There exists a file `CMakeLists.txt.NO_ROS_LIB_ONLY` in the `pepperl_fuchs_r2000` directory.
Replace `CMakeLists.txt` with it to compile the driver without ROS:

    $ cd pepperl_fuchs_r2000
    $ mv CMakeLists.txt.NO_ROS_LIB_ONLY CMakeLists.txt
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

This builds a SHARED library which can be used in your program. 
To build a static library remove the `SHARED` in the `add_library` command in the `CMakeLists.txt`.

### The basic usage of the driver library code (C++11 style) is as follows:

    #include <pepperl_fuchs_r2000/r2000_driver.h>

    int main(int argc, char **argv)
    {
      bool success;

      pepperl_fuchs::R2000Driver driver;
      success = driver.connect("192.168.0.100"); // Replace IP
      success = driver.setScanFrequency(35);     // Set scanner frequency in the range [10;50]
      success = driver.setSamplesPerScan(3600);  // Set samples per scan in the range [72,25200] (valid values are listed in manual)

      auto params = driver.getParameters();      // Get all parameter values as std::map<string, string>
      for( auto key_value : params )
      { Do something with the parameter values }

      success = driver.startCapturingUDP();      // Notice: startCapturingTCP() also exists

      while(true)
      {
          pepperl_fuchs::ScanData  scandata = driver.getFullScan(); // Do something with:
          scandata.headers;                                         // headers,
          scandata.distance_data;                                   // distances and
          scandata.amplitude_data;                                  // amplitudes
      }

      driver.stopCapturing();
      driver.disconnect();
    }

More documentation
-------------------------
The driver is commented in doxygen style. 
You can create a latex and html documentation in the `pepperl_fuchs_r2000/doxygen` directory 
by entering the following command in the `pepperl_fuchs_r2000` directory:

    $ doxygen doxygen.conf
    

