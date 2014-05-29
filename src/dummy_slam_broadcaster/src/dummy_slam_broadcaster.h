#ifndef DUMMY_SLAM_BROADCASTER_H
#define DUMMY_SLAM_BROADCASTER_H
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>

//! \class EmptyMapBroadcaster
//! \brief Provides an empty map with optionally a broadcasted base_footprint pose at position zero
class DummySLAMBroadcaster
{
public:
    DummySLAMBroadcaster();
    void broadcastMap( const ros::TimerEvent& e );
    void broadcastPose( const ros::TimerEvent& e );

private:
    void initMap();

    ros::NodeHandle node_handle_;

    int broadcast_zero_pose_enabled_;
    ros::Publisher map_publisher_;
    ros::Publisher pose_publisher_;
    ros::Timer pub_map_timer_;
    ros::Timer pub_pose_timer_;

    nav_msgs::OccupancyGrid empty_map_;

    tf::TransformBroadcaster tf_pose_broadcaster_;
};

#endif // DUMMY_SLAM_BROADCASTER_H
