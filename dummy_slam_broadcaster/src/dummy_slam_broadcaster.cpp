#include "dummy_slam_broadcaster.h"

//-------------------------------------------------------------------------------------------------
DummySLAMBroadcaster::DummySLAMBroadcaster():node_handle_("~")
{
    map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("/map",1);
    pub_map_timer_ = node_handle_.createTimer(ros::Duration(1.0), &DummySLAMBroadcaster::broadcastMap, this);

    node_handle_.param("broadcast_zero_pose_enabled",broadcast_zero_pose_enabled_,1);
    if(broadcast_zero_pose_enabled_)
    {
        pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/slam_out_pose",10);
        pub_pose_timer_ = node_handle_.createTimer(ros::Duration(0.05), &DummySLAMBroadcaster::broadcastPose, this);
    }
}

//-------------------------------------------------------------------------------------------------
void DummySLAMBroadcaster::broadcastMap(const ros::TimerEvent &e)
{
    empty_map_.header.stamp = ros::Time::now();
    map_publisher_.publish( empty_map_ );
}

//-------------------------------------------------------------------------------------------------
void DummySLAMBroadcaster::broadcastPose(const ros::TimerEvent &e)
{
    tf::Transform t;
    t.setOrigin( tf::Vector3(0,0,0) );
    t.setRotation(tf::Quaternion( 0,0,0,1 ));
    tf_pose_broadcaster_.sendTransform(tf::StampedTransform(t,ros::Time::now(),"/map","/scan"));

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    pose.pose.orientation.w = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;

    pose.header.frame_id = "/map";
    pose.header.stamp = ros::Time::now();

    pose_publisher_.publish(pose);
}

//-------------------------------------------------------------------------------------------------
void DummySLAMBroadcaster::initMap()
{
    const std::size_t mapSize = 2500;
    empty_map_.header.frame_id = "/map";
    empty_map_.info.resolution = 0.04;
    empty_map_.info.width = mapSize;
    empty_map_.info.height = mapSize;
    empty_map_.info.origin.position.x = -50;
    empty_map_.info.origin.position.y = -50;
    empty_map_.info.origin.position.z = 0;

    empty_map_.info.origin.orientation.x = 0;
    empty_map_.info.origin.orientation.y = 0;
    empty_map_.info.origin.orientation.z = 0;
    empty_map_.info.origin.orientation.w = 1;

    empty_map_.data.resize(mapSize*mapSize,-1);
}

//-------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_slam_broadcaster");
    new DummySLAMBroadcaster();
    ros::spin();
    return 0;
}
