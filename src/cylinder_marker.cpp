#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// its called cube_marker instead of cylinder because i'm lazy
int main(int argc, char ** argv) {
    ros::init(argc, argv, "cyl_publisher");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("/ransac/cyl_marker", 1);   
    visualization_msgs::Marker cube_marker;
        
    cube_marker.ns = "cube";
    cube_marker.id = 0;

    cube_marker.type = visualization_msgs::Marker::CYLINDER;
    cube_marker.color.r = 0.0;
    cube_marker.color.g = 0.5;
    cube_marker.color.b = 0.5;

    cube_marker.header.frame_id = "camera_rviz"; 
    cube_marker.header.stamp = ros::Time::now();

    cube_marker.color.a = 0.3;

    cube_marker.scale.x = 2;
    cube_marker.scale.y = 2;
    cube_marker.scale.z = 2;

    cube_marker.pose.position.x = 3.4832;
    cube_marker.pose.position.y = 0.507;
    cube_marker.pose.position.z =  1.05535;

    cube_marker.pose.orientation.x = -0.96;
    cube_marker.pose.orientation.y = 0.01;
    cube_marker.pose.orientation.z = -0.25;

    while (pub.getNumSubscribers() < 1) {
        if (!ros::ok())
        {
            return -1;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    pub.publish(cube_marker);
}