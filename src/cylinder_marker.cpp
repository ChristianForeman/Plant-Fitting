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

    cube_marker.header.frame_id = "camera_depth_optical_frame"; 
    cube_marker.header.stamp = ros::Time::now();

    cube_marker.color.a = 0.3;

    double radius = 0.0288058;
    cube_marker.scale.x = radius * 2;
    cube_marker.scale.y = radius * 2;
    cube_marker.scale.z = 10;

    // cube_marker.pose.position.x = -0.44714;
    // cube_marker.pose.position.y = 0.5448;
    // cube_marker.pose.position.z =  0.01454;

    // cube_marker.pose.orientation.x = 0.9677;
    // cube_marker.pose.orientation.y = -0.0358;
    // cube_marker.pose.orientation.z = 1.04719 + 0.24933; // 0.24933

    cube_marker.pose.position.x = 1.03729;
    cube_marker.pose.position.y = 0.4332;
    cube_marker.pose.position.z =  0.3988;
    // why do we add one to the z orientation???
    cube_marker.pose.orientation.x = -0.9644;
    cube_marker.pose.orientation.y = 0.06838;
    cube_marker.pose.orientation.z = 1 - 0.255243; // 0.24933
    while (pub.getNumSubscribers() < 1) {
        if (!ros::ok())
        {
            return -1;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }
    std::cout << "Publishing" << std::endl;
    int i = 0;
    while( i < 50) {
        pub.publish(cube_marker);
        ++i;
    }
}