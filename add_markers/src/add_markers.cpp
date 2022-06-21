#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Make object marker object available globally
visualization_msgs::Marker marker;

bool flag_near_pickup_point = false;
bool flag_near_dropoff_point = false;
bool flag_mission_status_complete = false;

void initializeObjectMarker()
{
    // Set our initial shape type to be a SPHERE
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "target_object";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    // Set the marker type.
    marker.type = shape;
}

void setObjectMarkerViz (float a = 1.0, float r = 0.62f, float g = 0.12f, float b = 0.94f)
{
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

void setObjectMarkerPose (float x, float y, float z = 0.0f, float orientation_x = 0.0f, float orientation_y = 0.0f, float orientation_z = 0.0f, float orientation_w = 1.0)
{
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = x;
    marker.pose.orientation.x = orientation_x;
    marker.pose.orientation.y = orientation_y;
    marker.pose.orientation.z = orientation_z;
    marker.pose.orientation.w = orientation_w;

    setObjectMarkerViz();
    marker.lifetime = ros::Duration();
}

double distanceToMarker(double x1, double y1, double x2, double y2){
    return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

void checkRobotToMarkerVicinity(const nav_msgs::Odometry &odom_msg)
{
    double pose_x = odom_msg.pose.pose.position.y;
    double pose_y = -odom_msg.pose.pose.position.x; // is "inverted"
    double distToPickup, distToDropoff;
    const float minDistError = 0.2f;

    distToPickup = distanceToMarker(position_x, position_y, pickup_zone_x, pickup_zone_y);
    distToDropoff = distanceToMarker(position_x, position_y, drop_off_zone_x, drop_off_zone_y);

    flag_near_pickup_point = false;
    flag_near_dropoff_point = false;

    if (distToPickup <= minDistError)
    {
        flag_near_pickup_point = true;
        ROS_INFO("INFO: Robot is near pickup point");
    }
    if (distToDropoff <= minDistError)
    {
        flag_near_dropoff_point = true;
        ROS_INFO("INFO: Robot is near drop-off point");
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odom_sub = n.subscribe("odom", 1, checkRobotToMarkerVicinity);

    const float pickup_pose_x = -3.25f, pickup_pose_y = -4.50f, dropoff_pose_x = -4.50f, dropoff_pose_y = 7.25f;
    ROS_INFO("INFO: Pickup  point: pose.x: %5.2f , pose.y: %5.2f", pickup_pose_x, pickup_pose_y);
    ROS_INFO("INFO: Drop-off point: pose.x: %5.2f , pose.y: %5.2f", dropoff_pose_x, dropoff_pose_y);

    while (ros::ok())
    {
        initializeObjectMarker();
        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
            ROS_WARN_ONCE("WARN: Please create a subscriber to the object marker");
            sleep(1);
        }
        ROS_INFO("INFO: Spawning object at pickup point");
        // Spawn object at pickup point
        setObjectMarkerPose(pickup_pose_x, pickup_pose_y);
        setObjectMarkerViz(1.0f);
        marker_pub.publish(marker);
        // Wait for robot to reach pickup point
        while (!flag_near_pickup_point) ros::spinOnce();
        ROS_INFO("INFO: Object is being picked up");
        ros::Duration(5.0).sleep();
        setObjectMarkerViz(0.0f);
        marker_pub.publish(marker);
        ROS_INFO("INFO: Object has been picked up, in transit");
        // Wait for robot to reach dropoff point
        while (!flag_near_dropoff_point) ros::spinOnce();
        setObjectMarkerPose(dropoff_pose_x, dropoff_pose_y);
        setObjectMarkerViz(1.0f);
        marker_pub.publish(marker);
        ROS_INFO("INFO: Object moved successfully");

        flag_mission_status_complete = true;
        if (flag_mission_status_complete)
        {
            ROS_INFO("INFO: Shutting down add_markers node");
            ros::Duration(10.0).sleep();
            break;
        }

        r.sleep();
    }
}
