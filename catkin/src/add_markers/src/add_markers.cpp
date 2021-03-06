#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/setupMarker.h"

ros::Publisher marker_pub;
ros::ServiceServer marker_srv;


// setup marker with requested position and visibility
bool handle_setupMarker_request (add_markers::setupMarker::Request& req,
                                    add_markers::setupMarker::Response& res){


        //request print
        ROS_INFO("GoToPositionRequest received - position_x:%1.2f, position_y:%1.2f, oppacity:%1.2f", 
                                     (float)req.position_x, (float)req.position_y, (float)req.oppacity);

	// Create a marker object of type visualization_msgs::Marker
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "cube";
        marker.id = 0;

        // Set the marker type as a CUBE. Other possibilities: SPHERE, ARROW, and CYLINDER
        marker.type = visualization_msgs::Marker::CUBE;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;	

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = req.position_x;
        marker.pose.position.y = req.position_y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = req.oppacity;

        marker.lifetime = ros::Duration();



        // Publish the marker
        marker_pub.publish(marker);

        // Return a response message
        res.msg_feedback = "Marker Published - position_x: " + std::to_string(marker.pose.position.x) +
                              " , position_y: " + std::to_string(marker.pose.position.y) + ", oppacity: " + std::to_string(marker.color.a);
        ROS_INFO_STREAM(res.msg_feedback);


   return true;

}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "add_markers");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;
    ros::Rate r(1);

    // Inform ROS master that we will be publishing a message of type visualization_msgs::Marker
    // on the visualization_marker topic with a publishing queue size of 2
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 2);
    
    // Define a setupMarker service with a handle_setupMarker_request callback function
    marker_srv = n.advertiseService("/add_marker/setupMarker", handle_setupMarker_request);
    ROS_INFO("Ready to use setupMarker service");


    // Handle ROS communication events
    ros:: spin();
    return 0;
} 
