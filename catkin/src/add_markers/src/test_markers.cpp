#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


ros::Publisher marker_pub;

// Set initial shape type to be a cube
uint32_t cube = visualization_msgs::Marker::CUBE;

// set initial position and oppacity
float pickUp_x = 2.8;
float pickUp_y = 5;
float dropOff_x = -1.9
float dropOff_y = 1.29
float oppacity_on = 1.0;
float oppacity_off = 0.0;

// setup marker with requested position and visibility
bool setupAndPublishMarker (float position_x, float position_y, float oppacity){

	// Create a marker object of type visualization_msgs::Marker
        visualization_msgs::Marker marker;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/odom";
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "cube";
        marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = cube;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;	

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = position_x;
        marker.pose.position.y = position_y;
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
        marker.color.a = oppacity;

        marker.lifetime = ros::Duration();



        // Publish the marker
        while (marker_pub.getNumSubscribers() < 1)
        {
           if (!ros::ok())
           {
              return 0;
           }
         ROS_WARN_ONCE("Please create a subscriber to the marker");
         sleep(1);
        }
        marker_pub.publish(marker);

   return true;

}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "add_markers");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;
    ros::Rate r(1);

    // Inform ROS master that we will be publishing a message of type visualization_msgs::Marker on the robot actuation topic with a publishing queue size of 2
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 2);
    
    setupAndPublishMarker(pickUp_x, pickUp_y,oppacity_on);
    sleep(5) 
    setupAndPublishMarker(pickUp_x, pickUp_y,oppacity_off);
    sleep(5)
    setupAndPublishMarker(dropOff_x, dropOff_y,oppacity_on);


    // Handle ROS communication events
    ros:: spin();
    return 0;
}
