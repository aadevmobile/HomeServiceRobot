# Home Service Robot
Udacity Robotics Engineering Course - Final Assignment

## Summary
This assignment consists on a creation of a simple environment where a robot - turtlebot - will navigate to a defined pick-up location, retrieve a virtual object,
and deliver it to a defined drop-off location using ROS navigation stack
The student should include a brief write-up explaining the packages used for this project, covering localization, mapping and navigation.

**Required external packages**
- [gmapping](http://wiki.ros.org/gmapping)
- [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)
- [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
- [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)


## Mapping
gmapping perform SLAM in order to build a map of the environment,
using test_slam.sh it's possible to manually drive the robot using turtlebot_teleop while visualizing the map exploration in rviz.


## Localization and Navigation
AMCL and ROS Navigation stack are used to have the robot localize itself and plan a path towards a specific goal while avoiding obstacles.
using test_navigation.sh it's possible to select different 2D Nav Goals in rviz and watch the robot define and follow a trajectory.


## Multiple goals
a new node - pickup_objects - is created in order to have pre-defined multiple goals in the map where the robot will execute automatically
pickup_objects.sh will launch the robot world, leverage localization and navigation in order to move towards the pre-defined multiple goals.
note: pick-up and drop-off coordinates are hardcoded in pickup_objects.cpp


## Markers
In order to simulate objects the robot should retrieve and deliver markers are added to the map.
add_markers.sh will add a cube to the pick-up location, disappear after 5 seconds and reapear in the drop-off location.


## Interface Nodes from Different Packages
To achieve the ultimate goal and have the markers appear, disappear when the robot arrives at the pick-up location 
and reapear when the robot reaches its drop-off location a setupMarker service was created. The integration is done
by raising a service client within pick-up node and called when necessarry to show, hide, change marker position.
home_service.sh will launch all required nodes and services, place a marker in the pick-up location, hide it when 
the robot arrives and show-up again once the virtual object its delivered to its destination.



