/*
  * File: flight_main_node.cpp
  * Date: 05/2018
  * Authors: Nate Hamilton & Arda Turkman
  *
  * Description:
  *
  *
  */
/*************************************************************************
 * Import necessary packages.
 *************************************************************************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <queue>

/************************************************************************
 * Define hard coded values.
 * These can be changed based on the test location.
 ************************************************************************/
#define FLIGHT_ALTITUDE             5.0    // The altitude
#define SEARCH_CENTER_LAT           32.2656763   // The GPS latitude val at the center of the search ellipse
#define SEARCH_CENTER_LONG          -111.2734111   // The GPS longitude val at the center of the search ellipse
#define DROPOFF_LONGITUDE           -111.2725171
#define DROPOFF_LATITUDE            32.2650528
#define LANDING_LONGITUDE           -111.2736179
#define LANDING_LATITUDE            32.2652491
#define REPEATED_MSG_THRESHOLD      10 // The number of times the target location needs to be duplicated to assume it is true
#define REPEATED_VAL_THRESHOLD      0.1 // The difference between 2 consecutive locations from IP that determines if it is a repeat
#define GENERAL_THRESHOLD           0.5f
#define PRECISE_THRESHOLD           0.1f
#define SEARCH_HORIZONTAL_RADIUS    20
#define SEARCH_VERTICAL_RADIUS      20
//TODO: Add more hardcoded values

/************************************************************************
 * Global variables are listed here.
 ************************************************************************/
mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_pose;
geometry_msgs::PoseStamped curr_pose;
bool home_set = false;
mavros_msgs::HomePosition home{};
int found = 0;
int repeated_location = 0;
float IP_location_x = 0;
float IP_location_y = 0;

/************************************************************************
 * All of the helper functions are listed here, but are written out
 * after the main function.
 ************************************************************************/
bool arm_the_drone(const ros::NodeHandle& nh, ros::ServiceClient& sc);
std::queue <geometry_msgs::PoseStamped> calculateRoute(double horizontal_radius, double vertical_radius);
void drop_off_marker(const ros::NodeHandle& nh);
bool land_the_drone(const ros::NodeHandle& nh, ros::ServiceClient& sc);
void move_drone_to_location_local(const ros::NodeHandle& nh,
                        const ros::Publisher& pub, float x, float y, float z, float threshold);
void move_drone_to_location_global(const ros::NodeHandle& nh,
                        const ros::Publisher& pub, float longitude, float latitude, float altitude, float threshold);
void pick_up_marker(const ros::NodeHandle& nh);
bool resetHome(const ros::NodeHandle& nh);
bool takeoff(const ros::NodeHandle& nh, ros::ServiceClient& sc);

/************************************************************************
 * All of the callback functions are listed here, but are written after
 * the helper functions.
 ************************************************************************/
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void marker_find_cb(const std_msgs::Int32::ConstPtr& msg);
void marker_position_cb(const std_msgs::Float64MultiArray::ConstPtr& msg);
void setHomeGeoPointCB(const mavros_msgs::HomePosition::ConstPtr& msg);
void state_cb(const mavros_msgs::State::ConstPtr& msg);

/************************************************************************
 * The main code.
 ************************************************************************/
int main(int argc, char **argv)
{
    /* Initialize the ROS node. */
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    home.geo.latitude = home.geo.longitude = home.geo.altitude = NAN;
	
    /* Setup all subscribers. */
	ROS_INFO("Setting up subscribers.");
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("/mavros/global_position/global", 10, global_pos_cb);
    ros::Subscriber marker_find_sub = nh.subscribe<std_msgs::Int32>
        ("/navigation/marker_find", 10, marker_find_cb);
    ros::Subscriber marker_position_sub = nh.subscribe<std_msgs::Float64MultiArray>
        ("/navigation/marker_position", 10, marker_position_cb);
		
    /* Setup all publishers. */
	ROS_INFO("Setting up publishers.");
    ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("/mavros/setpoint_position/global", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
			
    /* Set the publishing rate (MUST be faster than 2Hz) */
    ros::Rate rate(20.0);
	
    /* Set up service clients. */
	ROS_INFO("Setting up service clients.");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/takeoff");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
            
			
    /* Calculate the desired search route. */
	ROS_INFO("Calculating the search route.");
    std::queue <geometry_msgs::PoseStamped> wayPoints = calculateRoute(SEARCH_HORIZONTAL_RADIUS, SEARCH_VERTICAL_RADIUS);
	ROS_INFO("Search route calculated.");
	
    /* Wait for the RC controller to give up control. */
    while( current_state.mode == "POSCTL" && ros::ok()){
        //wait
		ROS_INFO("Waiting for the mode to be switched to OFFBOARD.");
    }
	
    /* Establish a safe home position. */
    ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("/mavros/home_position/home", 10, setHomeGeoPointCB);
    ROS_INFO("Waiting for Aero FC Home to be set...");
    while (ros::ok() && !home_set)
    {
        ros::spinOnce();
        rate.sleep();
    }
	
    /* Take off! */
	ROS_INFO("Arming the motors.");
    if(!arm_the_drone(nh, arming_client)){
        ROS_ERROR("Unable to arm the motors");
        ros::shutdown();
        return 1;
    }
	ROS_INFO("Taking off.");
    if(!takeoff(nh, takeoff_client)){
        ROS_ERROR("Unable to takeoff");
        ros::shutdown();
        return 1;
    }
	
    /* Make sure the drone gets changed to OFFBOARD mode to continue. */
	ROS_INFO("Making sure the mode is OFFBOARD");
    ros::Publisher mode_pub = nh.advertise<mavros_msgs::State>
            ("/mavros/state", 10);
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        mode_pub.publish(curr_pose);
        ros::spinOnce();
        rate.sleep();
    }
    //Set mode request
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();

    // change to offboard mode
    while( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request >
                                                ros::Duration(5.0)))
    {
        ROS_INFO(current_state.mode.c_str());
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
        mode_pub.publish(curr_pose);
        ros::spinOnce();
        rate.sleep();
    }
	
    /* Move to the starting location. */
    ROS_INFO("Moving to center of search area: %lf, %lf, %lf", SEARCH_CENTER_LONG, SEARCH_CENTER_LAT, FLIGHT_ALTITUDE);
    move_drone_to_location_global(nh, global_pos_pub, SEARCH_CENTER_LONG, SEARCH_CENTER_LAT, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
    //move_drone_to_location_global(nh, global_pos_pub, home.geo.longitude, home.geo.latitude, FLIGHT_ALTITUDE, GENERAL_THRESHOLD); //for sim
	
    // Get the local position of the ellipse center for route following to work
    geometry_msgs::PoseStamped search_center_pose = curr_pose;
	
    /* Search the area following specified paths until the marker is found. */
	while(repeated_location <= REPEATED_MSG_THRESHOLD && !wayPoints.empty() && ros::ok()){
		/* Follow the route until the IP side says they found the marker. */
		while(!found && !wayPoints.empty() && ros::ok()){
			if(current_state.mode == "OFFBOARD" && ros::ok()){
				ROS_INFO("Moving to next waypoint: %lf, %lf, %lf", (search_center_pose.pose.position.x + wayPoints.front().pose.position.x),
						 (search_center_pose.pose.position.y + wayPoints.front().pose.position.y), FLIGHT_ALTITUDE);
				move_drone_to_location_local(nh, local_pos_pub, (search_center_pose.pose.position.x + wayPoints.front().pose.position.x),
					(search_center_pose.pose.position.y + wayPoints.front().pose.position.y), FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
				wayPoints.pop();
			}
		}
		
		/* Follow IP commands until there are no more updates. */
		while(found && repeated_location <= REPEATED_MSG_THRESHOLD && ros::ok()){
			if(current_state.mode == "OFFBOARD" && ros::ok()){
				ROS_INFO("Moving to next specified location: %lf, %lf, %lf", (curr_pose.pose.position.x + IP_location_x),
						 (curr_pose.pose.position.y + IP_location_y), FLIGHT_ALTITUDE);
				move_drone_to_location_local(nh, local_pos_pub, (curr_pose.pose.position.x + IP_location_x),
					(curr_pose.pose.position.y + IP_location_y), FLIGHT_ALTITUDE, PRECISE_THRESHOLD);
			}
		}
    }
	
    /* Pick up the marker. */
    pick_up_marker(nh);
	
    /* Fly to the drop off location. */
    ROS_INFO("Moving to dropoff area: %lf, %lf, %lf", DROPOFF_LONGITUDE, DROPOFF_LATITUDE, FLIGHT_ALTITUDE);
    move_drone_to_location_global(nh, global_pos_pub, DROPOFF_LONGITUDE, DROPOFF_LATITUDE, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
    //move_drone_to_location_global(nh, global_pos_pub, home.geo.longitude, home.geo.latitude, FLIGHT_ALTITUDE, GENERAL_THRESHOLD); //for sim
	
    /* Execute drop off manuever. */
    drop_off_marker(nh);
	
    /* Fly to landing location. */
    ROS_INFO("Moving to landing area: %lf, %lf, %lf", LANDING_LONGITUDE, LANDING_LATITUDE, FLIGHT_ALTITUDE);
    move_drone_to_location_global(nh, global_pos_pub, LANDING_LONGITUDE, LANDING_LATITUDE, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
    //move_drone_to_location_global(nh, global_pos_pub, home.geo.longitude, home.geo.latitude, FLIGHT_ALTITUDE, GENERAL_THRESHOLD); //for sim
	
    /* Land the drone. */
    if(!land_the_drone(nh, land_client)){
        ROS_ERROR("Landing failed");
        ros::shutdown();
        return 1;
    }
	
    /* Code is complete, so shut down. */
    ros::shutdown();
    return 0;
}

/************************************************************************
 * Helper Functions:
 ************************************************************************/
// This function arms the drone's motors in preparation for takeoff.
bool arm_the_drone(const ros::NodeHandle& nh, ros::ServiceClient& sc){
    if (!home_set)
    {
        ROS_ERROR("Can't arm: No GPS Fix!");
        return false;
    }
    mavros_msgs::CommandBool srv_arm;
    srv_arm.request.value = true;
    if (sc.call(srv_arm) && srv_arm.response.success) {
        ROS_INFO("Vehicle armed");
        return true;
    }else {
        return false;
    }
}
// This function generates the search route
std::queue <geometry_msgs::PoseStamped> calculateRoute(double horizontal_radius, double vertical_radius){
    // Author of this function: Arda Turkmen
    std::queue <geometry_msgs::PoseStamped> wayPoints;//Queue
    geometry_msgs::PoseStamped pose;//Point to be added
    pose.pose.position.z = FLIGHT_ALTITUDE;//All of them have the same z value
    double k = 50;//Directly proportional to amount of lapses spiral make in ellipse.
    double s = 5;//Inversely proportional to the distance between each point.
    double dt = 1;//Used to put similar distance between each point. DO NOT PARAMETRIZE
    double t = 0;//Since it's a parametric equation t is the only variable in this equation rest
    // are constant in the same ellipse. DO NOT PARAMETRIZE
    while(t<k){
       pose.pose.position.x = (horizontal_radius*t*cos(t))/k; //Calculate x
       pose.pose.position.y = (vertical_radius*t*sin(t))/k; //Calculate y
       wayPoints.push(pose); //Add the point to the queue
       t+= M_PI/(dt*s);
       dt = t/M_PI + 1;
    }
    return wayPoints;
}
// This function executes a manuever for dropping off the marker
void drop_off_marker(const ros::NodeHandle& nh){
    //TODO
}
// This function executes the onboard landing manuever
bool land_the_drone(const ros::NodeHandle& nh, ros::ServiceClient& sc){
    if (!home_set)
    {
        ROS_ERROR("Can't land: No GPS Fix!");
        return false;
    }
    mavros_msgs::CommandTOL srv_land{};
    ros::Rate rate(20.0);
    ROS_INFO("Trying to land");
    if (sc.call(srv_land) && srv_land.response.success) {
        ROS_INFO("Landing");
        for(int i = 0; ros::ok() && i < 5*20; ++i){//Giving it enough time to land
            ros::spinOnce();
            rate.sleep();
        }
        return true;
    }else {
        return false;
    }
}
// This functions moves the drone to the desired local position
void move_drone_to_location_local(const ros::NodeHandle& nh,
                        const ros::Publisher& pub, float x, float y, float z, float threshold){
    ros::Rate rate(20.0);
    // Create a message with the desired location
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    // Until we have reached the location, we keep publishing the desired location
    while(std::abs(curr_pose.pose.position.x - x) > threshold &&
          std::abs(curr_pose.pose.position.y - y) > threshold &&
          std::abs(curr_pose.pose.position.z - z) > threshold &&
          ros::ok()){
        // If the mode is switched, then motion needs to be halted. Ergo, we stop publishing
        if(current_state.mode == "OFFBOARD" && ros::ok()){
            pub.publish(target);
            ros::spinOnce();
            rate.sleep();
        }
    }
}
// This function moves the drone to the desired global position
void move_drone_to_location_global(const ros::NodeHandle& nh,
                        const ros::Publisher& pub, float longitude, float latitude, float altitude, float threshold){
    ros::Rate rate(20.0);
    // Create a message with the desired location
    mavros_msgs::GlobalPositionTarget target;
    target.latitude = longitude;
    target.longitude = latitude;
    target.altitude = home.geo.altitude + altitude;
    // Until we have reached the location, we keep publishing the desired location
    while(std::abs(global_pose.longitude - longitude) > threshold &&
          std::abs(global_pose.latitude - latitude) > threshold &&
          std::abs(global_pose.altitude - altitude) > threshold &&
          ros::ok()){
        // If the mode is switched, then motion needs to be halted. Ergo, we stop publishing
        if(current_state.mode == "OFFBOARD" && ros::ok()){
            pub.publish(target);
            ros::spinOnce();
            rate.sleep();
        }
    }
}
// This function executes a manuever for picking up the marker
void pick_up_marker(const ros::NodeHandle& nh){
    //TODO
}
// This function causes the drone to takeoff
bool takeoff(const ros::NodeHandle& nh, ros::ServiceClient& sc){
    if (!home_set)
    {
        ROS_ERROR("Can't takeoff: No GPS Fix!");
        return false;
    }
    //Takeoff Service
    mavros_msgs::CommandTOL srv_takeoff;
    ros::Rate rate(20.0);
    srv_takeoff.request.altitude = home.geo.altitude + FLIGHT_ALTITUDE;
    srv_takeoff.request.latitude = home.geo.latitude;
    srv_takeoff.request.longitude = home.geo.longitude;
    //Actually takeoff
    ROS_INFO("Trying to takeoff");
    if (sc.call(srv_takeoff) && srv_takeoff.response.success) {
        ROS_INFO("Taking off");
        for(int i = 0; ros::ok() && i < 6*20; ++i){//Giving it enough time to take off
            ros::spinOnce();
            rate.sleep();
        }
        return true;
    } else {
        return false;
    }
}
/************************************************************************
 * Callback Functions:
 ************************************************************************/
// For getting the global position
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    global_pose = *msg;
    return;
}
// For getting the local position
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    curr_pose = *msg;
    return;
}
// For noting whether or not the marker has been found
void marker_find_cb(const std_msgs::Int32::ConstPtr& msg){
    std_msgs::Int32 temp = *msg;
    found = temp.data;
    return;
}
// For recording the locations reported by the image processing node
void marker_position_cb(const std_msgs::Float64MultiArray::ConstPtr& msg){
    std_msgs::Float64MultiArray temp = *msg;
    if(found){
        if((std::abs(temp.data[0] - IP_location_x) <= REPEATED_VAL_THRESHOLD) &&
            (std::abs(temp.data[1] - IP_location_y) <= REPEATED_VAL_THRESHOLD)){
                repeated_location++;
            }
        else{
            IP_location_x = temp.data[0];
            IP_location_y = temp.data[1];
        }
    }
    return;
}
//Home position subscriber callback
void setHomeGeoPointCB(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    home = *msg;
    home_set = true;
    ROS_INFO("Received Home (WGS84 datum): %lf, %lf, %lf", home.geo.latitude, home.geo.longitude, home.geo.altitude);
}
// For monitoring the drone's state/mode
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    return;
}