 /*
  * File: flight_main_node.cpp
  * Date: 05/2018
  * Authors: Nate Hamilton & Arda Turkman
  *
  * Description: 
  *
  *  
  */
  
/* Import necessary packages. */
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include <queue>

/* Define hard coded values. These can be changed based on the test location. */
#define FLIGHT_ALTITUDE     1.5f    // The altitude
#define SEARCH_CENTER_LAT   0.00f   // The GPS latitude val at the center of the search ellipse
#define SEARCH_CENTER_LONG  0.00f   // The GPS longitude val at the center of the search ellipse
#define DROPOFF_LONGITUDE
#define DROPOFF_LATITUDE
#define LANDING_LONGITUDE
#define LANDING_LATITUDE
#define REPEATED_MSG_THRESHOLD 0.00f // The number of times the target location needs to be duplicated to assume it is true
#define REPEATED_VAL_THRESHOLD 0.00f // The difference between 2 consecutive locations from IP that determines if it is a repeat
#define GENERAL_THRESHOLD
#define PRECISE_THRESHOLD

//TODO: Add more hardcoded values

/* Global variables are listed here. */
mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_pose;
geometry_msgs::PoseStamped curr_pose;
std_msgs::Int32 found;
int count = 0;

/* All of the helper functions are listed here, but are written out after the main function. */
void arm_the_drone(const ros::NodeHandle::ConstPtr& nh);
std::queue <geometry_msgs::PoseStamped> calculateRoute();
void drop_off_marker(const ros::NodeHandle::ConstPtr& nh);
void move_drone_to_location_local(const ros::NodeHandle::ConstPtr& nh, 
			const geometry_msgs::PoseStamped::ConstPtr& pub, float x, float y, float z, float threshold);
void move_drone_to_location_global(const ros::NodeHandle::ConstPtr& nh, 
			const mavros_msgs::GlobalPositionTarget::ConstPtr& pub, float longitude, float latitude, float altitude, float threshold);
void pick_up_marker(const ros::NodeHandle::ConstPtr& nh);
void takeoff(const ros::NodeHandle::ConstPtr& nh);

/* All of the callback functions are listed here, but are written after the helper functions. */
void global_pos_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void marker_find_cb(const std_msgs::Int32::ConstPtr& msg);
void marker_position_cb(std_msgs::Float64MultiArray::ConstPtr& msg);
void state_cb(const mavros_msgs::State::ConstPtr& msg);

int main(int argc, char **argv)
{
	/* Initialize the ROS node. */
	ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
	
    /* Setup all subscribers. */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/global", 10, global_pos_cb);
	ros::Subscriber _sub = nh.subscribe<std_msgs::Int32>
            ("/navigation/marker_find", 10, marker_find_cb);
	ros::Subscriber _sub = nh.subscribe<std_msgs::Float64MultiArray>
            ("/navigation/marker_position", 10, marker_position_cb);
    
	/* Setup all publishers. */
	ros::Publisher global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("mavros/setpoint_position/global", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    /* Set the publishing rate (MUST be faster than 2Hz) */
    ros::Rate rate(20.0);
	
    /* Calculate the desired search route. */
	std::queue <geometry_msgs::PoseStamped> wayPoints = calculateRoute();
	
	/* Wait for the RC controller to give up control. */
	while( current_state.mode != "OFFBOARD"){
		//wait
	}
	
	/* Take off! */
	arm_the_drone(nh);
	takeoff(nh);
	
	/* Move to the starting location. */
	move_drone_to_location_global(nh, global_pos_pub, SEARCH_CENTER_LONG, SEARCH_CENTER_LAT, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
	

	/* Search the area following specified paths until the marker is found. */
	i = 0;
	while(repeated_location <= REPEATED_MSG_THRESHOLD){
		/* Follow the route until the IP side says they found the marker. */
		while(!found && !wayPoints.isEmpty()){
			if(current_state.mode == "OFFBOARD"){
				move_drone_to_location_local(nh, local_pos_pub, float x, float y, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
				i++;
			}
		}
		
		/* Follow IP commands until there are no more updates. */
		while(found && repeated_location <= REPEATED_MSG_THRESHOLD){
			if(current_state.mode == "OFFBOARD"){
				move_drone_to_location_local(nh, local_pos_pub, float x, float y, float z, PRECISE_THRESHOLD);
			}
		}
	}
	
	/* Pick up the marker. */
	pick_up_marker(nh);
	
	/* Fly to the drop off location. */
	move_drone_to_location_global(nh, global_pos_pub, DROPOFF_LONGITUDE, DROPOFF_LATITUDE, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
	
	/* Execute drop off manuever. */
	drop_off_marker(nh);
	
	/* Fly to landing location. */
	move_drone_to_location_global(nh, global_pos_pub, LANDING_LONGITUDE, LANDING_LATITUDE, FLIGHT_ALTITUDE, GENERAL_THRESHOLD);
	
	/* Land the drone. */
	land_the_drone(nh);
	
}

/* Helper Functions: */
void arm_the_drone(const ros::NodeHandle::ConstPtr& nh){
	//TODO
}

std::queue <geometry_msgs::PoseStamped> calculateRoute(){
	//TODO
}

void drop_off_marker(const ros::NodeHandle::ConstPtr& nh){
	//TODO
}

void move_drone_to_location_local(const ros::NodeHandle::ConstPtr& nh, 
			const geometry_msgs::PoseStamped::ConstPtr& pub, float x, float y, float z, float threshold){
	geometry_msgs::PoseStamped target;
	target.pose.position.x = x;
	target.pose.position.y = y;
	target.pose.position.z = z;

	while(abs(curr_pose.pose.position.x - x) > threshold && 
		  abs(curr_pose.pose.position.y - y) > threshold && 
		  abs(curr_pose.pose.position.z - z) > threshold){
		if(current_state.mode == "OFFBOARD"){
			publisher.publish(target);
			ros::spinOnce();
			rate.sleep();
		}
	}    		
}

void move_drone_to_location_global(const ros::NodeHandle::ConstPtr& nh, 
			const mavros_msgs::GlobalPositionTarget::ConstPtr& pub, float longitude, float latitude, float altitude, float threshold){
	mavros_msgs::GlobalPositionTarget target;
	target.latitude = longitude;
	target.longitude = latitude;
	target.altitude = altitude;

	while(abs(global_pose.longitude - longitude) > threshold && 
		  abs(global_pose.latitude - latitude) > threshold && 
		  abs(global_pose.altitude - altitude) > threshold){
		if(current_state.mode == "OFFBOARD"){
			pub.publish(target);
			ros::spinOnce();
			rate.sleep();
		}
	}
}

void pick_up_marker(const ros::NodeHandle::ConstPtr& nh){
	//TODO
}

void takeoff(const ros::NodeHandle::ConstPtr& nh){
	//TODO
}




/* Callback Functions: */
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
	found = *msg;
	return;
}
// For recording the locations reported by the image processing node
void marker_position_cb(std_msgs::Float64MultiArray::ConstPtr& msg){
	std_msgs::Float64MultiArray temp = *msg;
	if(found){
		if((abs(temp.x(FIXTHIS) - IP_location_x) <= REPEATED_VAL_THRESHOLD) &&
			(abs(temp.y(FIXTHIS) - IP_location_y) <= REPEATED_VAL_THRESHOLD)){
				count++;
			}
		else{		
			IP_location_x = temp.x(FIXTHIS);
			IP_location_y = temp.y(FIXTHIS);
		}
	}
	return;
}
// For monitoring the drone's state/mode
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	return;
}






