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
//TODO: Add more hardcoded values

/* All of the helper functions are listed here, but are written out after the main function. */

/* All of the callback functions are listed here, but are written after the helper functions. */

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
	/* Setup all subscribers. */
	//TODO: Figure out which ones are needed and subscribe to them
	
    /* Calculate the desired search route. */
	//TODO: Write the function and put it here
	
	/* Wait for the RC controller to give up control. */
	//TODO: Write code for this
	
	/* Take off! */
	//TODO: Make a function for this probably
	
	/* Follow the route until the IP side says they found the marker. */
	
	/* Follow IP commands until there are no more updates. */
	
	
	/* Pick up the marker. */
	
	
	/* Fly to the drop off location. */
	
	
	/* Execute drop off manuever. */
	
	/* Fly to landing location. */
	
	/* Land the drone. */
	
	
}
