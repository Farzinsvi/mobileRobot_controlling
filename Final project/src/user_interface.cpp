#include "ros/ros.h"
#include "nonholonomic_controlling/UIMenu.h"
#include "nonholonomic_controlling/map_library.h"    // contains the definition of a function operating on the map
#include "std_srvs/Empty.h"


/*********************************************//**
* This function is a simple ServiceServer 
* displaying to the User the list of possible
* actions the robot an perform. After reading the
* User input it sends it back in the response
* field, without performing computations.
*
************************************************/

std::string plan_algo_used; /**< Name of the planning algorithm used */
std::map<std::string, int> plan_algo_map;   /**< Map of the available planning alogorithms "name":ID, retrieved from parameter server */
bool alive = true;  /**< Variable used to inform this node that it should return when the main node of the project shuts down */


/*********************************************//**
* Callback to a service call of type 
* nonholo_control::UIMenu on service '/ui_menu'
* 
* This function displays the menu with options
* the user can choose from. The input choice 
* is inserted in the 'response' field of the
* service, while the services generating the
* target position are called (if request by the
* input), and their point inserte in the response
* field as well.
*
* \param req (nonholo_control::UIMenu::Request &):	
* 			request field of the service;
* \param res (nonholo_control::UIMenu::Response &):	
* 			response field of the service;
*
* \retval success (bool):
* 			'true' by default, unused;
************************************************/
bool menuCllbck(nonholo_control::UIMenu::Request &req, nonholo_control::UIMenu::Response &res){
    int ans=10;
    char str[10], curr_alg[16], next_alg[16];
    
	if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    while(ans > 5 && ans > 0){
        ROS_INFO("\n1.\tRandom goal\n2.\tUser defined goal\n3.\tWall follow\n4.\tStop\n5.\tChange algorithm (from %s to %s)\n", 
                                                            plan_algo_used.c_str(), 
                                                            nextMapElement(plan_algo_map, plan_algo_used)->first.c_str()     );
                                                            // the string printed contain the current and next planning algorithm
                                                            // names, as present in the 'plan_algo_map' (note that the orderd in the 
                                                            // map is alphabetical on the algorithm names, keys in the map)
        scanf("%s", str);
        sscanf(str, "%d", &ans);
    }
   
	// empty request, service returns the new target position
	
    res.menu_case = ans; 
    return true;
}



/*********************************************//**
* Callback to a service call of type 
* std_srv::Empty, informing the User Interface 
* that the 'robot_mainframe' has been closed
* by an user request, and thus this node should
* be shut down as well (and with it all the nodes
* in its launch file, being it declared as
* 'required').
*
* \param req (std_srv::Empty::Request &):	
* 			(unused, empty);
* \param res (std_srv::Empty::Response &):	
* 			(unused, empty);
*
************************************************/
bool sdCllbck(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    alive = false;
    return true;
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "nonholonomic_controlling");
  	ros::NodeHandle n;
  	
    ros::ServiceServer srv_shutdown = n.advertiseService("/UI_shutdown", sdCllbck);
    ros::ServiceServer srv_ui = n.advertiseService("/ui_menu", menuCllbck);

	if(!ros::param::get("plan_algorithm", plan_algo_map))
	    { ROS_ERROR("No parameter named 'plan_algorithm' found."); }
	    
	ros::Rate loop(5);  // doesn't need to be particularly reactive.
	while(ros::ok() && alive){
        ros::spinOnce();
	}
	
	return 0;
}
