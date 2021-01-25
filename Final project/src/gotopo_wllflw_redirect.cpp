#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <ros/console.h>

/*********************************************//**
* This code provides a server for the services,
* driving the 'go_to_point' and 'wall_follower' 
* behaviour, which were remapped into 4 different
* services in the launch file, in order to 
* distinguish requests made by the 'bug_m' (here
* referred to with te index 'BUG') or 
* 'robot_mainframe' (here 'NHC', for 'non 
* holonomic control'. The service callbacks here 
* provided simply forward the requests to  the 
* 'actual servers', listening to 
* 'wall_follower_switch' and 'go_to_point_switch',
* if certain conditions are met.
*
************************************************/

ros::ServiceClient client_go_to_point;  /**< Service client enabling go to point directly */
ros::ServiceClient client_wall_follow;  /**< Service client enabling wall follow directly */
std_srvs::SetBool wllflw, gotopo;       /**< Variables where used for services calls */

std::string plan_algo_used;             /**< Will contain the planning algorithm currently used */
bool wall_follow_active;                /**< Global value denoting if the robot is currently
                                             in wall following mode, in order to allow 
                                             (or prevent) 'bug_m' node to control 
                                             go_to_point_switch and wall_follower_switch
                                         */


/*********************************************//**
* Callback to a call on service
* '/go_to_point_switch_nhc'
*
* This function simply forwards the request to
* the desired service (maintened for symmetry)
*
* \param req (std_srvs::SetBool::Request &):	
* 			request field of the service, 
*           will be forwarded;
* \param res (std_srvs::SetBool::Response &):	
* 			response field of the service,
*           filled with the results of the
*           desired call;
*
* \retval success (bool):
* 			same value as res.success;
*
************************************************/
bool goToPoNHC(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    ROS_DEBUG("NHC REQUESTED GO_TO_POINT");
    gotopo.request.data = req.data;
    client_go_to_point.call(gotopo);
    res.success = gotopo.response.success;

    return true;
}

/*********************************************//**
* Callback to a call on service
* '/go_to_point_switch_bug'
*
* This function decides if the request received
* should be forwarded to the service 
* '/go_to_point_switch' (for which it was intended
* by the calling node). The decision is made on
* the base of the current value of the parameter
* 'active_plan_algorithm', in this case the call
* is forwarded if the algorithm is 'bug0' and the
* robot is not in wall following mode
* (the call has been made from the bug_m node).
*
* \param req (std_srvs::SetBool::Request &):	
* 			request field of the service, 
*           will be forwarded if the 
*           conditions are met;
* \param res (std_srvs::SetBool::Response &):	
* 			response field of the service,
*           filled with the results of the
*           desired call if the conditions
*           are met, 'false' otherwise;
*
* \retval success (bool):
* 			same value as res.success;
*
************************************************/
bool goToPoBUG(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    if(!ros::param::get("active_plan_algorithm", plan_algo_used))
        { ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    if(!ros::param::get("/wall_follow_active", wall_follow_active))
        { ROS_ERROR("No parameter named 'wall_follow_active' found."); }
    
    ROS_DEBUG("BUG0 TRIED TO REQUEST GO_TO_POINT");
    
    if (plan_algo_used == "bug0" && !wall_follow_active)
    {
        ROS_DEBUG("BUG0 REQUESTED GO_TO_POINT");
        gotopo.request.data = req.data;
        client_go_to_point.call(gotopo);
        res.success = gotopo.response.success;
    }
    
    return true;
}

/*********************************************//**
* Callback to a call on service
* '/wall_follower_switch_nhc'
*
* This function simply forwards the request to
* the desired service (maintened for symmetry)
*
* \param req (std_srvs::SetBool::Request &):	
* 			request field of the service, 
*           will be forwarded;
* \param res (std_srvs::SetBool::Response &):	
* 			response field of the service,
*           filled with the results of the
*           desired call;
*
* \retval success (bool):
* 			same value as res.success;
*
************************************************/
bool wllFlwNHC(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    ROS_DEBUG("NHC REQUESTED WALL_FOLLOWER");
    wllflw.request.data = req.data;
    client_wall_follow.call(wllflw);
    res.success = wllflw.response.success;
    return true;
}

/*********************************************//**
* Callback to a call on service
* '/wall_follower_switch_bug'
*
* This function decides if the request received
* should be forwarded to the service 
* '/wall_follower_switch' (for which it was 
* intended by the calling node). The decision is
* made on the base of the current value of the 
* parameter 'active_plan_algorithm', in this case 
* the call is forwarded if the algorithm is 'bug0'
* and the robot is not in wall following mode
* (the call has been made from the bug_m node).
*
* \param req (std_srvs::SetBool::Request &):	
* 			request field of the service, 
*           will be forwarded if the 
*           conditions are met;
* \param res (std_srvs::SetBool::Response &):	
* 			response field of the service,
*           filled with the results of the
*           desired call if the conditions
*           are met, 'false' otherwise;
*
* \retval success (bool):
* 			same value as res.success;
*
************************************************/
bool wllFlwBUG(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    if(!ros::param::get("active_plan_algorithm", plan_algo_used))
        { ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
    if(!ros::param::get("/wall_follow_active", wall_follow_active))
        { ROS_ERROR("No parameter named 'wall_follow_active' found."); }
    
    ROS_DEBUG("BUG0 TRIED TO REQUEST WALL_FOLLOWER");

    if (plan_algo_used == "bug0" && !wall_follow_active)
    {
        ROS_DEBUG("BUG0 REQUESTED WALL_FOLLOWER");
        wllflw.request.data = req.data;
        client_wall_follow.call(wllflw);
        res.success = wllflw.response.success;
    }
    
    return true;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "gotopo_wllflw_redirect");
	ros::NodeHandle n;
	
	/* // Uncomment these lines to allow debugging messages to be printed on stdout
	    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();
    */
    
    ros::ServiceServer srv_gotopo_nhc   = n.advertiseService("/go_to_point_switch_nhc", goToPoNHC);
    ros::ServiceServer srv_gotopo_bug   = n.advertiseService("/go_to_point_switch_bug", goToPoBUG);
    ros::ServiceServer srv_wllflw_nhc   = n.advertiseService("/wall_follower_switch_nhc", wllFlwNHC);
    ros::ServiceServer srv_wllflw_bug   = n.advertiseService("/wall_follower_switch_bug", wllFlwBUG);

  	client_wall_follow                  = n.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
  	client_go_to_point                  = n.serviceClient<std_srvs::SetBool>("/go_to_point_switch");
  	
  	ros::spin();

    return 0;
}
