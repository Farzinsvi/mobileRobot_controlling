#include "ros/ros.h"
#include "std_srvs/SetBool.h"   // to reuse the wall_follow script
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "topic_tools/MuxSelect.h"
#include "nav_msgs/Odometry.h"		// Message containing estimated position
#include "move_base_msgs/MoveBaseActionGoal.h" // Service for setting a goal that move_base can understand

#include "nonholonomic_controlling/map_library.h"
#include "nonholonomic_controlling/TargetPos.h"	// Service for target position
#include "nonholonomic_controlling/UIMenu.h"	// Service for target position

#include <sstream>
#include <iostream>
#include <math.h>

/*********************************************//**
* This is the main node in the package, the one 
* which runs and coordinates (almost) all the
* others, responsible for dictating which of the
* available planning algorithm drives the robot
* toward the received goal.
* Notice that this node actually does minimal
* computing, while delegating the various tasks
* to other nodes.
************************************************/

nonholonomic_controlling::TargetPos target_srv;  /**< Service entityy where target position will be inserted into by one of the appropriate services */
ros::ServiceClient client_target_rand;	/**< Service client used for obtaining random target positions */
ros::ServiceClient client_target_user;	/**< Service client used for obtaining user defined target positions */
ros::Publisher pub;					    /**< Publisher used to publish velocities */

ros::ServiceClient client_go_to_point;  /**< Service client enabling go to point directly */
ros::ServiceClient client_wall_follow;  /**< Service client enabling wall follow directly */
ros::ServiceClient client_ui_menu;      /**< Service client used to communicate with the UI */

ros::ServiceClient mux_cmd_vel;         /**< Allowing for multiplexing between multiple robot driving sources*/
ros::ServiceClient client_bug_restart;  /**< Used to call a rospy script using 'roslaunch' API to restart 'bug' node */
ros::ServiceClient client_UI_shutdown;  /**< Used to inform the 'user_interface' node of this node shutting down */

geometry_msgs::Point target_position;	/**< Target position to reach, defined as a Point */
geometry_msgs::Point previous_target_position;	/**< Target position to reach, defined as a Point */
geometry_msgs::Point current_position;	/**< Current position of the robot, defined as a Point */

int drive_algo = MOVE_BASE_DRIVE; // 0 -> move_base, 1 -> bug0
char state_ = STATE_WAITING;
bool _target_reached = false;
bool _target_unreachable = false;

std::map<std::string, int> plan_algo_map;   /**< Map of the available planning alogorithms "name":ID, 
                                                retrieved from parameter server */
std::map<std::string, int>::iterator plan_algo_map_it;  /**< Iterator moving on the planning algorithms map */
std::string plan_algo_used;                 /**< Name of the planning algorithm used */

/*  STATUSES:
    0: waiting for target (includes following a wall)
    1: moving, non interruptible
*/
/*********************************************//**
* Routine to publish a target position (goal), 
* stored as a global attribute
* 
* This function displays the menu with options
* the user can choose from. The input choice 
* is inserted in the 'response' field of the
* service.
* Note the different way the two algorithms 
* require to indicate the goal position, with 
* move_base reading it from a topic 
* (move_base_action_goal) and bug0 reading it 
* from the parameter server.
*
************************************************/
void publish_target(){
        move_base_msgs::MoveBaseActionGoal move_msg;
        
        ros::param::set("/des_pos_x", target_position.x);
        ros::param::set("/des_pos_y", target_position.y);
        
        // fill it with the local data and publish it
        move_msg.goal.target_pose.header.frame_id = "map";
        move_msg.goal.target_pose.pose.position.x = target_position.x;
        move_msg.goal.target_pose.pose.position.y = target_position.y;
        move_msg.goal.target_pose.pose.orientation.w = 1;
        
        pub.publish(move_msg);
}

/*********************************************//**
* Function that updates the reference to the 
* planning algorithm used.
*
* The update is both local and on the value
* stored in the parameter server.
*
* \param new_plan (std::string):
*           name of the algorithm, as presented
*           in the keys of the map in the
*           parameters;
*
************************************************/
void set_plan_algo(std::string new_plan){

    drive_algo = plan_algo_map[new_plan];
    plan_algo_used = new_plan;
    ros::param::set("active_plan_algorithm", new_plan);
}

/*********************************************//**
* Function that updates the behaviour of the
* planning algorithm used.
*
* It first updates the reference to the algorithm,
* then uses a multiplexer to select which of the
* remapped cmd_vel outputs to use, either the
* move_base or bug0 one. It also calls the 
* swithcer for wall_follow and go_to_point 
* behaviour to set that behaviour (notice how
* they do not impact the driving mode move_base
* as a planner to a goal). In the function the
* value of the parameter 'wall_follow_active'
* is set as well, denoting if the wall follow
* behaviour is in act, in order to "mute" the
* bug0 service calls to go_to_point and 
* wall_follow in the meantime.
*
* \param new_drive (int):
*           the ID of the planning parameter to
*           use:\n
*           0 -> move_base\n
*           1 -> bug0\n
*           2 -> wall follow\n
*           3 -> stop (unused)\n
*
************************************************/
void change_drive(int new_drive){
    topic_tools::MuxSelect drive;
    std_srvs::SetBool wall_follow, go_to_point;
    bool wall_follow_active = false;
        
    if (new_drive == MOVE_BASE_DRIVE){
        set_plan_algo("move_base");
        // ---                              ---
	    wall_follow.request.data = false;
	    go_to_point.request.data = false;
        // ---                              ---
        drive.request.topic = "cmd_vel_move";
    }
    else if (new_drive == BUG0_DRIVE){
        set_plan_algo("bug0");
        // ---                              ---
	    wall_follow.request.data = false;
	    go_to_point.request.data = true;
        // ---                              ---
        drive.request.topic = "cmd_vel_bug";
    }
    else if (new_drive == WALL_FOLLOW_DRIVE){
        wall_follow_active = true;
        // ---                              ---
	    wall_follow.request.data = true;
	    go_to_point.request.data = false;
        // ---                              ---
        drive.request.topic = "cmd_vel_bug";
    }
    else if (new_drive == STOP_DRIVE){
        // ---                              ---
	    wall_follow.request.data = false;
	    go_to_point.request.data = false;
        // ---                              ---
        drive.request.topic = "cmd_vel_stop";
    }
              
    mux_cmd_vel.call(drive);
    
    client_wall_follow.call(wall_follow);
    client_go_to_point.call(go_to_point);
    
    if(!ros::param::has("/wall_follow_active"))
        { ROS_ERROR("No parameter named 'wall_follow_active' found."); }
    else
        ros::param::set("/wall_follow_active", wall_follow_active);
}

/*********************************************//**
* Callback executed every time a message on topic
* /odom is published, updating the current robot
* position.
*
* \param pose_msg (const nav_msgs::Odometry::ConstPtr&):	
* 			content of the message read;
*
************************************************/
void subscriberCallback(const nav_msgs::Odometry::ConstPtr& pose_msg){
    current_position = pose_msg->pose.pose.position;
}

/*********************************************//**
* Routine returning the distance between two 
* points.
*
* \param p1 (geometry_msgs::Point&):	
* 			first point;
* \param p2 (geometry_msgs::Point&):	
* 			second point;
* \retval pointDist (double):
*           the euclidean distance between p1 
*           and p2;
*
************************************************/
double pointDist(geometry_msgs::Point p1, geometry_msgs::Point p2){
	return sqrt(pow(p1.x-p2.x,2) + pow(p1.y-p2.y, 2));
}

/*********************************************//**
* Function computing the distance of the current
* robot position with respect to the target
* position.
*
* \retval errPos (double):
*           the euclidean distance between 
*           current and target position;
*           
************************************************/
double errPos(){
    return pointDist(target_position, current_position);
}

/*********************************************//**
* Callback executed every time a message on topic
* /target_reached is published. 
*
* The global variable _target_reached is set to 
* 'true'.
*
* \param empty (const std_msgs::Empty::ConstPtr&):	
* 			reference to empty message (unused);
*
************************************************/
void targetReachedCallback(const std_msgs::Empty::ConstPtr& empty){
    _target_reached = true; 
}

/*********************************************//**
* Callback executed every time a message on topic
* /target_unreachable is published. 
*
* The global variable _target_unreachable is set  
* to 'true'.
*
* \param empty (const std_msgs::Empty::ConstPtr&):	
* 			reference to empty message, unused;
*
************************************************/
void targetUnreachableCallback(const std_msgs::Empty::ConstPtr& empty){
    _target_unreachable = true; 
}


/*********************************************//**
* Routine performing a trivial form of recovery
* behaviour.
*
* The rovot is supposed to end up in this case 
* only when performing path planning using the
* 'bug_m' algorithm, since the 'move_base' 
* already has its own (quite more thought out)
* way of detecting and dealing with recovery
* behaviours.
* The robot is sent back to the previous target
* position, considered to be safe, using 
* 'move_base' planning (more robust than bug0).
* First the 'bug' node needs to be restarted, 
* in order for it to read the new target 
* position (more on that in the documentation).
*
* Notice that the routine has the side effect of
* modifying the current planning algorithm to
* 'move_base', even in the case it previously
* were 'bug0'.
*
************************************************/
void simpleRecovery(){
    std_srvs::Empty empty_srv;
    
    target_position = previous_target_position;
    publish_target();
    ROS_ERROR("The target position [%f %f] seems to be unreachable\nReturning to last visited target position", target_position.x, target_position.y);
    ROS_ERROR("The 'bug' node will be restarted");
    client_bug_restart.call(empty_srv);
    ros::Duration(7).sleep();   // wait to give time to the user to see the problem and 'bug' node to restart
    
   
    _target_reached = false;
	state_ = STATE_DRIVING;
    change_drive(MOVE_BASE_DRIVE);    // start driving towards last visited target position, would stop when it reaches it
    // Using move_base algorithm, way more robust than bug0, to get back, but without 
    // overwriting the current dribing algorithm chosen by the user.
}

int main(int argc, char **argv)
{
  	ros::init(argc, argv, "nonholonomic_controlling");
  	ros::NodeHandle n;
  	ros::Rate loop_rate(10);
    std_srvs::SetBool wall_follow, go_to_point;
    std_msgs::Empty empty;
    nonholo_control::UIMenu user_input;
    std_srvs::Empty empty_srv;
    
    
  	char in;
	
	client_ui_menu              =   n.serviceClient<nonholonomic_controlling::UIMenu>("/ui_menu");
  	client_wall_follow          =	n.serviceClient<std_srvs::SetBool>("/wall_follower_switch");
  	client_go_to_point          =	n.serviceClient<std_srvs::SetBool>("/go_to_point_switch");
  	
  	client_target_rand          =	n.serviceClient<nonholonomic_controlling::TargetPos>("/target_position/rand");
  	client_target_user          =	n.serviceClient<nonholonomic_controlling::TargetPos>("/target_position/user");
  	
    ros::Subscriber reached_sub =   n.subscribe("/target_reached", 1000, targetReachedCallback);
    ros::Subscriber unreach_sub =   n.subscribe("/target_unreachable", 1000, targetUnreachableCallback);
  	  	
  	mux_cmd_vel                 =   n.serviceClient<topic_tools::MuxSelect>("/mux_cmdvel/select");
  	client_bug_restart          =   n.serviceClient<std_srvs::Empty>("/bug_restart");
  	client_UI_shutdown           =	n.serviceClient<std_srvs::Empty>("/UI_shutdown");
  	
    pub = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1000);
	
	ros::Subscriber pose_sub = n.subscribe("/odom", 1000, subscriberCallback); 	/* subscriber to the topic relative to
																					robot odometry, setting the callback
																					computing velocity
																				 */
	if(!ros::param::get("plan_algorithm", plan_algo_map)){ ROS_ERROR("No parameter named 'plan_algorithm' found."); }
	if(!ros::param::get("active_plan_algorithm", plan_algo_used)){ ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
	drive_algo = plan_algo_map[plan_algo_used];
	
	 if(!ros::param::get("des_pos_x", target_position.x)){ ROS_ERROR("No parameter named 'des_pos_x' found."); } 
	 if(!ros::param::get("des_pos_y", target_position.y)){ ROS_ERROR("No parameter named 'des_pos_y' found."); } 
	 
	
    change_drive(drive_algo);
    ros::Rate(20);
    
	while(ros::ok()){
	    if (state_ == STATE_WAITING){
	        
	        client_ui_menu.call(user_input);
	        in = user_input.response.menu_case;
	        
	        if (in == CHOICE_RAND_POS || in == CHOICE_USER_POS){
    	        
	            if (in == CHOICE_RAND_POS){
                    client_target_rand.call(target_srv); 
                }  
                // empty request, service returns the new target position
	            else if(in == CHOICE_USER_POS){
                    client_target_user.call(target_srv); 
                }  
                
                _target_reached = false;    // we are trying to reach a new target
	            previous_target_position = target_position;
	            target_position.x = target_srv.response.target_pos.x;
	            target_position.y = target_srv.response.target_pos.y;
	            change_drive(drive_algo);
	            publish_target();
	            state_ = STATE_DRIVING;
	            
    	        ROS_INFO("Moving towards chosen goal [%f, %f]\n", 
    	                                                target_position.x, target_position.y);
	        }
	        else if (in == CHOICE_WALL_FOLLOW){
    	        ROS_INFO("Wall follow procedure started\n");
    	        change_drive(2);    
	        }
	        else if (in == CHOICE_STOP){
	            ROS_INFO("Standing still in [%f %f]\n", target_position.x, target_position.y);
	            client_UI_shutdown.call(empty_srv);
	            ros::shutdown();
	        }
	        else if (in == CHOICE_CHANGE_ALGORITHM){
    	        ROS_INFO("Changing movement algorithm.\n");
    	        plan_algo_map_it = nextMapElement(plan_algo_map, plan_algo_used); 
    	        // points to the next planning algorithm among those inserted in the param 
    	        // file map: note that they are ordered alphabetically with respect to the
    	        // algorithm names
    	        
    	        set_plan_algo(plan_algo_map_it->first);
	        }
	    }
	    else if (state_ == STATE_DRIVING){
	        if (_target_reached){
	            ROS_INFO("Goal reached [%f %f]\n", target_position.x, target_position.y);
	            state_ = STATE_WAITING;
	        }
	        else if(_target_unreachable){
	            simpleRecovery();
	            _target_unreachable = false;
	        }
	        else{
	            ROS_INFO("At [[%f %f]] going towards [%f %f]\nDistance [%f]\n", 
	                                    current_position.x, current_position.y, 
	                                    target_position.x, target_position.y, 
	                                    errPos());
	        }
	    }
	    
        ros::spinOnce();
        loop_rate.sleep();
	}
	
	return 0;
}
