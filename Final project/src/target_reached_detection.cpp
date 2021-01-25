#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "actionlib_msgs/GoalStatusArray.h" // Service for reading current status of move_base planning algorithm
#include "move_base_msgs/MoveBaseActionGoal.h" // 
#include <mutex>
#include <condition_variable>
#include <ros/callback_queue.h>
#include <thread>

/*********************************************//**
* This code detects whenever a target results 
* reached by any of the algorithms used. To do 
* that it looks at the status of the 'move_base'
* planning (if present) and at the service
* requeste by the 'bug_m' (originally for its
* UI, now redirected).
* This implements a new node handle, with a 
* callback queue dedicated to deal with services 
* requests made from bug_m at the instant it
* reaches a target. While, in the original 
* algorithm, it waited until a new user input
*  by directly calling the user interface, here
* that solution is not feasible, since the UI
* is called by the "mainframe" algorithm. 
* But what this node can know is when a new goal
* is established, since it gets published on 
*'move_base/goal'. The callback to the redirected
* user_interface service from bug_m is thus 
* stopped (using mutexes and conditional 
* variables) until a new move_base/goal message
* arrives (which sets the global variable 
* target_reacheed to 'false'). However, if we
* were to use the same callback queue for all
* incoming messages and requests, this wait on
* the condvar would stop the spinner from serving
* other messages (thus preventing the variable
* to ever change, deadlocking the system).
* This is the reason why a new spinner thread
* is created to uniquely serve the 
* '/redirect_bug_user_interface' service 
* requests, so that it can be blocked without
* interfering with all other callbacks.
*
************************************************/

std::condition_variable cv_plan;        /**< Conditional variable regulating the reply to the service called from bug_m*/
std::mutex mux_plan;                    /**< Mutex protecting the boolean the conditional variable is tested on */
bool is_bug_plan = false;               /**< Boolean expressing the condition, 'true' when the plan is "bug0", 'false' else */

std_msgs::Empty empty_msg;
bool target_reached = true;
ros::Publisher pub; // NOTE: the choice of using messages instead of services is due to the fact that, altough sporadic events,
                    // we want the system to know as soon as possible (and, more important, without loosing events) when a 
                    // target is reached.
bool wall_follow_active;                /**< Global value denoting if the robot is currently
                                             in wall following mode, in order to allow 
                                             (or prevent) 'bug_m' node to control 
                                             go_to_point_switch and wall_follower_switch
                                         */

/*********************************************//**
* Callback to a call on service 
* '/redirect_bug_user_interface'
*
* The script 'bug_m' calls the service 
* 'user_interface', remapped to this service, when
* the target position is reached in order to 
* receive a new one. Originally another node would
* ask that input to the user, but here it's
* intercepted in order to detect when a target 
* results reached when using the bug0 planner.
* Note that the service is empty, the important
* thing is just that a call is received, no
* exchange of data. 
* Notice the usage of a con
*
* \param req (std_srvs::Empty::Request &):	
* 			request field of the service 
            (empty, unused);
* \param res (std_srvs::Empty::Response &):	
* 			response field of the service,
*           (empty, unused);
*
* \retval success (bool):
* 			'true' by default (unused);
*
************************************************/
bool bugReachedCllbck(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    std::string plan_algo_used;
    if(!ros::param::get("active_plan_algorithm", plan_algo_used))
        { ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }
        
    if(!ros::param::get("/wall_follow_active", wall_follow_active))
        { ROS_ERROR("No parameter named 'wall_follow_active' found."); }
    
    ROS_DEBUG("BUG0 CALLED FOR TARGET REACHED");
    if (!target_reached && !wall_follow_active)
    {
        target_reached = true; 
        ROS_DEBUG("Bug0 called for target reached.\n");
        pub.publish(empty_msg);  
    }
    {
        std::unique_lock<std::mutex> lock(mux_plan);
        cv_plan.wait(lock, []{return !target_reached;}); 
    }
    ROS_DEBUG("BUG0 CAN UPDATE DES_POS_X_Y");
    // the conditional variable will unlock the mutex only once the new goal
    // has been set, in other terms when target_reached is 'false', which
    // happens when the callback for move_base/goal is launched
    // if the current planning algorithm is not bug0 ignore this call
    return true;
}

/*********************************************//**
* Callback to a message on topic 
* '/move_base/status'
*
* The message contains various information, among
* which is the current status of the robot with 
* respect to the target position.
* The status ID = 3 corresponds to 
* "target reached". If such status is reached
* when the current planning algorithm is 
* "move_base" a message on topic 
* '/target_reached' is published. Notice that, due
* to the periodic nature of this callback, a flag
* is here set, making this node react only to the
* first message stating a target reached received.
*
* \param status_msg 
*   (const actionlib_msgs::GoalStatusArray::ConstPtr&):	
* 			status message, the actual status ID
*           is contained in field 'status' inside
*           the last element of the 'status_list'
*           field of the message,
*           
*
************************************************/
void moveStatusCllbck(const actionlib_msgs::GoalStatusArray::ConstPtr& status_msg){
    std::string plan_algo_used;
    if(!ros::param::get("active_plan_algorithm", plan_algo_used))
        { ROS_ERROR("No parameter named 'active_plan_algorithm' found."); }

    if (!target_reached && plan_algo_used == "move_base" 
        && !status_msg->status_list.empty() 
        && status_msg->status_list.back().status == 3){
        
        target_reached = true;    // blocks sensitivity to status msgs
        ROS_DEBUG("MOVE_BASE called for target reached.\n");
        pub.publish(empty_msg); // status_list is a vector, the last element is accessed
   }
    // if the current planning algorithm is not move_base ignore its call
}

/*********************************************//**
* Callback to a message on topic 
* '/move_base/goal'
*
* Once a new goal is issued the node is made
* susceptible again to messages or service calls
* interpretable as 'target reached'.
*
* \param move_msg 
*   (const move_base_msgs::MoveBaseActionGoal::ConstPtr&):	
* 			goal message (unused);
*
************************************************/
void moveGoalCllbck(const move_base_msgs::MoveBaseActionGoal::ConstPtr& move_msg){
    ROS_DEBUG("TRD RECEIVED MOVEGOAL CALLBACK\n");
    {
        std::lock_guard<std::mutex> lock(mux_plan);
      	target_reached = false;
      	cv_plan.notify_all();
    }
    // this resets the sensitivity to the status msgs
    // and notifies the mutex waiting on the conditional
    // variable blocking the return of the bug service
    // callback
}

/*********************************************//**
* Callback to a message on topic 
* '/target_reached'
*
* Once a message from that topic is received the
* node is made unsusceptible to a new message
* informing of the target being reached (until a 
* new goal is issued, see 'moveGoalCllbck' 
* function.
*
* \param empty (const std_msgs::Empty::ConstPtr&):	
* 			(empty, unused);
*
************************************************/
void targetReachedCllbck(const std_msgs::Empty::ConstPtr& empty){
    ROS_DEBUG("TRD RECEIVED TARGETREACHED CALLBACK\n");
    target_reached = true;
    
}

int main(int argc, char** argv){
	ros::init(argc, argv, "target_reached_server");
	ros::NodeHandle n;
	ros::NodeHandle bug_n;
	ros::CallbackQueue bug_queue;
	ros::AsyncSpinner bug_spinner(1, &bug_queue);
	
	/* // Uncomment these lines to allow for debug messages to be printed on stdout
	    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();
    */
    
    pub = n.advertise<std_msgs::Empty>("/target_reached", 1000);
    ros::Subscriber status_sub  =   n.subscribe("/move_base/status", 1000, moveStatusCllbck);
    ros::Subscriber goal_sub    =   n.subscribe("/move_base/goal", 1000, moveGoalCllbck);
    ros::Subscriber reach_sub   =   n.subscribe("/target_reached", 1000, targetReachedCllbck);
     
	bug_n.setCallbackQueue(&bug_queue);
    ros::ServiceServer service  = bug_n.advertiseService("/redirect_bug_user_interface", bugReachedCllbck);
    std::thread spinner_bug_thread([&bug_queue]()   {
                                                        ros::SingleThreadedSpinner spinner_bug;
                                                        spinner_bug.spin(&bug_queue);
                                                    });
    
    ros::spin();
    spinner_bug_thread.join();
    return 0;
}
