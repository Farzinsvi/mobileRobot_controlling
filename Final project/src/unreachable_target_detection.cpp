#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "move_base_msgs/MoveBaseActionGoal.h" // 

/*********************************************//**
* This node is used to crudely identify a 
* situation where the goal coordinates are
* assumed to be unreachable by the robot.
* To do that, a timer is initialized each time a
* new goal is published, and the time passed is
* checked at each spin. If more than 2 minutes 
* have passed the goal is deemed unreachable, and
* a message on the topic '/unreachable_target' is
* published.
*
************************************************/

std_msgs::Empty empty_msg;
ros::Publisher pub;         /**< Publisher used to publish message when a target is reached */
ros::Time target_init;      /**< Timer used to track time passing between a goal request and it being reached*/
bool started = false;       /**< Variable that is 'false' whenever a goal has been reached but the next one hasn't been assigned yet */

/*********************************************//**
* Callback to a call on topic 'move_base/goal'
*
* Each time a new goal is published this 
* callback starts the 'target_init' timer,
* used to evaluate time passed since last goal
* wsa given.
*
* \param move_msg 
*   (const move_base_msgs::MoveBaseActionGoal::ConstPtr&):	
* 			message, its content is not useful;
*
************************************************/
void moveGoalCllbck(const move_base_msgs::MoveBaseActionGoal::ConstPtr& move_msg){
    started = true;
    target_init = ros::Time::now(); // every time a new goal is set the timer is reset
    ROS_DEBUG("UNREACH_DETECTION TIMER STARTED\n");
}


/*********************************************//**
* Callback to a call on topic '/target_reached'
*
* Each time a given tarrget is reached the timer
* will stop to be checked (but not reset, that's 
* done at the arrival  of a new goal).
*
* \param empty 
*   (const std_msgs::Empty::ConstPtr&):	
* 			(unused, empty);
*
************************************************/
void targetReachedCallback(const std_msgs::Empty::ConstPtr& empty){
    started = false; 
}


int main(int argc, char** argv){
	ros::init(argc, argv, "unreachable_goal_detection_server");
	ros::NodeHandle n;
	double sec_passed;
    
    ros::Publisher pub          =   n.advertise<std_msgs::Empty>("/target_unreachable", 1000);
  	ros::Subscriber reach_sub   =   n.subscribe("/target_reached", 1000, targetReachedCallback);
    ros::Subscriber pose_sub    =   n.subscribe("/move_base/goal", 1000, moveGoalCllbck);
    ros::Rate loop_rate(1);    // doesn't need to check with high frequency
    
    while (1){
        sec_passed = (ros::Time::now() - target_init).toSec();
        if ( started && sec_passed > 120){
            // more than 2 minutes passed from the instant the new target position was given, we can assume it got stuck
            // in the meantime
            started = false;
            ROS_ERROR("TIMEOUT DETECTED, TARGET SHOULD BE CONSIDERED UNREACHABLE\n");
            pub.publish(empty_msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
