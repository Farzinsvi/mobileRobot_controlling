#include "ros/ros.h"
#include <vector>
#include <utility>

#include "nonholonomic_controlling/TargetPos.h"	// Service for target position


/*********************************************//**
* This function is a simple ServiceServer 
* asking the User to insert a target position,
* which needs to compell to the imposed limits,
* restricting the choice to one of those defined
* in the parameter server.
*
************************************************/

std::vector< std::pair<double, double> > target_pos_;
int target_pos_dim_ = 0;

/*********************************************//**
* Function that searches for a 2D point 
* coordinates in the list of valid goals.
*
* \param x (double):
*           x coordinate;
* \param y (double):
*           y coordinate;
*
* \retval found (bool):
*           if the element has been found;
************************************************/
bool find_in_target_pos_(double x, double y){
    int found = false;
    for(int i=0; i<target_pos_dim_; i++){ if(found = (target_pos_[i].first==x)&&(target_pos_[i].second==y) ){ break; } }
    // found is given the val True only if [x,y] appears in the set, at which point it jumps out of the cycle
    
    return found;
}

/*********************************************//**
* Routine to print all valid positions retrieved
* from the parameter server.
*
************************************************/
void print_target_pos_(){
    for(int i=0; i<target_pos_dim_; i++){
        printf("[%lf , %lf]\t", target_pos_[i].first, target_pos_[i].second);
    }
    printf("\n");
}

/*********************************************//**
* Callback to a call on service 
* '/target_position/user'
*
* This function asks the user to insert a value
* for both X and Y coordinates of the goal, but
* allowing only valid targets (present in the 
* param file)
*
* \param req (nonholo_control::TargetPos::Request &):	
* 			request field of the service 
            (empty, unused);
* \param res (nonholo_control::TargetPos::Response &):	
* 			response field of the service,
*           a geometry_msgs::Point, contains the
*           valid position inserted;
*
* \retval success (bool):
* 			'true' by default (unused);
*
************************************************/
bool user_pos(nonholonomic_controlling::TargetPos::Request &req, nonholo_control::TargetPos::Response &res){
    double x=0.0, y=0.0;
	bool found = false;
	while(!found){  // the question is repeated as long until a valid position is inserted
    	ROS_INFO("Insert the desired target position, knowing it has to be one of these\n");
    	print_target_pos_();
    	printf("X:\t"); scanf("%lf", &x);
    	printf("Y:\t"); scanf("%lf", &y);
    	found = find_in_target_pos_(x, y);
	}
	ROS_INFO("Value accepted.\n");
	res.target_pos.x = x;
	res.target_pos.y = y;
	return true;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "random_position_server");
	ros::NodeHandle n;
	std::vector<double> param_vector;
	
	ros::ServiceServer service = n.advertiseService("/target_position/user", user_pos);
										
	if(!ros::param::get("goals_vector", param_vector))
    	{ ROS_ERROR("No field 'goals_vector' found in the parameter file."); }
    	
	for(int i = 0; i<param_vector.size(); i+=2){
	    target_pos_.push_back(std::make_pair(param_vector[i], param_vector[i+1]));
	}
	target_pos_dim_ = target_pos_.size();   // number of valid target positions
	ros::spin();

	return 0;
}
