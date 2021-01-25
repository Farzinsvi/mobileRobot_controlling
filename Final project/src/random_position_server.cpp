#include "ros/ros.h"
#include <cstdlib>	// rand()
#include <ctime>	// rand() seed
#include <vector>
#include <utility>

#include "nonholonomic_controlling/TargetPos.h"	// Service for target position

/*********************************************//**
* This function is a simple ServiceServer 
* returning a random position among those 
* expressed in the parameter server, as repose
* to a request received.
*
************************************************/

std::vector< std::pair<double, double> > target_pos_;
int prev_choice = -1;
int target_pos_dim_ = 0;
bool avoid_duplicates = false;


/*********************************************//**
* Callback to a call on service 
* '/target_position/rand'
*
* This function chooses one of the available 
* targets expressed in the parameter server
* with pseudo-randomic behaviour. If the command
* line argument 'avoid_duplicates' is 'true' it
* will avoid to ouput the same random position
* it previously chose (although if the last
* target location was user-defined it could
* be replicated nonetheless).
*
* \param req (nonholonomic_controlling::TargetPos::Request &):	
* 			request field of the service 
            (empty, unused);
* \param res (nonholovomic_controlling::TargetPos::Response &):	
* 			response field of the service,
*           a geometry_msgs::Point;
*
* \retval success (bool):
* 			'true' by default (unused);
*
************************************************/
bool rand_pos(nonholonomic_controlling::TargetPos::Request &req, nonholo_control::TargetPos::Response &res){
    int i;
    do{
        while( (i = rand()) > RAND_MAX - (RAND_MAX - (target_pos_dim_-1) )%target_pos_dim_ ){}
        // discard values of rand whih would unbalance the ranndomization
        i = i%target_pos_dim_;
    }while(avoid_duplicates && prev_choice==i); // easy but time wasting way: more complex
                                                // implementation could remove previous goal from the vector, 
                                                // saving it elsewhere, and reinserting it when the next goal
                                                // is defined.
    prev_choice = i;                            // Update the last choice made
    res.target_pos.x = target_pos_[i].first;
	res.target_pos.y = target_pos_[i].second;
	return true;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "random_position_server");
	ros::NodeHandle n;
	std::vector<double> param_vector;
	
	if(argc>1 && strcmp(argv[1],"true")==0){    // avoid choosing as the next random position one already visited
	    avoid_duplicates = true;
	}
	
	srand(time(0));	/* random seed initialization */
	ros::ServiceServer service = n.advertiseService("/target_position/rand", rand_pos);
										
	if(!ros::param::get("goals_vector", param_vector))
	    { ROS_ERROR("No field 'goals_vector' found in the parameter file."); }
	    
	for(int i = 0; i<param_vector.size(); i+=2){
	    target_pos_.push_back(std::make_pair(param_vector[i], param_vector[i+1]));
	}
	    
	target_pos_dim_ = target_pos_.size();   // number of valid target positions
	ros::spin();

	return 0;
}
