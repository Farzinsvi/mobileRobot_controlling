#! /usr/bin/env python

import rospy
import roslaunch
import time
from std_srvs.srv import *

######                                  #####
# This simple node waits for a call on its  #
# service 'bug_restart_service', which then #
# instructs it to launch a new instance of  #
# the 'bug' node, using 'roslaunch' API     #
# in the 'main'.                            #
#####                                   #####
package = 'final_assignment'
executable = 'bug_m.py'
_to_restart = False

node = roslaunch.core.Node(package, executable, name='bug', \
                    remap_args= [('user_interface','redirect_bug_user_interface'), \
                                ('wall_follower_switch','wall_follower_switch_bug'), \
                                ('go_to_point_switch','go_to_point_switch_bug')], \
                                output='screen')


def bugRestartCllbck(req):
    """
    A function that sets the global 
    variable '_to_restar', later on
    used by the main to launch a new
    instance of the 'bug' node.

    """
    global _to_restart
    _to_restart = True
    print 'service called'
    #this uses the API of the roslaunch package to restart the
    # specified package
    return []

def main():
    global _to_restart
    rospy.init_node('bug_restart_service')
    
    srv = rospy.Service('bug_restart', Empty, bugRestartCllbck)
    
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    
    rospy.loginfo("bug_restart_service launched")
    rate = rospy.Rate(20)
    
    while not rospy.is_shutdown():
        if _to_restart:
            print 'starting the process'
            process = launch.launch(node)
            print process.is_alive()
            _to_restart = False
            
        rate.sleep()

if __name__ == '__main__':
    main()
