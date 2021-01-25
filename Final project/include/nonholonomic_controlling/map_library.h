#ifndef MAP_LIBRARY_H
#define MAP_LIBRARY_H

#include "ros/ros.h"

#define MOVE_BASE_DRIVE 0
#define BUG0_DRIVE 1
#define WALL_FOLLOW_DRIVE 2
#define STOP_DRIVE 3
#define WALL_FOLLOW_DRIVE 2
#define CHOICE_RAND_POS 1
#define CHOICE_USER_POS 2
#define CHOICE_WALL_FOLLOW 3
#define CHOICE_STOP 4
#define CHOICE_CHANGE_ALGORITHM 5
#define STATE_WAITING 0
#define STATE_DRIVING 1

/*********************************************//**
* Function returning the element in a map next to
* a key passed, treating the map as circular.
*
* \param map (std::map<std::string, int>):
*           map to iterate on;
* \param current_key (std::string):
*           key to evaluate the next element of;
*
* \retval next_map_element 
*      (std::map<std::string, int>::iterator):
*           iterator containing the pair 
*           <key, value> of the successive 
*           element to the key passed;
*
************************************************/
std::map<std::string, int>::iterator nextMapElement(std::map<std::string, int> map, std::string current_key);

#endif
