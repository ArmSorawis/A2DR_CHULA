#!/usr/bin/env python

import os

def readGoal_floor5():
    goal_path = "/home/a2dr-robot/a2dr_ws/src/a2dr_navigation/text/"
    goal_filename = "goal_list_f5.txt"
    path = goal_path + goal_filename
    
    fil_goalList = []
    num_goal = 10

    with open(path) as f:
        goal_list = f.read().splitlines()
    
    for index in range(len(goal_list)):
        if index < num_goal:
            string = goal_list[index]
            goal_list[index] = string.split(", ")
            fil_goalList.append(goal_list[index])
        if index >= num_goal:
            pass
    for index in range(len(fil_goalList)):
        for index_sub in range(len(fil_goalList[index])):
            fil_goalList[index][index_sub] = float(fil_goalList[index][index_sub])

    return fil_goalList

goal = readGoal_floor5()
print(goal)