#!/usr/bin/env python

'''
DAT160 competition scoring.
'''

from __future__ import absolute_import
from re import I
import rospy
import sys, os
import time

from rospy.core import rospywarn
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

from math import log, sqrt

from cls.data_logger import Logger
score_logger = Logger()
#TODO: change logging location
filepath = "/home/gizem/catkin_ws/src/beginner_tutorials/src/output"

t_remaining = 600
ar_human_score = 200
ar_human_pos_score = 75
ar_fire_score = 100
ar_fire_pos_score = 50
robots_meet_score = 2000

n_of_ar_human = 0 # found number of AR tags representing human
n_of_ar_fire = 0 # found number of AR tags representing fire
n_of_ar_human_pos = 0 # correct human AR tag positions
n_of_ar_fire_pos = 0 # correct fire AR tag positions
robots_meet = False # Flag to check if the robots meet at the big fire

# small_fire_id = [0, 1]
# big_fire_id = [0, 1, 3, 4]
# human_id = [2]
fire_id = [0, 1, 3, 4]
# big_fire_id = [1]
human_id = [2]
gazebo_models_list = ["ground_plane", 'unit_box', 'unit_sphere', 'Marker0', 'Marker1', 'Marker2', 'Marker3', 'Marker4']
models_pos_dict = {}
found_tags = set([])


def get_gazebo_models():
    ## TEST PURPOSE ONLY
    # rospy.set_param('ar_fire', {'id': 0, 'position': {'x': 1, 'y': 2, 'z': 3}})
    rospy.set_param('ar_human', {'id': 2, 'position': {'x': 1, 'y': 2, 'z': 3}})

    found = False
    data = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5)
    while not found:
        if data:
            for model_name in gazebo_models_list:
                if model_name in data.name:
                    index = data.name.index(model_name)
                    pos = data.pose[index].position
                    new = {model_name:pos}
                    models_pos_dict.update(new)
            found = True
        else:
            rospy.loginfo("Topic /gazebo/model_states NOT Ready yet, trying again ")
            found = False


## COMPETITION
def distance_error(p1, p2):
    # TODO: make it more general case if necessary
    _TH = 1.41
    d1 = sqrt(p1.x**2 * p1.y**2 + p1.z**2)
    d2 = sqrt(p2['x']**2 * p2['y']**2 + p2['z']**2)

    d = abs(d2-d1)

    pass_flag = False
    if (d < _TH):
        pass_flag = True
    else:
        pass_flag = False
    return pass_flag, d


def check_pos(model_name, pos):
    global models_pos_dict
    pass_flag, d = distance_error(models_pos_dict[model_name], pos)
    return pass_flag, d


#TODO not the best check up. refactor the whole function later if necessary
def competition(group_nr):
    '''
    Get parameters ar_fire and ar_human (if available)
    Check if in the respective list
    Check if it is found before
    Check if position correct

    Set *_fail parameter if AR found but position wrong

    Update score
    '''
    global start_time, n_of_ar_human, n_of_ar_fire, n_of_ar_human_pos, n_of_ar_fire_pos, found_tags
    start_time = time.localtime()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            ar_fire = rospy.get_param('ar_fire')
            fire_param_found = True
            print "New fire found"
            print ar_fire['id']
            rospy.delete_param('ar_fire')
        except KeyError, e:
            fire_param_found = False
        try:
            ar_human = rospy.get_param('ar_human')
            human_param_found = True
            print "New human found"
            rospy.delete_param('ar_human')
        except KeyError, e:
            human_param_found = False
        try:
            if human_param_found:
                if ar_human['id'] in human_id:
                    resp_model_name = 'Marker'+unicode(ar_human['id'])
                    resp_pos = ar_human['position']
                    pass_flag, d = check_pos(resp_model_name, resp_pos)
                    if not pass_flag:
                        print "n_of_ar_human", n_of_ar_human
                        if not ar_human['id'] in found_tags:
                            n_of_ar_human += 1
                            found_tags.add(ar_human['id'])
                            score_logger.save(["Found human AR id: ", ar_human['id'] , "n_of_ar_human: ", n_of_ar_human])
                        print "Human AR wrong position"
                        rospy.set_param('ar_human_fail', {'id': ar_human['id'], 'distance': d})
                    else:
                        if not ar_human['id'] in found_tags:
                            n_of_ar_human += 1
                            found_tags.add(ar_human['id'])
                            score_logger.save(["Found human AR id:", ar_human['id'] , n_of_ar_human])
                        n_of_ar_human_pos += 1
                        score_logger.save(["Found human AR pos:", ar_human['id'] , "n_of_ar_human_pos: ", n_of_ar_human_pos])
                        human_id.remove(ar_human['id'])
            if fire_param_found:
                if ar_fire['id'] in fire_id:
                    resp_model_name = 'Marker'+unicode(ar_fire['id'])
                    resp_pos = ar_fire['position']
                    pass_flag, d = check_pos(resp_model_name, resp_pos)
                    if not pass_flag:
                        if not ar_fire['id'] in found_tags:
                            n_of_ar_fire += 1
                            print "n_of_ar_fire", n_of_ar_fire
                            found_tags.add(ar_fire['id'])
                            score_logger.save(["Found fire AR id: ", ar_fire['id'] , "n_of_ar_fire: ", n_of_ar_fire])
                        print "Fire AR id is wrong position"
                        rospy.set_param('ar_fire_fail', {'id': ar_fire['id'], 'distance': d})
                    else:
                        if not ar_fire['id'] in found_tags:
                            n_of_ar_fire += 1
                            found_tags.add(ar_fire['id'])
                            score_logger.save(["Found fire AR id: ", ar_fire['id'] , "n_of_ar_fire: ", n_of_ar_fire])
                        n_of_ar_fire_pos += 1
                        fire_id.remove(ar_fire['id'])
        except Exception, e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print exc_type, fname, exc_tb.tb_lineno    
        
        update_score()
        rate.sleep()

## Result
def update_score():
    time, score = total_score_calc()
    print time,score, ((n_of_ar_fire)+(n_of_ar_human))

    if ((score <= 0) or ((n_of_ar_fire)+(n_of_ar_human)) == 5): ## (or time <= 0)
        rospy.set_param('finish_flag', True)

    if rospy.get_param('finish_flag'):
        print "Game is ended. Your score = "
        print score
        rospy.set_param('score', score)
        score_logger.save(["Final score: ", score, " in: ", time, " seconds"])
        score_logger.save(["Total human tags: ", n_of_ar_human, " Total fire tags: ", n_of_ar_fire])
        score_logger.save(["Total human pos: ", n_of_ar_human_pos, " Total fire pos: ", n_of_ar_fire_pos])
        sys.exit()


def total_score_calc():
    curr_time = time.localtime()
    elapsed_time = ((((curr_time.tm_min)*60)+(curr_time.tm_sec))-(((start_time.tm_min)*60)+(start_time.tm_sec)))
    score = t_remaining - elapsed_time + n_of_ar_fire*ar_fire_score + n_of_ar_human*ar_human_score
    return elapsed_time, score



if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Please specify the group number")

    group_nr = sys.argv[1]
    # group_nr = 1
    rospy.init_node("scoring")
    get_gazebo_models()
    start_flag = False
    rospy.set_param('start_flag', True) ## Only for test purposes it is True for now. Change it to False soon
    while not start_flag:
        try:
            start_flag = rospy.get_param('start_flag')
            print "Please set start_flag parameter"
            rospy.sleep(1)
        except KeyboardInterrupt:
            print "Keyboard interrupt. Exiting"
    
    rospy.set_param("finish_flag", False)
    score_logger.log_file_creator(filepath, group_nr)
    competition(score_logger)
