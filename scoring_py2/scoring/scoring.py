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
from nav_msgs.msg import Odometry

from math import log, sqrt

from cls.data_logger import Logger
score_logger = Logger()
#TODO: change logging location
filepath = "/home/rosuser/catkin_ws/src/multi_robot_challenge_22/src/scoring"

t_remaining = 600
ar_human_score = 200
# ar_human_pos_score = 100
ar_fire_score = 100
# ar_fire_pos_score = 50
ar_big_fire_score = 150
robots_meet_score = 400

n_of_ar_human = 0 # found number of AR tags representing human
n_of_ar_fire = 0 # found number of AR tags representing fire
n_of_ar_big_fire = 0 #
n_of_ar_human_pos = 0 # correct human AR tag positions
n_of_ar_fire_pos = 0 # correct fire AR tag positions
n_of_ar_big_fire_pos = 0
robots_meet = False # Flag to check if the robots meet at the big fire

support_sent = False #Flag to check if support has been sent

# small_fire_id = [0, 1]
# big_fire_id = [0, 1, 3, 4]
# human_id = [2]
fire_id = [0, 1, 3, 4]
# big_fire_id = [4]
human_id = [2]
gazebo_models_list = ["ground_plane", 'unit_box', 'unit_sphere', 'Marker0', 'Marker1', 'Marker2', 'Marker3', 'Marker4']
models_pos_dict = {}
found_tags = set([])


robot_0_pos = -1
robot_1_pos = -1
robot_2_pos = -1

def clbk_odom_0(msg):
    global robot_0_pos
    robot_0_pos = msg.pose.pose.position

def clbk_odom_1(msg):
    global robot_1_pos
    robot_1_pos = msg.pose.pose.position

def clbk_odom_2(msg):
    global robot_2_pos
    robot_2_pos = msg.pose.pose.position


def get_gazebo_models():
    # global models_pos_dict
    ## TEST PURPOSE ONLY
    # rospy.set_param('ar_fire', {'id': 0, 'position': {'x': 1, 'y': 2, 'z': 3}})
    # rospy.set_param('ar_human', {'id': 2, 'position': {'x': 1, 'y': 2, 'z': 3}})

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
def distance_error(p1, p2, _TH=1.41):
    d = sqrt((p1.x - p2['x'])**2 + (p1.y - p2['y'])**2)

    pass_flag = False
    if (d < _TH):
        pass_flag = True
    else:
        pass_flag = False
    return pass_flag, d

def distance_error_robot(p1, p2, _TH=1.41):
    # TODO: make it more general case if necessary

    d = sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

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

def check_robot_pos():
    global models_pos_dict, robot_0_pos, robot_1_pos, robot_2_pos
    global support_sent
    pass_flag = False
    # print robot_0_pos
    pass_flag_0, d_0 = distance_error_robot(robot_0_pos, models_pos_dict['Marker4'], 3)
    pass_flag_1, d_1 = distance_error_robot(robot_1_pos, models_pos_dict['Marker4'], 3)
    pass_flag_2, d_2 = distance_error_robot(robot_2_pos, models_pos_dict['Marker4'], 3)

    if not support_sent:
        if pass_flag_0:
            rospy.set_param(robot_0_pos)
            support_sent = True
        elif pass_flag_1:
            rospy.set_param(robot_1_pos)
            support_sent = True


    print "Check Robot Pos"
    print d_0
    print d_1
    print d_2
    if pass_flag_0 and pass_flag_1 or pass_flag_2 and pass_flag_1 or pass_flag_2 and pass_flag_0:
        pass_flag = True

    return pass_flag


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
    global start_time, n_of_ar_human, n_of_ar_fire, n_of_ar_human_pos, n_of_ar_fire_pos, n_of_ar_big_fire, n_of_ar_big_fire_pos, found_tags, robots_meet
    start_time = time.localtime()
    check_big_fire_robots = False
    first_meet = True
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if check_big_fire_robots and first_meet:


            robot_pass_flag = check_robot_pos()
            robots_meet = robot_pass_flag

            if robots_meet:
                first_meet = False
                score_logger.save("The robots met at the big fire")


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
                            n_of_ar_human_pos += 1
                            found_tags.add(ar_human['id'])
                            score_logger.save(["Found human AR pos:", ar_human['id'] , "n_of_ar_human_pos: ", n_of_ar_human_pos])
                        human_id.remove(ar_human['id'])
            if fire_param_found:

                if ar_fire['id'] in fire_id:
                    resp_model_name = 'Marker'+unicode(ar_fire['id'])
                    resp_pos = ar_fire['position']
                    pass_flag, d = check_pos(resp_model_name, resp_pos)
                    if ar_fire['id'] == 4:
                        robot_pass_flag = check_robot_pos()
                        check_big_fire_robots = True
                        robots_meet = robot_pass_flag
                        if robots_meet:
                            first_meet = False
                            score_logger.save("The robots met at the big fire")

                    if not pass_flag:
                        if not ar_fire['id'] in found_tags:
                            if ar_fire['id'] == 4: n_of_ar_big_fire += 1
                            else: n_of_ar_fire += 1
                            print "n_of_ar_fire", n_of_ar_fire
                            found_tags.add(ar_fire['id'])
                            score_logger.save(["Found fire AR id: ", ar_fire['id'] , "n_of_ar_fire: ", n_of_ar_fire])
                        print "Fire AR id is wrong position"
                        rospy.set_param('ar_fire_fail', {'id': ar_fire['id'], 'distance': d})
                    else:
                        if not ar_fire['id'] in found_tags:
                            if ar_fire['id'] == 4: n_of_ar_big_fire += 1
                            else: n_of_ar_fire += 1
                            if ar_fire['id'] == 4: n_of_ar_big_fire_pos += 1
                            else: n_of_ar_fire_pos += 1
                            found_tags.add(ar_fire['id'])
                            score_logger.save(["Found fire AR id: ", ar_fire['id'], " n_of_ar_fire_pos: ", n_of_ar_fire_pos])

                        fire_id.remove(ar_fire['id'])
        except Exception, e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print exc_type, fname, exc_tb.tb_lineno
            print e


        update_score()
        rate.sleep()

## Result
def update_score():
    global robots_meet
    finished_flag = False
    time, score = total_score_calc()
    print time,score, ((n_of_ar_fire)+(n_of_ar_human))

    if (score <= 0) or (((n_of_ar_fire+n_of_ar_human+n_of_ar_big_fire) == 5) and robots_meet):
        finished_flag = True

    if finished_flag:
        print "Game is ended. Your score = "
        print score
        rospy.set_param('score', score)
        score_logger.save(["Final score: ", score, " in: ", time, " seconds"])
        score_logger.save(["Total human tags: ", n_of_ar_human, " Total fire tags: ", n_of_ar_fire])
        score_logger.save(["Total human pos: ", n_of_ar_human_pos, " Total fire pos: ", n_of_ar_fire_pos])
        sys.exit()


def total_score_calc():
    global robots_meet
    curr_time = time.localtime()
    elapsed_time = ((((curr_time.tm_hour)*60)+((curr_time.tm_min)*60)+(curr_time.tm_sec))-(((start_time.tm_hour)*60)+((start_time.tm_min)*60)+(start_time.tm_sec)))
    score = t_remaining - elapsed_time + n_of_ar_fire_pos*ar_fire_score + n_of_ar_human_pos*ar_human_score + n_of_ar_big_fire_pos*ar_big_fire_score
    if robots_meet:
        score += robots_meet_score
    return elapsed_time, score



if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Please specify the group number")

    group_nr = sys.argv[1]
    # group_nr = 1
    rospy.init_node("scoring")

    rospy.Subscriber("/tb3_0/odom", Odometry, clbk_odom_0)
    rospy.Subscriber("/tb3_1/odom", Odometry, clbk_odom_1)
    rospy.Subscriber("/tb3_2/odom", Odometry, clbk_odom_2)




    get_gazebo_models()
    start_flag = False
    rospy.set_param('start_flag', False) ## Only for test purposes it is True for now. Change it to False soon
    while not start_flag:
        try:
            start_flag = rospy.get_param('start_flag')
            print "Please set start_flag parameter"
        except KeyboardInterrupt:
            print "Keyboard interrupt. Exiting"

        rospy.sleep(1)

    rospy.set_param("finish_flag", False)
    score_logger.log_file_creator(filepath, group_nr)
    competition(score_logger)
