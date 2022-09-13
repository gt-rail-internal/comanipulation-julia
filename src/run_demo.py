#!/usr/bin/env python

# For simple data rostopic echo -b file.bag -p /topic > data.csv
# http://wiki.ros.org/rosbag/Tutorials/Exporting%20image%20and%20video%20data

from codecs import raw_unicode_escape_decode
from ros_dispatch import parse_traj_file, follow_trajectory, start_ros_node, end_ros_node 
import random
import Tkinter as tk
import datetime
import rospy
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Bool
import json
from pygame import mixer
import time
import sys
import os
from learning_tf2.msg import Num, MAstamp

mixer.init()

def play_music():
    time.sleep(3)
    mixer.music.load("beep.mp3")
    mixer.music.play()

def get_problem():
    operators = [' + ', ' - ']
    first_num = random.randint(100000, 999999)
    second_num = random.randint(100000, 999999)
    third_num = random.randint(100000, 999999)
    return str(first_num) + random.choice(operators) + str(second_num) + random.choice(operators) + str(third_num)

def run_experiment(user_id, path, pub_start, pub_stop):
    path = "/home/hritiksapra/Documents/comoto.jl/src"
    map_type_to_traj = {
        "WHITE_1_A_C_52":"traj_1_A_C_52.txt",
        "PINK_2_A_B":"traj_2_A_B.txt",
        "PINK_2_D_E":"traj_2_D_E.txt",
        "PINK_2_E_B":"traj_2_E_B.txt",
        "PINK_2_E_C":"traj_2_E_C.txt",
        "BLACK_3_A_C":"traj_3_A_C_52.txt",
        "BLACK_3_C_B":"traj_3_C_B.txt",
        "BLACK_3_D_B":"traj_3_D_B.txt",
    }

    start_traj = ["PINK_2_E_C", "WHITE_1_A_C_52"] # [, 
    end_traj = ["BLACK_3_A_C","PINK_2_D_E"] # , 
    middle_traj = ["BLACK_3_D_B", "BLACK_3_C_B", "PINK_2_A_B", "PINK_2_E_B"] # 

    methods =  ["COMOTO", "NOMINAL", "DIST"]

    log_data = []

    random.shuffle(methods)
    # random.shuffle(start_traj)
    # random.shuffle(end_traj)
    # random.shuffle(middle_traj)

    start_ros_node()

    window = tk.Tk()
    width= window.winfo_screenwidth() 
    height= window.winfo_screenheight()
    window.geometry("%dx%d" % (width, height))
    window.configure(bg='#8f918e')
    frame = tk.Frame(window, bg="#8f918e", width = width, height = height)
    my_string_var = tk.StringVar() 
    my_string_var.set("Please listen to the instructions before starting the experiment")
    text_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=my_string_var, font=("Arial", 40))
    second_string_var = tk.StringVar()
    second_string_var.set("")
    calculate_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=second_string_var, font=("Arial", 40))
    third_string_var = tk.StringVar()
    third_string_var.set("")
    clear_label = tk.Label(frame, bg="#8f918e", fg = "black", textvariable=third_string_var, font=("Arial", 40))
    frame.pack()
    text_label.place(x = 300, y = 500)
    calculate_label.place(x = 500, y = 600)
    clear_label.place(x = 455, y = 300)

    print("Demo")

    counter = 0

    types = [start_traj[1], end_traj[0], middle_traj[2]]

    # log_data.append("Part 1")
    robot_type = 1

    for method in methods:

        print(method)

        random.shuffle(types)

        for i in range(len(types)):
            traj_file_name = map_type_to_traj[types[i]]

            if method == "NOMINAL":
                traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]

            if method == "DIST":
                traj_file_name = traj_file_name[:len(traj_file_name) - 4] + "_dist.txt"

            # log_data.append(traj_file_name)

            type_name = types[i]

            traj_file_name = "/home/hritiksapra/Documents/comoto.jl/src/"+traj_file_name

            traj, dt, t_0 = parse_traj_file(traj_file_name)

            raw_input("Display Start Instructions?")

            frame.configure(bg='#8f918e')
            text_label.configure(bg = "#8f918e", fg = "black")
            calculate_label.configure(bg = "#8f918e", fg = "black")
            clear_label.configure(bg = "#8f918e", fg = "black")
            
            my_string_var.set("         The arm is now moving to the start position")
            second_string_var.set("")
            # second_string_var.set("                 This is Robot {}".format(robot_type))
            third_string_var.set("")


            raw_input("Go to start?")

            # follow_trajectory([traj[0]], 4.0,  4.0)

            raw_input("Display move instructions?")

            if type_name.startswith("WHITE"):
                frame.configure(bg='white')
                my_string_var.set("Reach for the WHITE calculator (Number 1) after the BEEP")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "white")
                calculate_label.configure(bg = "white")
                clear_label.configure(bg = "white")
            
            if type_name.startswith("PINK"):
                frame.configure(bg='#e75480')
                my_string_var.set("Reach for the PINK calculator (Number 2) after the BEEP")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "#e75480", fg = "white")
                calculate_label.configure(bg = "#e75480", fg = "white")
                clear_label.configure(bg = "#e75480", fg = "white")
            
            if type_name.startswith("BLACK"):
                frame.configure(bg='black')
                my_string_var.set("Reach for the BLACK calculator (Number 3) after the BEEP")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "black", fg = "white")
                calculate_label.configure(bg = "black", fg = "white")
                clear_label.configure(bg = "black", fg = "white")
            
        
            calculator_problem = get_problem()
            # log_data.append(calculator_problem)
            second_string_var.set("Calculate: " + calculator_problem)

            raw_input("Start countdown?")
            # pub_start.publish(Bool(True))

            play_music()

            time.sleep(0.5)

            begin = time.time()

            # follow_trajectory(traj, dt, 1.0)

            raw_input("Calculated Problem?")
            # pub_stop.publish(Bool(True))
            # pub_start.publish(Bool(False))

            end = time.time()

            # log_data.append(str(end - begin))

            print(end - begin)

            answer = ""

            while len(answer) == 0:
                answer = raw_input("Answer: ")
                answer.strip()

            # log_data.append(answer)
        robot_type += 1
        break #-------------------------------------
        
        my_string_var.set("Please step back from the table and fill out the survey")
        second_string_var.set("")
        third_string_var.set("")
        
        frame.configure(bg='#8f918e')
        text_label.configure(bg = "#8f918e", fg = "black")
        calculate_label.configure(bg = "#8f918e", fg = "black")
        clear_label.configure(bg = "#8f918e", fg = "black")

        raw_input("----Stop for survey!----")
    

    next_types = [start_traj[1]]

    random.shuffle(next_types)

    print("First part done")

    # log_data.append("Part 2")

    for type_name in next_types:

        print(type_name)
        random.shuffle(methods)
        robot_type = 0

        for method in methods:
            
            robot_type += 1

            traj_file_name = map_type_to_traj[type_name]

            if method == "NOMINAL":
                traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]
            
            if method == "DIST":
                traj_file_name = traj_file_name[:len(traj_file_name) - 4] + "_dist.txt"

            # log_data.append(traj_file_name)

            traj_file_name = "/home/hritiksapra/Documents/comoto.jl/src/"+traj_file_name

            traj, dt, t_0 = parse_traj_file(traj_file_name)

            print(method)

            raw_input("Display Start Instructions?")

            my_string_var.set("         The arm is now moving to the start position")
            second_string_var.set("")
            third_string_var.set("")
            
            frame.configure(bg='#8f918e')
            text_label.configure(bg = "#8f918e", fg = "black")
            calculate_label.configure(bg = "#8f918e", fg = "black")
            clear_label.configure(bg = "#8f918e", fg = "black")

            raw_input("Go to start?")

            my_string_var.set("         The arm is now moving to the start position")
            second_string_var.set("")
            # second_string_var.set("                 This is Robot {}".format(robot_type))
            third_string_var.set("")

            follow_trajectory([traj[0]], 4.0,  4.0)

            raw_input("Display move instructions?")

            if type_name.startswith("WHITE"):
                frame.configure(bg='white')
                my_string_var.set("                                                        ")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "white")
                calculate_label.configure(bg = "white")
                clear_label.configure(bg = "white")
            
            if type_name.startswith("PINK"):
                frame.configure(bg='#e75480')
                my_string_var.set("                                                        ")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "#e75480", fg = "white")
                calculate_label.configure(bg = "#e75480", fg = "white")
                clear_label.configure(bg = "#e75480", fg = "white")
            
            if type_name.startswith("BLACK"):
                frame.configure(bg='black')
                my_string_var.set("                                                        ")
                third_string_var.set("Make sure the calculator is on and cleared")
                text_label.configure(bg = "black", fg = "white")
                calculate_label.configure(bg = "black", fg = "white")
                clear_label.configure(bg = "black", fg = "white")
            
            calculator_problem = get_problem()

            second_string_var.set("")
            
            raw_input("Start countdown?")
            # pub_start.publish(Bool(True))

            play_music()

            time.sleep(1)

            begin = time.time()

            follow_trajectory(traj, dt, 1.0)

            raw_input("Calculated Problem?")
            # pub_stop.publish(Bool(True))
            # pub_start.publish(Bool(False))

            end = time.time()

            answer = ""

            while len(answer) == 0:
                answer = raw_input("Answer: ")
                answer.strip()

        my_string_var.set("Please step back from the table and fill out the survey")
        second_string_var.set("")
        third_string_var.set("")

        frame.configure(bg='#8f918e')
        text_label.configure(bg = "#8f918e", fg = "black")
        calculate_label.configure(bg = "#8f918e", fg = "black")
        clear_label.configure(bg = "#8f918e", fg = "black")

        raw_input("----Stop for survey!----")

    raw_input("End experiment?")
    textfile = open(path+"/experiment_run.txt", "w")
    end_ros_node()

def main(user_id, pub_start, pub_stop):
    logdir_path = '/home/hritiksapra/Documents/comoto.jl/src/expt_logs/user_{}'.format(user_id)
    if not os.path.exists(logdir_path):
        os.mkdir(logdir_path)
        print('Created directory for User {}'.format(user_id))

    run_experiment(user_id, logdir_path, pub_start, pub_stop)

if __name__ == '__main__':
    rospy.init_node('flag_topic', anonymous=False)
    pub_start = rospy.Publisher('/flag_topic/start', Num, queue_size=10)
    pub_stop = rospy.Publisher('/flag_topic/stop', Num, queue_size=10)
    main(sys.argv[1], pub_start, pub_stop)