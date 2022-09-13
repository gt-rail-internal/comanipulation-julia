from ros_dispatch import parse_traj_file, follow_trajectory, start_ros_node, end_ros_node 
import random
import Tkinter as tk
import datetime
import rospy
import tf
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import json
from pygame import mixer
import time
import sys
import os

mixer.init()

def play_music():
    time.sleep(3)
    mixer.music.load("beep.mp3")
    mixer.music.play()

def get_problem():
    operators = [' + ', ' - ']
    first_num = random.randint(100000, 999999)
    second_num = random.randint(100000, 999999)
    return str(first_num) + random.choice(operators) + str(second_num)

map_type_to_traj = {
    "BLACK_1_A_D_62":"traj_3_D_B.txt",
}

methods =   ["NOMINAL"]

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
frame.pack()
text_label.place(x = 300, y = 500)
calculate_label.place(x = 600, y = 600)

counter = 0

raw_input("Display Info")

frame.configure(bg='black')
my_string_var.set("Reach for the BLACK calculator (Number 3) after the beep")
text_label.configure(bg = "black", fg = "white")
calculate_label.configure(bg = "black", fg = "white")    

calculator_problem = get_problem()
second_string_var.set("Calculate: " + calculator_problem)

raw_input("Begin Experiment")

for method in methods:

    types = list(map_type_to_traj.keys())

    for i in range(len(types)):
        traj_file_name = map_type_to_traj[types[i]]

        if method == "NOMINAL":
            traj_file_name = traj_file_name[:5] + "nominal_" + traj_file_name[5:]

        type_name = types[i]

        traj, dt, t_0 = parse_traj_file(traj_file_name)

        time.sleep(2)        

        follow_trajectory([traj[0]], 4.0,  4.0)
        
        play_music()

        time.sleep(0.5)

        follow_trajectory(traj, dt, 1.0)
    
time.sleep(30)


end_ros_node()