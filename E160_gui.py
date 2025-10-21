import json
import random
import time
from E160_environment import *
from E160_graphics import *
import tkinter as tk


def main():  
    
    # set time step size in seconds
    deltaT = 0.1

    # instantiate robot navigation classes
    environment = E160_environment(deltaT)
    graphics = E160_graphics(environment)

    root = graphics.tk
    debug_label = tk.Label(root, text="waiting...")
    debug_label.pack()
    
    
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break
        
        # update robots
        environment.update_robots(deltaT)

        ctrl = environment.robots[0].controller
        c = ctrl.get_robot_state()
        d = ctrl.robot.state_des.get_des_state() 

        c_str = (
            f"X={c[0]:.2f}, Y={c[1]:.2f}, θ={c[2]:.2f}, "
            f"vix={c[3]:.2f}, viy={c[4]:.2f}, wi={c[5]:.2f}, "
            f"v={c[6]:.2f}, w={c[7]:.2f}"
        )

        d_str = f"X={d[0]:.2f}, Y={d[1]:.2f}, θ={d[2]:.2f}"
        
        debug_label.config(text=f"c: {c_str}\nd: {d_str}")


        # log all the robot data
        environment.log_data()
    
        # maintain timing
        time.sleep(deltaT)
            
main()
