#!/usr/bin/env python3
#coding=utf-8
import time
from Transbot_Lib import Transbot
from ipywidgets import interact
import ipywidgets as widgets

# 创建Transbot对象 bot
# Create Transbot object as bot
bot = Transbot()

# 启动接收数据 
# Start receiving data
bot.create_receive_threading()

from time import sleep
from datetime import datetime

# 控制机械臂关节运动
# Control the joint movement of the manipulator
def arm_servo7(s_angle):
    if s_angle > 210 or s_angle < 160:
        return
    bot.set_uart_servo_angle(7, s_angle)
    
# 控制机械臂关节运动
# Control the joint movement of the manipulator
def arm_servo8(s_angle):
    if s_angle > 180 or s_angle < 30:
        return
    bot.set_uart_servo_angle(8, s_angle)
    
# 控制机械臂关节运动
# Control the joint movement of the manipulator
def arm_servo9(s_angle):
    if s_angle > 180 or s_angle < 30:
        return
    bot.set_uart_servo_angle(9, s_angle)

def init_pose():
    arm_servo9(30)
    sleep(1)
    arm_servo7(210)
    sleep(1)
    arm_servo8(30)
    sleep(1)
init_pose()

def read_gripper_angle():
    max_attempts = 20
    for i in range(max_attempts):
        value = bot.get_uart_servo_angle(9)
        if value >= 0:
            return value
    exit(0)
    

print(bot.get_uart_servo_angle(9))

def close_gripper():
    #ret_changes = []
    target_angle = 30
    step_size = 5.0
    tolerance = 14.0
    comparable = lambda x,y: abs(x-y) <= tolerance
    last_angles = []
    last_target_angles = []
    while(target_angle <= 180-step_size):
        current_angle = read_gripper_angle()
        print("current angle and target angle %d %d" % (current_angle, target_angle))
        last_angles.append(current_angle)
        last_target_angles.append(target_angle)
        if current_angle >= 80 and len(last_angles) >= 2 and last_angles[-2] != last_angles[-1] and last_angles[-2] + 3 >= last_angles[-1]:
            arm_servo9(last_target_angles[-2])
            print("FEEDBACK STOP 1")
            sleep(0.5)
            break
        if comparable(target_angle, current_angle):
            target_angle += step_size
            arm_servo9(target_angle)
            sleep(0.5)
        else:
            break
    #return ret_changes
    return []
    
def open_gripper():
    arm_servo9(30)
    sleep(1)

def pick():
    open_gripper()
    sleep(1)
    arm_servo7(160)
    sleep(1)
    close_gripper()
    sleep(1)
    arm_servo7(210)
    sleep(1)
    
def drop():
    arm_servo7(160)
    sleep(1)
    open_gripper()
    sleep(1)
    arm_servo7(210)
    sleep(1)
    

#pick()
#drop()

# 程序结束后请删除transbot对象，避免在其他程序中使用transbot库造成冲突
# After the program is complete, delete the Transbot object to avoid conflicts caused by using the Transbot library in other programs
#del bot
