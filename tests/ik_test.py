from os import wait3
import gym
from simple_xarm.envs.xarm_ik_env import XarmIK
import pybullet as p
import numpy as np
import time
# from simple_xarm.envs.xarm_env import XarmEnv

env = XarmIK()
observation = env.reset()


for _ in range(5000):
  obj_pos = list(env.getObjectPose(env.object_id)[0])
  print(f"obj_pos: {obj_pos}")
  pre_pick_offset = [0,0,.5]
  pre_pick_pos = [obj_pos[0]+pre_pick_offset[0],obj_pos[1]+pre_pick_offset[1],obj_pos[2]+pre_pick_offset[2]]
  # pick_pos = [obj_pos[0],obj_pos[1],obj_pos[2]-0.07]
  pick_pos = [obj_pos[0],obj_pos[1],obj_pos[2]-0.20]
  env.setPose(pre_pick_pos,env.initial_eef_q4,grip_width=0,wait_finish=True) #open

  # print("open")Engin
  # env.setPose(pick_pos,env.initial_eef_q4,grip_width=0,wait_finish=True) #open
  # print("close")
  # env.setPose(pick_pos,env.initial_eef_q4,grip_width=1,wait_finish=True) #close
  # print("half")
  # env.setPose(pick_pos,env.initial_eef_q4,grip_width=0.5,wait_finish=True) #half
  
  print("2")
  env.setPose(pick_pos,env.initial_eef_q4,grip_width=0.,wait_finish=True)
  print("3")
  print("Get joint",env.getJointStates())
  env.setPose(pick_pos,env.initial_eef_q4,grip_width=0.9,wait_finish=True)
  print("4")
  env.setPose(pre_pick_pos,env.initial_eef_q4,grip_width=0.85,wait_finish=True)
  env.step()
  
env.close()