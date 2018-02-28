__author__ = 'Jacqueline Rutschke'

import nengo
import numpy as np
import motionPrimitives
import time

leg_0_AD_joints = [['/robot_leg0_alpha_joint_pos_cntr/command'],['/robot_leg0_delta_joint_pos_cntr/command']]
leg_1_AD_joints = [['/robot_leg1_alpha_joint_pos_cntr/command'],['/robot_leg1_delta_joint_pos_cntr/command']]
leg_2_AD_joints = [['/robot_leg2_alpha_joint_pos_cntr/command'],['/robot_leg2_delta_joint_pos_cntr/command']]
leg_3_AD_joints = [['/robot_leg3_alpha_joint_pos_cntr/command'],['/robot_leg3_delta_joint_pos_cntr/command']]
leg_4_AD_joints = [['/robot_leg4_alpha_joint_pos_cntr/command'],['/robot_leg4_delta_joint_pos_cntr/command']]
leg_5_AD_joints = [['/robot_leg5_alpha_joint_pos_cntr/command'],['/robot_leg5_delta_joint_pos_cntr/command']]

leg_0_BG_joints = [['/robot_leg0_beta_joint_pos_cntr/command'],['/robot_leg0_gamma_joint_pos_cntr/command']]
leg_1_BG_joints = [['/robot_leg1_beta_joint_pos_cntr/command'],['/robot_leg1_gamma_joint_pos_cntr/command']]
leg_2_BG_joints = [['/robot_leg2_beta_joint_pos_cntr/command'],['/robot_leg2_gamma_joint_pos_cntr/command']]
leg_3_BG_joints = [['/robot_leg3_beta_joint_pos_cntr/command'],['/robot_leg3_gamma_joint_pos_cntr/command']]
leg_4_BG_joints = [['/robot_leg4_beta_joint_pos_cntr/command'],['/robot_leg4_gamma_joint_pos_cntr/command']]
leg_5_BG_joints = [['/robot_leg5_beta_joint_pos_cntr/command'],['/robot_leg5_gamma_joint_pos_cntr/command']]

# For lifting
leg0_3_4_BG = [leg_0_BG_joints[0], leg_0_BG_joints[1],
leg_3_BG_joints[0], leg_3_BG_joints[1],
leg_4_BG_joints[0], leg_4_BG_joints[1]]

leg1_2_5_BG = [leg_1_BG_joints[0], leg_1_BG_joints[1],
leg_2_BG_joints[0], leg_2_BG_joints[1],
leg_5_BG_joints[0], leg_5_BG_joints[1]]

#For Swinging
leg0_1_4_5_AD = [leg_0_AD_joints[0], leg_0_AD_joints[1],
leg_1_AD_joints[0], leg_1_AD_joints[1],
leg_4_AD_joints[0], leg_4_AD_joints[1],
leg_5_AD_joints[0], leg_5_AD_joints[1]]

leg2_3_AD = [leg_2_AD_joints[0], leg_2_AD_joints[1],
leg_3_AD_joints[0], leg_3_AD_joints[1]]
   
def lift125Cos(t):
    if t%8>=4:
        return 0.5+(np.cos(t * np.pi/4))/2
    return 1

def lift034Cos(t):
    if t%8<=4:
        return 0.5+(np.cos(t * np.pi/4))/2
    return 1
    
def swingFourCos(t):
    return 0.5+(np.cos(t * np.pi/4))/2
    
def swingTwoCos(t):
    return 1-swingFourCos(t)

liftLEFT = motionPrimitives.MotionPrimitives(1, lift034Cos, leg0_3_4_BG)
liftRIGHT = motionPrimitives.MotionPrimitives(1, lift125Cos, leg1_2_5_BG)
swingFOUR = motionPrimitives.MotionPrimitives(0, swingFourCos, leg0_1_4_5_AD)
swingTWO = motionPrimitives.MotionPrimitives(0, swingTwoCos, leg2_3_AD)

model = nengo.Network()
with model:
   liftLEFT.get_network()
   liftRIGHT.get_network()
   swingFOUR.get_network()
   swingTWO.get_network()
