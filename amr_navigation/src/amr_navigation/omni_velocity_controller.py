#!/usr/bin/env python

PACKAGE = 'amr_navigation'
import rospy

from math import atan2, copysign,cos,sin
from velocity_controller import VelocityController, Velocity
from velocity_controller import get_shortest_angle, get_distance

class OmniVelocityController(VelocityController):
    """
    def __init__(self, *args):
            raise NotImplementedError('This is your assignment to implement OmniVelocityController')
            pass
    """
    
    """
    ========================= YOUR CODE HERE =========================

    Instructions: put here all the functions that are necessary to
    implement the VelocityController interface. You may
    use the DiffVelocityController as an example.

    Implement the constructor to accept all the necessry parameters
    and implement compute_velocity() method

    You are free to write any helper functions or classes you might
    need.

    ==================================================================

    """    
    
    
    def __init__(self, l_max_vel, l_tolerance, a_max_vel, a_tolerance, 
                     max_linear_acceleration, max_angular_acceleration):
            self._l_max_vel = l_max_vel
            self._l_tolerance = l_tolerance
            self._a_max_vel = a_max_vel
            self._a_tolerance = a_tolerance
            self._max_l_acc = max_linear_acceleration
            self._max_a_acc = max_angular_acceleration
    
    def compute_velocity(self, actual_pose):
            
           # Displacement and orientation to the target in world frame:
            dx = self._target_pose.x - actual_pose.x
            dy = self._target_pose.y - actual_pose.y
            pose_theta=get_shortest_angle(atan2(dy, dx),actual_pose.theta)

            # Step 1: compute remaining distances
            linear_dist = get_distance(self._target_pose, actual_pose)
            angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)
            
            if (    abs(linear_dist)<self._l_tolerance and
                    abs(angular_dist)<self._a_tolerance):
                self._linear_complete = True
                self._angular_complete = True
                return Velocity()
                         
            # Step 2: compute velocities
            linear_vel_x, linear_vel_y, angular_vel = 0, 0, 0
          #  w_turn = ((pose_theta-angular_dist)/angular_dist)
            
            l_time=linear_dist/self._l_max_vel
            a_time=angular_dist/self._a_max_vel      
            
       
            
            
            if abs(linear_dist)>self._l_tolerance:
                scaled_l_vel=self._l_max_vel
                linear_vel_x = (scaled_l_vel*cos(pose_theta) if abs(linear_dist)>5*self._l_tolerance else
                               self._l_tolerance*cos(pose_theta))
                linear_vel_y = (scaled_l_vel*sin(pose_theta) if abs(linear_dist)>5*self._l_tolerance else
                               self._l_tolerance*sin(pose_theta))
            if abs(angular_dist)>self._a_tolerance:
                scaled_a_vel=self._a_max_vel*(a_time/l_time)
                      
                angular_vel=(scaled_a_vel if abs(angular_dist)>5*self._a_tolerance else
                               self._a_tolerance*(a_time/l_time))
               
            rospy.loginfo(actual_pose)
            return Velocity(linear_vel_x,
                            linear_vel_y,
                            angular_vel)
