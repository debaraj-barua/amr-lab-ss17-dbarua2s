#!/usr/bin/env python

PACKAGE = 'amr_navigation'
import rospy
from math import sqrt,cos,sin, atan2,copysign
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
            self._max_l_acc = max_linear_acceleration*(-1) #since we are using only deacceleration
            self._max_a_acc = max_angular_acceleration*(-1) #since we are using only deacceleration
    
    """
    'a' refers to angular component and 'l' refers to linear component
    
    @c      initial_l_vel, initial_a_vel: Stores magnitude of velocity
    @c      pose_theta                  : Stores the angle of the velocity vector to the X-axis
    @c      linear_dist,angular_dist    : Stores linear and angular distance to target
    @c      l_dist_stop, a_dist_stop    : Stores distance required for deaccelerating to 0 from maximum speed
    @c      l_dist_unacc, a_dist_unacc  : Stores the distance till point of deacceleration, i.e., (linear_dist-l_dist_stop)
    @c      linear_vel, angular_vel     : Stores effective velocity to be published
    @c      linear_vel_x, linear_vel_y  : Stores X and Y component of linear_vel
    @c      l_time, a_time              : Stores time required to complete motion, used for scaling velocities    
    @c      scaled_l_vel,scaled_a_vel   : Stores scaled down values of velocities so that both angular and linear motions finish simulteneously
    
    """    

    
    def compute_velocity(self, actual_pose):
            rospy.loginfo(actual_pose)
            
            initial_l_vel=abs(self._l_max_vel)
            initial_a_vel=abs(self._a_max_vel)
            
            #Displacement and orientation to the target in world frame:
            dx = self._target_pose.x - actual_pose.x
            dy = self._target_pose.y - actual_pose.y
            pose_theta=get_shortest_angle(atan2(dy, dx),actual_pose.theta)
            
            # Step 1: compute remaining distances
            linear_dist = get_distance(self._target_pose, actual_pose)
            angular_dist = get_shortest_angle(self._target_pose.theta, actual_pose.theta)
              
            #Checking if target is reached
            if (abs(linear_dist)<self._l_tolerance and
                abs(angular_dist)<self._a_tolerance     ):
                
                rospy.loginfo("Completing.....")  
                rospy.loginfo(actual_pose)            
                self._linear_complete = True
                self._angular_complete = True
                return Velocity()
            
            #calculate distance to stop
            l_dist_stop=abs(-(self._l_max_vel**2)/(2*self._max_l_acc))
            a_dist_stop=abs(-(self._a_max_vel**2)/(2*self._max_a_acc))            
            
            #calculate distance for fixed velocity and deaccelerated velocity
            l_dist_unacc =  abs(linear_dist - l_dist_stop)
            a_dist_unacc=   abs(angular_dist - a_dist_stop)
           

            
            
            
            # Step 2: compute velocities
            linear_vel, linear_vel_x, linear_vel_y, angular_vel = 0, 0, 0, 0   
            scaled_l_vel,scaled_a_vel=0,0
            
            
            if(linear_dist<l_dist_stop and angular_dist< a_dist_stop):
                #Deaccelerate  both (A) 
                rospy.loginfo("Undergoing Linear and angular deacceleration")

                linear_vel=abs(sqrt(abs(-(2*self._max_l_acc*linear_dist))))
                angular_vel=abs(sqrt(abs(-(2*self._max_a_acc*angular_dist)))) 
                
                l_time=abs((-(linear_vel)/self._max_l_acc))
                a_time=abs((-(angular_vel)/self._max_a_acc))
           
                            
            elif(linear_dist>l_dist_stop and angular_dist< a_dist_stop):
                #Deaccelerate angular (B)
                rospy.loginfo("Undergoing angular deacceleration")

                linear_vel=abs(initial_l_vel)
                angular_vel=abs(sqrt(abs(-(2*self._max_a_acc*angular_dist))) )
                
                l_time_1=(l_dist_unacc/linear_vel)
                l_time_2=abs((-(linear_vel)/self._max_l_acc))
                l_time=l_time_1+l_time_2
                
                a_time=abs((-(angular_vel)/self._max_a_acc))
                
            elif(linear_dist<l_dist_stop and angular_dist>a_dist_stop):
                #Deaccelerate linear (C)
                rospy.loginfo("Undergoing linear deacceleration")
                linear_vel=abs(sqrt(abs(-(2*self._max_l_acc*linear_dist))))
                angular_vel=abs(initial_a_vel)
                
                a_time_1=(a_dist_unacc/angular_vel)
                a_time_2=abs((-(angular_vel)/self._max_a_acc))
                a_time=a_time_1+a_time_2
                
                l_time=abs((-(linear_vel)/self._max_l_acc))
                


            elif(linear_dist>l_dist_stop and angular_dist>a_dist_stop):
                #No deacceleration (D)
                rospy.loginfo("No deacceleration")
                
                linear_vel=abs(initial_l_vel)
                angular_vel=abs(initial_a_vel)

                l_time_1=(l_dist_unacc/linear_vel)
                l_time_2=abs((-(linear_vel)/self._max_l_acc))
                l_time=l_time_1+l_time_2
                
                a_time_1=(a_dist_unacc/angular_vel)
                a_time_2=abs((-(angular_vel)/self._max_a_acc))
                a_time=a_time_1+a_time_2                

            #scaling velocities so that both motion ends at same time             
            if (a_time<l_time):
                    scaled_a_vel=angular_vel*(a_time/l_time)
                    scaled_l_vel=linear_vel
            else :
                    scaled_a_vel=angular_vel
                    scaled_l_vel=linear_vel*(l_time/a_time)         
            
            if abs(linear_dist)>self._l_tolerance:
                rospy.loginfo("Calculating Linear vel") 
                linear_vel_x = abs((scaled_l_vel*cos(pose_theta) if abs(linear_dist)>self._l_tolerance else
                               self._l_tolerance*cos(pose_theta)))
                linear_vel_y = abs((scaled_l_vel*sin(pose_theta) if abs(linear_dist)>self._l_tolerance else
                               self._l_tolerance*sin(pose_theta)))
                linear_vel=abs(sqrt(linear_vel_x**2+linear_vel_y**2))
                rospy.loginfo(copysign(linear_vel, linear_dist))  
                rospy.loginfo(copysign(linear_vel_x, dx))  
                rospy.loginfo(copysign(linear_vel_y, dy))
            if abs(angular_dist)>self._a_tolerance:
                rospy.loginfo("Calculating angular vel")
                angular_vel=abs((scaled_a_vel if abs(angular_dist)>self._a_tolerance else
                               self._a_tolerance))
                rospy.loginfo(copysign(angular_vel, angular_dist))
          
            return Velocity(copysign(linear_vel_x, dx),
                            copysign(linear_vel_y, dy),
                            copysign(angular_vel, angular_dist))
            

            
            
            