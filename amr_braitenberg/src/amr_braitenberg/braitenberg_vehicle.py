#!/usr/bin/env python


class BraitenbergVehicle:
    
    TYPE_A = 0  # direct connections
    TYPE_B = 1  # cross connections
    TYPE_C = 2  # direct and cross connections
        
    def __init__(self, *args):
        """
        init with default params (type A, factor 1.0)
        """
        self.set_params()
        pass
    
    
    def set_params(self, vehicle_type=TYPE_A, factor_1=1.0, factor_2=1.0):
        self._vehicle_type = vehicle_type
        self._f_1, self._f_2 = factor_1, factor_2
    
    


    def compute_wheel_speeds(self, left_in, right_in):
        """
        ==================== YOUR CODE HERE ====================
        Instructions: based on the input from the left and
                      right sonars compute the speeds of the
                      wheels. Use the parameters stored in the
                      private fields self._vehicle_type, self._f_1, and
                      self._f_2 (if applicable).

        Hint: a good idea would be to pass here the normalized sonar
        readings scaled by maximum range, i.e. proximity to an obstacle
        (in interval [0..1])
        ========================================================
        """
        """
        This function gets left and right sonar range readings and calculate wheel speed
        based on the type of Braitenberg vehicle.
        
        @c ws1, ws2: These variables store wheel speeds for left and right wheels respectively.
        
        @c max_range: This variable stores maximum range of the sonar sensor
        
        @c normalized_left,normalized_right:
                    This variable is used to store normalized sensor readings 
                    scaled by maximum range.
                    For normalized_left, this is calculated by: 
                            (max_range-left_in)/max_range
                    This will result in a value between 0 and 1. 
                    
        """
        
        ws1=0.0
        ws2=0.0
        max_range=5
        normalized_left=(max_range-left_in)/max_range       
        normalized_right=(max_range-right_in)/max_range     

        if (self._vehicle_type==0):
            
            ws1=normalized_left*self._f_1
            ws2=normalized_right*self._f_2

        elif (self._vehicle_type==1):
           
            ws1=normalized_right*self._f_1
            ws2=normalized_left*self._f_2 

        elif (self._vehicle_type==2):
            
            ws1=(normalized_right+normalized_left)*self._f_1
            ws2=(normalized_right+normalized_left)*self._f_2 

        else:
            """
            wheel speed zero if parameter of vehicle type is other then 0, 1 or 2
            Generally will not execute, but included as a redundant condition
            """
            ws1=0.0
            ws2=0.0   
            

        return (ws1, ws2)