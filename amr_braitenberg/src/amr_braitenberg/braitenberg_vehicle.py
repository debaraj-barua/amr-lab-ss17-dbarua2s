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
        ws1=0.0
        ws2=0.0
        if (self._vehicle_type==0):
            
            print "0000000000000000000000000000000000"
            print "left_in: ",left_in
            print "right_in: ",right_in
            ws1=0.0
            ws2=0.0   

        elif (self._vehicle_type==1):
            print "1111111111111111111111111111111111"
            print "left_in: ",left_in
            print "right_in: ",right_in
            ws1=0.0
            ws2=0.0   

        elif (self._vehicle_type==2):
            print "2222222222222222222222222222222222"
            print "left_in: ",left_in
            print "right_in: ",right_in
            ws1=0.0
            ws2=0.0   

        else:
            print "ELSEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE"
            print "left_in: ",left_in
            print "right_in: ",right_in            
            ws1=0.0
            ws2=0.0   


        return (ws1, ws2)