# -*- coding: utf-8 -*-
import numpy
import time

#Tools
from .ProcessModule.ProcessingModule import ProcessingModule
import controller.reaCog.WalknetSettings as WSTATIC
from tools.FreezableF import Freezable as Freezable
from decimal import Decimal

##
# Drive robot into a given start posture
##
class StartPostureModule (ProcessingModule, Freezable):
    __current_time=Decimal('0')
    time_precision=Decimal('0.0001')
    ##
    #   Init
    #   @param name for the module
    #   @param  the Hector robot structure (to access data and communication protocol)
    #   @posture describes the posture - given for each leg (FL, FR, ML, ...)
    #       a x-position in the leg coordinate system has to be provided
    def __init__(self, name, robot, posture):
        self.name = name
        self.robot = robot
#       self.communication_interface = robot.communication_interface
        
        ProcessingModule.__init__(self, name)
        # Create a communication client for the timing of the simulation
#       self.timer=self.communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])
        self.it = -10
        self.init_leg = -1
        
        self.start_posture = [ numpy.array(WSTATIC.front_initial_aep), \
            numpy.array(WSTATIC.front_initial_aep), \
            numpy.array(WSTATIC.middle_initial_aep), \
            numpy.array(WSTATIC.middle_initial_aep), \
            numpy.array(WSTATIC.hind_initial_aep), \
            numpy.array(WSTATIC.hind_initial_aep)]

        for i in range(0,6):
            self.start_posture[i][0]= posture[i]
        self.posture = posture
        self.start_posture[2][0]-= -0.0005
 #       self.start_posture[1][0]= -0.0395
        
        # Adjust for left legs the y position (switch sign)
        self.start_posture[0][1] = -self.start_posture[0][1]
        self.start_posture[2][1] = -self.start_posture[2][1]
        self.start_posture[4][1] = -self.start_posture[4][1]

        self.angles = []
        self.frozen=True
        
    def processing_step(self, timeStamp):
        if ( 0 <= self.it ):
            if (self.it % 10 == 0):
                self.init_leg += 1
                if (self.init_leg < 6):
                    self.it = 0
                    #print("START: ", self.robot.robot.legs[self.init_leg].getInputAngles(), " - ", self.init_leg)
                    self.angles = self.robot.robot.legs[self.init_leg].computeInverseKinematics( self.start_posture[self.init_leg] )
                    #print("GOAL:  ", self.angles)
            if (self.init_leg < 6):
                self.robot.robot.legs[self.init_leg].alpha.desiredPosition = float(self.angles[0])
                self.robot.robot.legs[self.init_leg].beta.desiredPosition = float(self.angles[1])
                self.robot.robot.legs[self.init_leg].gamma.desiredPosition = float(self.angles[2])
                #print(self.robot.robot.legs[self.init_leg].getInputAngles())
            if (self.init_leg == 6):
                print("Init posture finished " + str(self.posture))
                self.init_leg +=1
        self.it += 1
        
    ## 
    #   Init the module - called before it is iterated over all the modules
    def init_module(self):
        if __debug__:
            print('Init posture module')