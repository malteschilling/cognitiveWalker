## -*- coding: utf-8 -*-
#Tools
from ..ProcessModule.ProcessingModule import ProcessingModule
from tools.FreezableF import Freezable as Freezable
from decimal import Decimal

import time

def getCurrentTime():
    return float(SimulatorTimerModule.getCurrentTime())

##
#   The simulator module basically encapsulates the specifics of the simulator,
#   i.e. the difference to a real environment.
#   In the simulator case the simulation has to be advanced after each control
#   step (meaning in the post control step).
##
class SimulatorTimerModule (ProcessingModule, Freezable):
    __current_time=Decimal('0')
    time_precision=Decimal('0.0001')
    ##
    #   Init
    #   @param name for the module
    #   @param  the Hector robot structure (to access data and communication protocol)
    #   @controllerFrequency frequency of the controller
    def __init__(self, name, robot, controllerFrequency):
        self.name = name
        self.robot = robot
        self.frame = 0
        self.communication_interface = robot.communication_interface
        self.controllerFrequency = controllerFrequency
        ProcessingModule.__init__(self, name)
        # Create a communication client for the timing of the simulation
        self.timer=self.communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])
        self.it = 0
        self.imu=self.communication_interface.CreateBfbClient(120, ["IMU_PROT"])
        self.frozen=True
        
        self.timer.cameraPosition = str([ [.75, -1.2, 0.8], [90.0, -24.0, 0.0] ])
        
        self.timer.cameraPosition = str([ [0.75, 1.2, 0.13], [-90.0, 0.0, 0.0] ])
        # perspective SIDE VIEW, see robot left legs
        #self.timer.cameraPosition = str([ [1.0, -1.8, 1.0], [90.0, -17.0, 0.0] ])
        #self.timer.cameraPosition = str([ [0.25, -1., 1.], [90.0, -45.0, 0.0] ])
        # Top View
        # self.timer.cameraPosition = str([ [1.0, 0., 1.8], [-90.0, -90.0, 0.0] ])
        
    @classmethod
    def getCurrentTime(cls):
        return cls.__current_time
        
    ##
    #   Post processing: Sending data to the Simulator after the control values
    #   are calculated.
    #   @param timeStamp current simulator time
    def post_processing_step(self, timeStamp):
        # Capture every simulation step
        #if (self.it % 4 == 0):                             # Auskommentieren fuer abschalten Einzelbilder aufnehmen
         #   self.timer.captureFrame = float(self.it)/40000 # Auskommentieren fuer abschalten Einzelbilder aufnehmen
        #   time.sleep(0.02)
        # Capture every simulation step
        if (self.it % 2):
            robot_pos = [round(i,2) for i in self.imu.position]
            # Moving camera
            #self.timer.cameraPosition = str([ [1.0+robot_pos[0], 1.8, 1.0], [-90.0, -17.0, 0.0] ])
            # Capturing frames turned currently off
            #self.timer.captureFrame = float(self.__current_time)/100
        #   time.sleep(0.02)
        self.it += 1
        # Let the simulation run for a given time.
        self.timer.relTimerMs=1/self.controllerFrequency
        SimulatorTimerModule.__current_time+=Decimal(1/self.controllerFrequency ).quantize(self.time_precision)
#       self.timer.robotTransparency = 0.59181716
#       self.timer.internalModelGlobalPosition = str([[2., 1.],\
#               [2., 1.],\
#               [0., 0.],\
#               [1., 0.],\
#               [1.03, 0.],\
#               [1., 0.]])
    ## 
    #   Init the module - called before it is iterated over all the modules
    def init_module(self):
        if __debug__:
            print('Init simulator timing module')
