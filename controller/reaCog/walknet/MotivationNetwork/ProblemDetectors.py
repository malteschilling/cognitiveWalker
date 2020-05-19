#!/usr/bin/env python
'''
Created on 17.7.2014

@author: mschilling
'''
from ... import WalknetSettings as WSTATIC 
from controller.reaCog.walknet.MotivationNetwork.SensorUnit import SensorUnit
from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import getCurrentTime as getCurrentTime

import Hector.RobotSettings as RSTATIC

""" A Problem Detector Unit for detecting Instability.
"""
class UnstableProblemDetector(SensorUnit):
    def __init__(self, name, body_model_stance, motivationNet, leg_nr, startValue=0, bias=0, group_name=''):
        SensorUnit.__init__(self, name=name, group_name=group_name)
        self.mmcStanceModel = body_model_stance
        self.motivationNet = motivationNet
        self.leg_nr = leg_nr
        self.stable = True
        self.iteration = 0
        self.iteration_instable = -1

    ##
    #   Check if the Body Model configuration is static stable.
    #   In this version two line equations are used (in parametric version)
    #       - diagonal between left and right most hind leg with gc
    #       - line along the middle segment
    #   It is calculated where those two lines intersect, i.e. with respect to the 
    #   middle segment: the segment factor specifies this intersection as expressed
    #   in the line equation of this line: a negative value means that this 
    #   point is behind the middle segment (= the diagonal between the most hind legs
    #   lies behind the center of gravity). For a positive value the factor describes
    #   how CoG and the hind line of the polygon of static stability relate.
    def updateSensorValue(self):
        if not(self.mmcStanceModel.get_current_static_stability_caused_by_leg(self.leg_nr)):
            if self.stable:
                self.stable = False
                self.iteration_instable = self.iteration
                print("INSTABLE IN: ", self.iteration_instable, " - ", self.name, " = ", 
                    self.mmcStanceModel.temp_stability_fact_back, self.mmcStanceModel.temp_stability_fact_front, [self.mmcStanceModel.get_ground_contact(i) for i in range(0,6)],
                    [(self.motivationNet.wrobot.wRobot.wlegs[i].predictedGroundContact(), 
                        self.motivationNet.wrobot.wRobot.wlegs[i].leg.input_foot_position[2]) for i in range(0,6)])
            else:
                self.output_value = 1.
        else:
            self.output_value = 0
            self.stable = True
        self.iteration += 1        