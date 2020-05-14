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
class ProblemDetectorStability(SensorUnit):
    def __init__(self, name, body_model_stance, motivationNet, leg_nr, old_pd, startValue=0, bias=0, group_name=''):
        SensorUnit.__init__(self, name=name, group_name=group_name)
        self.mmcStanceModel = body_model_stance
        self.motivationNet = motivationNet
        self.leg_nr = leg_nr
        if (leg_nr == 0):
            self.leg_support_order = [[2,4],[1,3,5], (lambda x: x < (WSTATIC.stability_cog + WSTATIC.stability_cog_threshold))]
        elif (leg_nr == 1):
            self.leg_support_order = [[0,2,4],[3,5], (lambda x: x < (WSTATIC.stability_cog + WSTATIC.stability_cog_threshold))]
        elif (leg_nr == 2):
            self.leg_support_order = [[4,0],[5,3,1], (lambda x: x > (WSTATIC.stability_cog - WSTATIC.stability_cog_threshold))]
        elif (leg_nr == 3):
            self.leg_support_order = [[4,2,0],[5,1], (lambda x: x > (WSTATIC.stability_cog - WSTATIC.stability_cog_threshold))]
        elif (leg_nr == 4):
            self.leg_support_order = [[2,0],[5,3,1], (lambda x: x > (WSTATIC.stability_cog - WSTATIC.stability_cog_threshold))]
        elif (leg_nr == 5):
            self.leg_support_order = [[4,2,0],[3,1], (lambda x: x > (WSTATIC.stability_cog - WSTATIC.stability_cog_threshold))]        
        self.stable = True
        self.iteration = 0
        self.iteration_instable = -1
        self.old_pd = old_pd
        
    def updateSensorValue(self):
        #NEW: if self.stable and (self.mmcStanceModel.get_ground_contact(self.leg_nr) ):
        if (self.mmcStanceModel.get_ground_contact(self.leg_nr) ):
            self.output_value = 0
            self.stable = True
        else:
            left_stability = False
            right_stability = False
            
#            gc = [0,0,0,0,0,0]
 #           for i in range(0,6):
  #              gc[i] = self.mmcStanceModel.get_ground_contact(i)
   #         print("CURRENT GC: ", gc)

            for leg in self.leg_support_order[0]:
                if (getattr(self.motivationNet, RSTATIC.leg_names[leg])).wleg.predictedGroundContact() :
                    left_stability = True
    #                print("Last left leg gc: ", leg)
                    break
            left_leg = leg
            for leg in self.leg_support_order[1]:
                if (getattr(self.motivationNet, RSTATIC.leg_names[leg])).wleg.predictedGroundContact() :
                    right_stability = True
     #               print("Last right leg gc: ", leg)
                    break
            right_leg = leg
            temp_stability_fact = self.mmcStanceModel.swig_stance_body_model.check_static_stability(left_leg, right_leg)
            #print("Lambda: ", self.leg_support_order[2](temp_stability_fact), " - ", (temp_stability_fact > WSTATIC.stability_threshold))
            gc = [0,0,0,0,0,0]
            for i in range(0,6):
                gc[i] = self.mmcStanceModel.get_ground_contact(i)
#            if (self.name == 'pd_check_unstable_fr_new'):
 #               print("Leg: ", temp_stability_fact, " - ", gc)
            if not(left_stability and right_stability) or (self.leg_support_order[2](temp_stability_fact) ):
                self.stable = False
#                print("NEW INSTABLE IN: ", self.iteration, " - ", self.name, " = ", temp_stability_fact)
                self.output_value = 1.
                self.iteration_instable = self.iteration
            else:
                self.output_value = 0
                self.stable = True
                
        self.iteration += 1
        #print("New PD - ", self.name, " : ", self.output_value, " - ", self.old_pd.output_value) 