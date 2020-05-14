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
        
    def updateSensorValue(self):
        #self.mmcStanceModel.check_static_stability_along_segment()
#        if (self.name == 'pd_check_unstable_hl'):
 #           print("Stability HL ", self.mmcStanceModel.temp_stability_fact)
        if not(self.mmcStanceModel.get_current_static_stability_caused_by_leg(self.leg_nr)):
        #if not(self.mmcStanceModel.check_static_stability_along_segment()):
            if self.stable:
                self.stable = False
                self.iteration_instable = self.iteration
                print("INSTABLE IN: ", self.iteration_instable, " - ", self.name, " = ", 
                    self.mmcStanceModel.temp_stability_fact_back, self.mmcStanceModel.temp_stability_fact_front, [self.mmcStanceModel.get_ground_contact(i) for i in range(0,6)],
                    [(self.motivationNet.wrobot.wRobot.wlegs[i].predictedGroundContact(), 
                        self.motivationNet.wrobot.wRobot.wlegs[i].leg.input_foot_position[2]) for i in range(0,6)])
#                 if (self.name == 'pd_check_unstable_hl'):
#                     for mleg in self.motivationNet.motivationNetLegs:
#                         print("GC: ", mleg.wleg.realLeg.predictedGroundContact(), " - ", mleg.wleg.realLeg.leg.input_foot_position[2],
#                             " - SWING: ", mleg.swing_motivation_toFront.output_value, mleg.swing_starter.output_value,
#                             " - STANCE: ", mleg.stance_motivation_toBack.output_value )
# #                    self.motivationNet.debug = True
#                     #for i in range(0,6):
#                      #   print(self.mmcStanceModel.get_ground_contact(i))
#                     #print("Last leg on ground: : " , self.mmcStanceModel.left_leg, self.mmcStanceModel.right_leg)
#                     #input()
#                if (self.name == 'pd_check_unstable_hr'):
#                     for mleg in self.motivationNet.motivationNetLegs:
#                         print("GC: ", mleg.wleg.realLeg.predictedGroundContact(), " - ", mleg.wleg.realLeg.leg.input_foot_position[2],
#                             " - SWING: ", mleg.swing_motivation_toFront.output_value, mleg.swing_starter.output_value,
#                             " - STANCE: ", mleg.stance_motivation_toBack.output_value )                      
#                 if (self.name == 'pd_mmc_check_unstable_hl'):
#                     print(self.mmcStanceModel.gc)
#                     for mleg in self.motivationNet.motivationNetLegs:
#                         print("GC: ", mleg.wleg.realLeg.predictedGroundContact(), " - ", mleg.wleg.realLeg.leg.input_foot_position[2],
#                             " - SWING: ", mleg.swing_motivation_toFront.output_value, mleg.swing_starter.output_value,
#                             " - STANCE: ", mleg.stance_motivation_toBack.output_value,
#                             " - GC SIM: ", mleg.wleg.mmcLeg.predictedGroundContact() )
#                     #input()
#                 if (self.name == 'pd_mmc_check_unstable_hr'):
#                     print(self.mmcStanceModel.gc)
#                     for mleg in self.motivationNet.motivationNetLegs:
#                         print("GC: ", mleg.wleg.realLeg.predictedGroundContact(), " - ", mleg.wleg.realLeg.leg.input_foot_position[2],
#                             " - SWING: ", mleg.swing_motivation_toFront.output_value, mleg.swing_starter.output_value,
#                             " - STANCE: ", mleg.stance_motivation_toBack.output_value,
#                             " - GC SIM: ", mleg.wleg.mmcLeg.predictedGroundContact() )
            else:
                self.output_value = 1.
            #print("Unstable ", self.name)
        else:
            self.output_value = 0
            self.stable = True
#        print("Stability ", self.iteration, " - ", self.name, " = ", self.output_value, " / ", self.mmcStanceModel.temp_stability_fact_back)
        self.iteration += 1        
                                 
    ##
    #   Check if the BM configuration is static stable.
    #   In this version two line equations are used (in parametric version)
    #       - diagonal between left and right most hind leg with gc
    #       - line along the middle segment
    #   It is calculated where those two lines intersect, i.e. with respect to the 
    #   middle segment: the segment factor specifies this intersection as expressed
    #   in the line equation of this line: a negative value means that this 
    #   point is behind the middle segment (= the diagonal between the most hind legs
    #   lies behind the center of gravity). For a positive value the factor describes
    #   how CoG and the hind line of the polygon of static stability relate.
#   def check_static_stability_along_segment(self): 
#       stability = True
#       left_leg, right_leg = 4, 5
#       
#       # Test if CoG moves moves behind the connecting line
#       # connecting the leg on each side which
#       # - has gc
#       # - is the leg furthest backward having gc on that side
#       # inSwingPhase(self)
#       while left_leg>=0 and (getattr(self.motivationNet, RSTATIC.leg_names[left_leg])).inSwingPhase():
#           left_leg -= 2
#       while right_leg>0 and (getattr(self.motivationNet, RSTATIC.leg_names[right_leg])).inSwingPhase():
#           right_leg -= 2
#       # If there is no gc at all on one side it should be unstable
#       if (left_leg < 0) or (right_leg < 0):
#           stability = False
#       else:
#           #left_leg_obj = getattr(self.wRobot, RSTATIC.leg_names[left_leg])
#           #right_leg_obj = getattr(self.wRobot, RSTATIC.leg_names[right_leg])
#           #print(left_leg, right_leg, self.motivationNet.wrobot.hind_right_leg.predictedGroundContact(), self.motivationNet.hind_right_leg.inStancePhase() )
#           
#           #print(left_leg_obj.input_foot_position, right_leg_obj.input_foot_position)
#           #print(-self.mmcStanceModel.front_vect[left_leg][-1], self.mmcStanceModel.front_vect[right_leg][-1])
#           diag_vect = -self.mmcStanceModel.front_vect[left_leg][-1] + self.mmcStanceModel.front_vect[right_leg][-1] \
#                       + self.mmcStanceModel.get_segm_vectors_between_legs(left_leg, right_leg, -1)
#           left_foot_cog_vect = -self.mmcStanceModel.front_vect[left_leg][-1]
#           left_foot_cog_vect[2] = 0.
#           #print("Stability: ", left_leg, right_leg, diag_vect, left_foot_cog_vect)
#           segment_factor = (diag_vect[1]*left_foot_cog_vect[0] - diag_vect[0]*left_foot_cog_vect[1]) \
#               / (diag_vect[0]*self.mmcStanceModel.segm[1][-1][1] - diag_vect[1]*self.mmcStanceModel.segm[1][-1][0]) 
#           # Correction factor of the parameter:
#           # If the most hind leg is a middle leg, the factor has to be increased by one
#           # - if both are front legs, it has to be increased by two.
#           segment_factor += (2-max(left_leg,right_leg)//2)
#           #print("Stability Problem Detector ", segment_factor)
#           if segment_factor > 0.25:
#               #print("Stability along middle segment - legs: ", left_leg, right_leg, " - factor ", segment_factor)
#               stability = False
#               #input("Press Enter to continue...")
#       return stability
