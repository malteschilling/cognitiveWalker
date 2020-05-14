from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
from controller.reaCog.MotivationNetwork.PhasicUnit import PhasicUnit
from controller.reaCog.MotivationNetwork.ModulatingMotivationUnit import ModulatingMotivationUnit
from controller.reaCog.MotivationNetwork.ConnectionModulatorUnit import ConnectionModulatorUnit
from controller.reaCog.MotivationNetwork.DelayUnit import DelayUnit
from controller.reaCog.walknet.MotivationNetwork.ProblemDetectors import UnstableProblemDetector

from controller.reaCog.MotivationNetwork.WTAUnit import WTAUnitFast

import controller.reaCog.WalknetSettings as WSTATIC

class CognitivePhases(object):

    def __init__(self, cogn_exp, learn_fct, reset_fct):
        self.cognitive_expansion = cogn_exp
    
        # Different Phases Of the search - each has one MotivationUnit modulating the
        # specific actions during that MU_SAL
        # 0. Normal Behaviour
        self.MU_Beh = MotivationUnit("Phase_Beh", 1, group_name='cognitive_expansion')
        
        # TODO: introduce stop of behavior. 
        # Remove the inhibition from ProblemDetectors on swing in MotivationNetRobot 
        
        # 1. Spreading of Activation
        self.MU_SAL = ConnectionModulatorUnit("Phase_SAL", group_name='cognitive_expansion')
        # 2. Selecting a Winner of the activated units
        self.MU_WTA = MotivationUnit("Phase_WTA", group_name='cognitive_expansion')
        # 3. Mental Simulation
        self.MU_Sim = MotivationUnit("Phase_Sim", group_name='cognitive_expansion')
        # Was learning - is now testing behaviour : ModulatingMotivationUnit("Phase_TestBeh", learn_fct , group_name='cognitive_expansion')
        # 4. Test Behaviour 
        self.MU_TestBeh = MotivationUnit("Phase_TestBeh", group_name='cognitive_expansion')

        self.Phase_MUs = [self.MU_Beh, self.MU_SAL, self.MU_WTA, self.MU_Sim, self.MU_TestBeh]

        # WTA Structure of the Phase Motivation Units.
        for i in range(0,5):
            self.Phase_MUs[i].addConnectionTo(self.Phase_MUs[i], 1)
            if (i<4):
                self.Phase_MUs[i].addConnectionTo(self.Phase_MUs[i+1], -0.25)
                self.Phase_MUs[i+1].addConnectionTo(self.Phase_MUs[i], -0.25)
        self.Phase_MUs[1].changeConnectionWeight(self.Phase_MUs[1], 0.75)

        self.Phase_MUs[4].addConnectionTo(self.Phase_MUs[0], -0.25)
        self.Phase_MUs[0].addConnectionTo(self.Phase_MUs[4], -0.25)
        #(ws*MUSAL + wi*MUWTA + wi* MUSIM + wi*MUTestBeh + wi*MUBehav )  - SALlimit + gPDeton*5. + NewSearch*5.

        #self.ProblemDetectorSum = prob_detect
        #self.ProblemDetectorSum.addConnectionTo(self.MU_Beh, -1)
        #self.ProblemDetectorSum.addConnectionTo(self.MU_SAL, 1)
        self.cognitive_expansion.motivationNetRobot.MU_StartPlan.addConnectionTo(self.MU_Beh, -1)
        self.cognitive_expansion.motivationNetRobot.MU_StartPlan.addConnectionTo(self.MU_SAL, 1)
        #TODO: ADD SWITCH
        self.ProblemDetectorDelay = MotivationUnit("PD_Delay_On", group_name='cognitive_expansion')
        self.cognitive_expansion.motivationNetRobot.MU_StartPlan.addConnectionTo(self.ProblemDetectorDelay, 1)
        #self.ProblemDetectorSum.addConnectionTo(self.ProblemDetectorDelay, 1)
        self.ProblemDetectorDelay.addConnectionTo(self.MU_SAL, -1) # was -1
        self.ProblemDetectorDelay.addConnectionTo(self.MU_WTA, 1)
        
        self.MU_Beh.addConnectionTo(self.MU_WTA, -4)
        self.MU_TestBeh.addConnectionTo(self.MU_WTA, -4)

#       WTAUnitFast.foundWinner.addConnectionTo(self.MU_Sim, 1)
#       WTAUnitFast.foundWinner.addConnectionTo(self.MU_WTA, -2)
        self.MU_Sim.addConnectionTo(self.ProblemDetectorDelay, -1)
        self.MU_Sim.addConnectionTo(self.MU_SAL, -2)

        # Timed Transition from Simulation to Behavior
        # Learning is skipped
        
        self.SimulationCounter = DelayUnit("SimulationCounter", group_name='cognitive_expansion', delay=WSTATIC.unstable_slow_down_window+300)
        self.MU_Sim.addConnectionTo(self.SimulationCounter, 1)
        self.SimulationCounter.addConnectionTo(self.MU_Sim, -2)
        self.SimulationCounter.addConnectionTo(self.MU_TestBeh, 0.75)

        # Going into Learning phase
        self.MU_TestBeh.addConnectionTo(self.MU_SAL, -2)
        self.MU_TestBeh.changeConnectionWeight(self.MU_Sim, -1)
        
        self.MU_Beh.addConnectionTo(self.MU_Sim, -1)
        
#8        self.MU_TestBeh.addConnectionTo(self.ProblemDetectorSum, -1)
        self.MU_TestBeh.addConnectionTo(self.cognitive_expansion.motivationNetRobot.MU_StartPlan, -1)
        
        self.TestBehaviourCounter = DelayUnit("TestBehaviourCounter", group_name='cognitive_expansion', delay=WSTATIC.unstable_slow_down_window)
        self.MU_TestBeh.addConnectionTo(self.TestBehaviourCounter, 1)
        self.TestBehaviourCounter.addConnectionTo(self.MU_TestBeh, -2)
        self.TestBehaviourCounter.addConnectionTo(self.MU_Beh, 2)
        
        # Reset the behavioral search network:
        self.ResetBehavioralSearch = ModulatingMotivationUnit("Reset_Beh_Search", self.cognitive_expansion.resetBehaviorSearch, group_name='cognitive_expansion')
        self.TestBehaviourCounter.addConnectionTo(self.ResetBehavioralSearch, 1)
        
#       self.MU_TestBeh.addConnectionTo(self.MU_Beh, 2)     
        # TODO: rearrange diagonal 
#       self.MU_TestBeh.addConnectionTo(self.MU_TestBeh, -1)
        
        self.old_phase = 0
        
        self.MU_NextRun = ModulatingMotivationUnit("Reset_Phases", reset_fct, bias=-2, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_HL = UnstableProblemDetector("pd_mmc_check_unstable_hl", self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot, self.cognitive_expansion.motivationNetRobot, 4, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_HL.addConnectionTo(self.MU_NextRun, 1)
        
        self.pD_MMC_unstableCheck_HR = UnstableProblemDetector("pd_mmc_check_unstable_hr", self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot, self.cognitive_expansion.motivationNetRobot, 5, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_HR.addConnectionTo(self.MU_NextRun, 1)
        
        self.MU_Sim.addConnectionTo(self.MU_NextRun, 2)
        #self.IMProblemDetectorDelay = MotivationUnit("IMPD_Delay_On", group_name='cognitive_expansion')
        #self.IMProblemDetectorSum.addConnectionTo(self.IMProblemDetectorDelay, 1)
        #self.IMProblemDetectorDelay.addConnectionTo(self.MU_SAL, -1)
        #self.IMProblemDetectorDelay.addConnectionTo(self.MU_WTA, 1)
        
        # When the internal simulation is getting instable, 
        # the just starting (and causing) swing movement should be suppressed for a short time
        self.pD_MMC_unstableHL_delay = PhasicUnit("pD_MMC_unstableHL_delay", group_name='cognitive_layer', bias=-1, time_window =  10)
#26        self.pD_MMC_unstableHL_delay.addConnectionTo(self.motivationNetRobot.stance_forward_velocity, -2)
        self.pD_MMC_unstableCheck_HL.addConnectionTo(self.pD_MMC_unstableHL_delay, 1)
        
        self.pD_MMC_unstableHR_delay = PhasicUnit("pD_MMC_unstableHR_delay", group_name='cognitive_layer', bias=-1, time_window =  10)
#26        self.pD_MMC_unstableHR_delay.addConnectionTo(self.motivationNetRobot.stance_forward_velocity, -2)
        self.pD_MMC_unstableCheck_HR.addConnectionTo(self.pD_MMC_unstableHR_delay, 1)
        
        # Enforcing that stance velocity is turned off for some time
        self.pD_MMC_unstableHL_delay.addConnectionTo(self.pD_MMC_unstableHL_delay, 2)
        self.pD_MMC_unstableHR_delay.addConnectionTo(self.pD_MMC_unstableHR_delay, 2)
        
        # Problem detector MMC front legs
        self.pD_MMC_unstableCheck_FL = UnstableProblemDetector("pd_mmc_check_unstable_fl", self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot, self.cognitive_expansion.motivationNetRobot, 0, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_FL.addConnectionTo(self.MU_NextRun, 1)
        self.pD_MMC_unstableCheck_FR = UnstableProblemDetector("pd_mmc_check_unstable_fl", self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot, self.cognitive_expansion.motivationNetRobot, 1, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_FR.addConnectionTo(self.MU_NextRun, 1)
        
        self.pD_MMC_unstableFL_delay = PhasicUnit("pD_MMC_unstableFL_delay", group_name='cognitive_layer', bias=-1, time_window =  10)
        self.pD_MMC_unstableCheck_FL.addConnectionTo(self.pD_MMC_unstableFL_delay, 1)
        self.pD_MMC_unstableFR_delay = PhasicUnit("pD_MMC_unstableFR_delay", group_name='cognitive_layer', bias=-1, time_window =  10)
        self.pD_MMC_unstableCheck_FR.addConnectionTo(self.pD_MMC_unstableFR_delay, 1)
        
        self.pD_MMC_unstableFL_delay.addConnectionTo(self.pD_MMC_unstableFL_delay, 2)
        self.pD_MMC_unstableFR_delay.addConnectionTo(self.pD_MMC_unstableFR_delay, 2)
        
        self.MU_Sim.addConnectionTo(self.pD_MMC_unstableHL_delay, 2)
        self.MU_Sim.addConnectionTo(self.pD_MMC_unstableHR_delay, 2)
        self.MU_Sim.addConnectionTo(self.pD_MMC_unstableFL_delay, 2)
        self.MU_Sim.addConnectionTo(self.pD_MMC_unstableFR_delay, 2)
        
        # TODO? Removed the delay - has to be applied on the original Problemdetector
#7        self.pD_MMC_unstableHL_delay.addConnectionTo(self.cognitive_expansion.motivationNetRobot.hind_left_leg.stance_motivation, 10)
#7        self.pD_MMC_unstableHL_delay.addConnectionTo(self.cognitive_expansion.motivationNetRobot.hind_left_leg.swing_motivation, -10)
        
#7        self.pD_MMC_unstableHR_delay.addConnectionTo(self.cognitive_expansion.motivationNetRobot.hind_right_leg.stance_motivation, 10)
#7        self.pD_MMC_unstableHR_delay.addConnectionTo(self.cognitive_expansion.motivationNetRobot.hind_right_leg.swing_motivation, -10)
        
        # During the TestBeh the walking also has to be slowed down
#       self.MU_TestBeh.addConnectionTo(self.pD_MMC_unstableHL_delay, 2)
        self.MU_Sim_delay = PhasicUnit("MU_Sim_delay", group_name='cognitive_layer', time_window =  WSTATIC.unstable_slow_down_window)
        self.MU_Sim.addConnectionTo(self.MU_Sim_delay, 1)
        self.MU_Sim_delay.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        self.MU_TestBeh.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        
        self.MU_SAL.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        self.MU_WTA.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        self.transitionInhibitor = MotivationUnit("transInhibition")
        self.MU_WTA.addConnectionTo(self.transitionInhibitor, 1)
        self.transitionInhibitor.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        
#       self.testBehaviour_delay = PhasicUnit("testBehaviour_delay", group_name='cognitive_layer', time_window =  WSTATIC.unstable_slow_down_window)
#       self.testBehaviour_delay.addConnectionTo(self.motivationNetRobot.stance_forward_velocity, -1)
#       self.MU_TestBeh.addConnectionTo(self.testBehaviour_delay, 1)
#       # Enforcing that stance velocity is turned off for some time
#       self.testBehaviour_delay.addConnectionTo(self.testBehaviour_delay, 2)
        
        self.it = 0

    def printCurrentPhaseValues(self):
        print("Beh: ", self.MU_Beh.output_value, " - SAL: ", self.MU_SAL.output_value, " - WTA: ", 
            self.MU_WTA.output_value, " - Sim: ", self.MU_Sim.output_value, " - TestBeh: ", 
            self.MU_TestBeh.output_value)
        #idea_str = "Idea:   "; wta_str = "WTA:    ";
        #for i in range(6):
        #   idea_str = idea_str + '{:.4}'.format(float(self.motivationNetRobot.idea_MU[i].output_value)) + "\t"
        #   wta_str = wta_str + '{:.4}'.format(float(self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value)) + "\t"
        #print(idea_str);
        #print(wta_str);
    
    def getCurrentPhase(self):
        max_act=0
        max_phase=0
        for i in range(len(self.Phase_MUs)):
            if (self.Phase_MUs[i].output_value > max_act):
                max_phase = i
            
        self.it +=1 
        
#        if (self.MU_SAL.output_value > 0.5):
 #           print("SAL ACTIVATION: ")
  #          for i in range(len(self.cognitive_expansion.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.original_MUs)):
   #             print(i, " - SAL: ", self.cognitive_expansion.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_SAL[i].output_value, 
    #                " ; WTA: ", self.cognitive_expansion.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value,
     #               " ; RTB: ", self.cognitive_expansion.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_RTB[i].output_value)
      #      input()

#30        if (self.MU_Sim.output_value > 0.5):
#30            print("SIMULATION, Pos ML: ", self.motivationNetRobot.motivationNetLegs[2].wleg.realLeg.leg.input_foot_position[0], 
#30                " / ", self.motivationNetRobot.motivationNetLegs[2].wleg.mmcLeg.leg.input_foot_position[0],
#30                " - sw_toFr: ", self.motivationNetRobot.motivationNetLegs[2].swing_motivation_toFront.output_value, 
#30                " - sw_toBack: ", self.motivationNetRobot.motivationNetLegs[2].swing_motivation_toBack.output_value, 
#30                " - idea_timer: ", self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.modulate_idea_input_time_window.current_count,
#30                " - vel: ", self.motivationNetRobot.motivationNetLegs[2].wleg.realLeg.controlVelocities, 
#30                " / ", self.motivationNetRobot.motivationNetLegs[2].wleg.mmcLeg.controlVelocities)
        
        #self.printCurrentPhaseValues()
        #print("In CP updated: ", self.motivationNetRobot.wrobot.mmcRobot.gc)
        #for motiv_leg in self.motivationNetRobot.motivationNetLegs:
        #print("Swing HL: ", self.motivationNetRobot.motivationNetLegs[4].swing_motivation_toFront.output_value, self.motivationNetRobot.motivationNetLegs[4].swing_motivation.output_value )
#        print( (2*self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_WTA[4].output_value) )
 #       print( self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.modulate_idea_input_time_window.output_value)
  #      print( self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.modulate_idea_input.output_value)
    
            #input()
#       if (self.MU_Sim.output_value > 0.):
#           #print("Unstable: ", self.pD_MMC_unstableHL_delay.output_value, " - ", self.pD_MMC_unstableCheck_HL.output_value)
#           print("ML SW: ", self.motivationNetRobot.middle_left_leg.swing_motivation_toBack.output_value,
#               " - Stance: ", self.motivationNetRobot.stance_forward_velocity.output_value)
#           print("WTA - ", 2, " = ", self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_WTA[2].output_value, WTAUnitFast.foundWinner.output_value)
                    
#       self.it +=1 
        if not(max_phase == self.old_phase):
            print("Switched to Phase ", max_phase, " = ", self.Phase_MUs[max_phase].name, " : ", self.pD_MMC_unstableCheck_HL.iteration) 
            self.printCurrentPhaseValues()
            #input()
            
            #if (self.MU_WTA.output_value > 0.) or (self.MU_SAL.output_value > 0.):
             #   print("WTA winner ", self.MU_WTA.output_value, WTAUnitFast.foundWinner.output_value)
              #  for i in range(0, len(self.cognitive_expansion.SAL_WTA_Net.layer_SAL) ):
               #     print("WTA - ", i, " = ", self.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value, 
                #        " SAL : ", self.cognitive_expansion.SAL_WTA_Net.layer_SAL[i].output_value,
                 #       " RTB : ", self.cognitive_expansion.SAL_WTA_Net.layer_RTB[i].output_value)
            
            if (self.MU_Sim.output_value > 0.1 or self.MU_TestBeh.output_value > 0.1):
                current_idea = 0
                for i in range(0, len(self.cognitive_expansion.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_SAL) ):
                    if (self.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value > 0.5):
                        current_idea = i
                print("========================================================")
                print("Selected idea as a new behavior: ", current_idea, " = ", 
                    self.cognitive_expansion.SAL_WTA_Net.original_MUs[current_idea].name)
                print("========================================================")
#                print("PD HL: ", self.pD_MMC_unstableCheck_HL.iteration, self.pD_MMC_unstableCheck_HL.output_value, self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot.temp_stability_fact)
 #               print("PD HR: ", self.pD_MMC_unstableCheck_HR.iteration, self.pD_MMC_unstableCheck_HR.output_value, self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot.temp_stability_fact)
  #              print(self.MU_NextRun.output_value)
                
   #             print(self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot.gc)
#                for mleg in self.cognitive_expansion.motivationNetRobot.motivationNetLegs:
 #                   print("GC: ", mleg.wleg.realLeg.predictedGroundContact(), 
  #                      " - SWING: ", mleg.swing_motivation_toFront.output_value, mleg.swing_starter.output_value,
   #                     " - STANCE: ", mleg.stance_motivation_toBack.output_value, 
    #                    " - GC Sim: ", mleg.wleg.mmcLeg.predictedGroundContact() )             
                #input()
        self.old_phase = max_phase
        return max_phase
