from .CognitivePhases import CognitivePhases
from .SpreadAndSelectNetwork import SpreadAndSelectNetwork
from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

from ..MotivationNetwork.MotivationUnit import executeNeuralNetStep

from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import getCurrentTime as getCurrentTime

class CognitiveExpansion(object):

    def __init__(self, motiv_net, orig_MUs):
        self.motivationNetRobot = motiv_net

        self.Phase = CognitivePhases(self, self.learnCurrentWinner, self.resetSimulationLoop )

        self.SAL_WTA_Net = SpreadAndSelectNetwork(orig_MUs, self.Phase)
        
        # Induce information into the spreading activation layer
        # = where did the problem occur?
        # TODO: Already in MotivationNetRobot? There PD mapped to SAL,
        # Should be only here with new PDs!
        #self.prob_detect.addConnectionTo(self.SAL_WTA_Net.layer_SAL[2], 1)
#EXP_2018_01        
        self.motivationNetRobot.safeStop_HL.addConnectionTo(self.SAL_WTA_Net.layer_SAL[4], 1)
        #self.motivationNetRobot.safeStop_HL.addConnectionTo(self.SAL_WTA_Net.layer_SAL[1], 1)
        #EXP_2018_01 line above added
#        self.motivationNetRobot.safeStop_HL.addConnectionTo(self.SAL_WTA_Net.layer_WTA[2], -1)
#        self.motivationNetRobot.safeStop_HL.addConnectionTo(self.SAL_WTA_Net.layer_RTB[2], 1)
        
#EXP_2018_01        
        self.motivationNetRobot.safeStop_HR.addConnectionTo(self.SAL_WTA_Net.layer_SAL[6], 1)
        
        # Front leg spreading information
        self.motivationNetRobot.safeStop_FL.addConnectionTo(self.SAL_WTA_Net.layer_SAL[0], 1)
        self.motivationNetRobot.safeStop_FR.addConnectionTo(self.SAL_WTA_Net.layer_SAL[10], 1)
#        self.motivationNetRobot.safeStop_HR.addConnectionTo(self.SAL_WTA_Net.layer_WTA[4], -1)
#        self.motivationNetRobot.safeStop_HR.addConnectionTo(self.SAL_WTA_Net.layer_RTB[4], 1)
        #ToDo: all active behaviors should be inhibited
        #self.prob_detect.addConnectionTo(self.SAL_WTA_Net.layer_WTA[2], -1)

    def resetWTA(self):
        self.SAL_WTA_Net.resetWTA()
    
    def resetBehaviorSearch(self, activation, fct_param):
        self.SAL_WTA_Net.resetWTA()
        self.SAL_WTA_Net.resetSAL()
        self.SAL_WTA_Net.resetRTB()            
        print("Reset Cognitive Search for Behavior")
        #for i in range(len(self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.original_MUs)):
         #   print(i, " - SAL: ", self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_SAL[i].output_value, 
          #      " ; WTA: ", self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value,
           #     " ; RTB: ", self.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_RTB[i].output_value)

    def learnCurrentWinner(self, activation):
        self.SAL_WTA_Net.learnCurrentWinner(activation)
        self.resetWTA()
        TurnOff = MotivationUnit("TurnOFF_PD", bias=1, group_name='cognitive_expansion')
        TurnOff.addConnectionTo(TurnOff, 1)
#8        TurnOff.addConnectionTo(self.prob_detect, -1)

    def refreshInternalModelFromRobotData(self):
#       for wleg in self.motivationNetRobot.wrobot.wlegs:
#           print("REAL  BEFORE - ", wleg.realLeg.leg.name," = ", wleg.realLeg.leg.input_foot_position)
#           print("STANCE BEFORE - ", wleg.realLeg.leg.name," = ", self.motivationNetRobot.bodyModelStance.get_leg_vector(wleg.realLeg.leg.name) )
#           print("INTBM BEFORE - ", wleg.realLeg.leg.name," = ", self.motivationNetRobot.wrobot.mmcRobot.get_leg_vector(wleg.realLeg.leg.name) )
        for motiv_leg in self.motivationNetRobot.motivationNetLegs:
            motiv_leg.swing_net.resetToSavedSwingState()
            motiv_leg.stance_net.init_stance_footpoint = False
            #TODO reset of behindPEP - could be saved from original state, in this way is just turned off
            motiv_leg.behindPEP.last_activation_time = 0.
        for i in range(0, 6):
            self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(i)
        for wleg in self.motivationNetRobot.wrobot.wlegs:
            if (wleg.realLeg.predictedGroundContact() ):    
                self.motivationNetRobot.bodyModelStance.put_leg_on_ground(wleg.realLeg.leg.name, \
                    wleg.realLeg.leg.input_foot_position)
        self.motivationNetRobot.bodyModelStance.updateLegStates()
        self.motivationNetRobot.wrobot.mmcRobot.gc = [False, False, False, False, False, False]
        for wleg in self.motivationNetRobot.wrobot.wlegs:
#D          print("refresh on ground", wleg.realLeg.leg.name, " - ", wleg.realLeg.predictedGroundContact() )
            if (wleg.realLeg.predictedGroundContact()):
                self.motivationNetRobot.wrobot.mmcRobot.put_leg_on_ground(wleg.realLeg.leg.name, \
                    wleg.realLeg.leg.input_foot_position)
#       print("HR REAL POS: ", self.motivationNetRobot.wrobot.hind_right_leg.input_foot_position)
#D      for mleg in self.motivationNetRobot.motivationNetLegs:
#D          print(mleg.swing_motivation.output_value, mleg.wleg.realLeg.leg.input_foot_position)

#       for mmc_leg in self.motivationNetRobot.wrobot.mmcRobot.mmcLegs:
#           print("REAL LEG BEFORE - ", mmc_leg.wleg.leg.name, " = ", mmc_leg.wleg.leg.input_foot_position)
#           print("INTBM LEG BEFORE - ", mmc_leg.wleg.leg.name," = ", mmc_leg.getFootPosition() )
            
        for mmc_leg in self.motivationNetRobot.wrobot.mmcRobot.mmcLegs:
            #for i in range(0, 10):
            #   mmc_leg.add_sensory_joint_values([mmc_leg.wleg.leg.alpha.inputPosition, \
            #       mmc_leg.wleg.leg.beta.inputPosition, mmc_leg.wleg.leg.gamma.inputPosition])
            #   mmc_leg.mmc_kinematic_iteration_step()  
            mmc_leg.init_leg_mmc_model()
#       print(  self.motivationNetRobot.wrobot.mmcRobot.gc )
#       for wleg in self.motivationNetRobot.wrobot.wlegs:
#           print( (wleg.realLeg.predictedGroundContact()) )
#           print("REAL  AFTER  - ", wleg.realLeg.leg.name," = ", wleg.realLeg.leg.input_foot_position)
#           print("STANCE AFTER  - ", wleg.realLeg.leg.name," = ", self.motivationNetRobot.bodyModelStance.get_leg_vector(wleg.realLeg.leg.name) )
#           print("INTBM AFTER  - ", wleg.realLeg.leg.name," = ", self.motivationNetRobot.wrobot.mmcRobot.get_leg_vector(wleg.realLeg.leg.name) )
        
#       for mmc_leg in self.motivationNetRobot.wrobot.mmcRobot.mmcLegs:
#           print("REAL LEG AFTER  - ", mmc_leg.wleg.leg.name, " = ", mmc_leg.wleg.leg.input_foot_position)
#           print("INTBM LEG AFTER  - ", mmc_leg.wleg.leg.name," = ", mmc_leg.getFootPosition() )
        
#       print("**************************************************")

#        print("GC updated: ", self.motivationNetRobot.wrobot.mmcRobot.gc)
 #       for motiv_leg in self.motivationNetRobot.motivationNetLegs:
  #          print("Swing: ", motiv_leg.inSwingPhase())
   #     for itleg in self.motivationNetRobot.wrobot.wlegs:
    #        print(itleg.predictedGroundContact())

        #self.motivationNetRobot.wrobot.mmcRobot.updateLegStates()
        #print("GC updated: ", self.motivationNetRobot.wrobot.mmcRobot.gc)
        self.motivationNetRobot.wrobot.mmcRobot.mmc_iteration_step()
        #print("CURRENT MOTIV: ", self.motivationNetRobot.motivationNetLegs[2].swing_motivation.output_value, " - ",\
        #   self.motivationNetRobot.motivationNetLegs[2].stance_motivation.output_value, " ; ",\
        #   self.motivationNetRobot.motivationNetLegs[2].gc.output_value, " - ",\
        #   self.motivationNetRobot.motivationNetLegs[2].behindPEP.output_value)
        
        #executeNeuralNetStep()
        #print("CURRENT MOTIV: ", self.motivationNetRobot.motivationNetLegs[2].swing_motivation.output_value, " - ",\
        #   self.motivationNetRobot.motivationNetLegs[2].stance_motivation.output_value, " ; ",\
        #   self.motivationNetRobot.motivationNetLegs[2].gc.output_value, " - ",\
        #   self.motivationNetRobot.motivationNetLegs[2].behindPEP.output_value)

    def resetSimulationLoop(self, activation, fct_param):
#TODO: Reset of MU for swing and stance
        print("Problem Detected in Internal Simulation: ", activation, " - ", self.Phase.MU_NextRun.output_value, 
            ", HL: ", self.Phase.pD_MMC_unstableCheck_HL.output_value, self.Phase.pD_MMC_unstableHL_delay.output_value,
            ", HR: ", self.Phase.pD_MMC_unstableCheck_HR.output_value, self.Phase.pD_MMC_unstableHR_delay.output_value)
        self.resetWTA()
        self.Phase.MU_Sim.addIncomingValue(-2)
        self.Phase.MU_SAL.addIncomingValue(2)
        self.refreshInternalModelFromRobotData()
        
