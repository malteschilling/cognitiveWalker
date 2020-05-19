from .CognitivePhases import CognitivePhases
from .SpreadAndSelectNetwork import SpreadAndSelectNetwork
from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

from ..MotivationNetwork.MotivationUnit import executeNeuralNetStep

from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import getCurrentTime as getCurrentTime

##
# 	CognitiveExpansion
#
#	extends the reactive controller. It introduces different processing stages 
# 	(CognitivePhases) and adds three layers for selecting alternative behaviors 
#	(SpreadAndSelectNetwork).
#	The CognitiveExpansion object initializes these different parts and connects them
#	to the reactive controller.
#
#	For details on the controller structure see submitted publication 
#  	or for an older version: Schilling and Cruse, 2017 "ReaCog, a Minimal 
#	Cognitive Controller Based on Recruitment of Reactive Systems" in Frontiers in 
#	Neurorobotics.
##  
class CognitiveExpansion(object):

    def __init__(self, motiv_net, orig_MUs):
        self.motivationNetRobot = motiv_net

        self.Phase = CognitivePhases(self, self.learnCurrentWinner, self.resetSimulationLoop )

        self.SAL_WTA_Net = SpreadAndSelectNetwork(orig_MUs, self.Phase)
        
        # Induce information into the spreading activation layer
        # = where did the problem occur?     
        self.motivationNetRobot.safeStop_HL.addConnectionTo(self.SAL_WTA_Net.layer_SAL[4], 1)
        self.motivationNetRobot.safeStop_HR.addConnectionTo(self.SAL_WTA_Net.layer_SAL[6], 1)
        
        # Front leg spreading information
        self.motivationNetRobot.safeStop_FL.addConnectionTo(self.SAL_WTA_Net.layer_SAL[0], 1)
        self.motivationNetRobot.safeStop_FR.addConnectionTo(self.SAL_WTA_Net.layer_SAL[10], 1)

    def resetWTA(self):
        self.SAL_WTA_Net.resetWTA()
    
    def resetBehaviorSearch(self, activation, fct_param):
        self.SAL_WTA_Net.resetWTA()
        self.SAL_WTA_Net.resetSAL()
        self.SAL_WTA_Net.resetRTB()            
        print("Reset Cognitive Search for Behavior")

    def learnCurrentWinner(self, activation):
        self.SAL_WTA_Net.learnCurrentWinner(activation)
        self.resetWTA()
        TurnOff = MotivationUnit("TurnOFF_PD", bias=1, group_name='cognitive_expansion')
        TurnOff.addConnectionTo(TurnOff, 1)

    def refreshInternalModelFromRobotData(self):
        for motiv_leg in self.motivationNetRobot.motivationNetLegs:
            motiv_leg.swing_net.resetToSavedSwingState()
            motiv_leg.stance_net.init_stance_footpoint = False
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
            if (wleg.realLeg.predictedGroundContact()):
                self.motivationNetRobot.wrobot.mmcRobot.put_leg_on_ground(wleg.realLeg.leg.name, \
                    wleg.realLeg.leg.input_foot_position)
            
        for mmc_leg in self.motivationNetRobot.wrobot.mmcRobot.mmcLegs:
            mmc_leg.init_leg_mmc_model()

        self.motivationNetRobot.wrobot.mmcRobot.mmc_iteration_step()
        
    def resetSimulationLoop(self, activation, fct_param):
        print("Problem Detected in Internal Simulation: ", activation, " - ", self.Phase.MU_NextRun.output_value, 
            ", HL: ", self.Phase.pD_MMC_unstableCheck_HL.output_value, self.Phase.pD_MMC_unstableHL_delay.output_value,
            ", HR: ", self.Phase.pD_MMC_unstableCheck_HR.output_value, self.Phase.pD_MMC_unstableHR_delay.output_value)
        self.resetWTA()
        self.Phase.MU_Sim.addIncomingValue(-2)
        self.Phase.MU_SAL.addIncomingValue(2)
        self.refreshInternalModelFromRobotData()
        
