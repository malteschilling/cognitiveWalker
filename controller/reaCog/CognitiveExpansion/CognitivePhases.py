from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
from controller.reaCog.MotivationNetwork.PhasicUnit import PhasicUnit
from controller.reaCog.MotivationNetwork.ModulatingMotivationUnit import ModulatingMotivationUnit
from controller.reaCog.MotivationNetwork.ConnectionModulatorUnit import ConnectionModulatorUnit
from controller.reaCog.MotivationNetwork.DelayUnit import DelayUnit
from controller.reaCog.walknet.MotivationNetwork.ProblemDetectors import UnstableProblemDetector

from controller.reaCog.MotivationNetwork.WTAUnit import WTAUnitFast

import controller.reaCog.WalknetSettings as WSTATIC

##
# 	CognitivePhases
#
#	The cognitive expansion operates in different phases which each are represented as 
# 	a single MotivationUnit. These are instantiated and connected in CognitivePhases.
#	- normal behavior
#	- search for an alternative behavior (SAL)
#	- selection of a single alternative (WTA)
#	- internal simulation as such
#	- and testing of the selected behavior on the real system
#	The phases are represented as MotivationUnits and switching is triggered through
#	computation of the neural network
#
#	For details on the controller structure see submitted publication 
#  	or for an older version: Schilling and Cruse, 2017 "ReaCog, a Minimal 
#	Cognitive Controller Based on Recruitment of Reactive Systems" in Frontiers in 
#	Neurorobotics.
##  
class CognitivePhases(object):

    def __init__(self, cogn_exp, learn_fct, reset_fct):
        self.cognitive_expansion = cogn_exp
    
        # Different Phases Of the search - each has one MotivationUnit modulating the
        # specific actions during that MU_SAL
        # 0. Normal Behaviour
        self.MU_Beh = MotivationUnit("Phase_Beh", 1, group_name='cognitive_expansion')
        # 1. Spreading of Activation
        self.MU_SAL = ConnectionModulatorUnit("Phase_SAL", group_name='cognitive_expansion')
        # 2. Selecting a Winner of the activated units
        self.MU_WTA = MotivationUnit("Phase_WTA", group_name='cognitive_expansion')
        # 3. Mental Simulation
        self.MU_Sim = MotivationUnit("Phase_Sim", group_name='cognitive_expansion')
		# testing behaviour : ModulatingMotivationUnit("Phase_TestBeh", learn_fct , group_name='cognitive_expansion')
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

        self.cognitive_expansion.motivationNetRobot.MU_StartPlan.addConnectionTo(self.MU_Beh, -1)
        self.cognitive_expansion.motivationNetRobot.MU_StartPlan.addConnectionTo(self.MU_SAL, 1)
        
        self.ProblemDetectorDelay = MotivationUnit("PD_Delay_On", group_name='cognitive_expansion')
        self.cognitive_expansion.motivationNetRobot.MU_StartPlan.addConnectionTo(self.ProblemDetectorDelay, 1)

        self.ProblemDetectorDelay.addConnectionTo(self.MU_SAL, -1) # was -1
        self.ProblemDetectorDelay.addConnectionTo(self.MU_WTA, 1)
        
        self.MU_Beh.addConnectionTo(self.MU_WTA, -4)
        self.MU_TestBeh.addConnectionTo(self.MU_WTA, -4)

        self.MU_Sim.addConnectionTo(self.ProblemDetectorDelay, -1)
        self.MU_Sim.addConnectionTo(self.MU_SAL, -2)
        
        self.SimulationCounter = DelayUnit("SimulationCounter", group_name='cognitive_expansion', delay=WSTATIC.unstable_slow_down_window+300)
        self.MU_Sim.addConnectionTo(self.SimulationCounter, 1)
        self.SimulationCounter.addConnectionTo(self.MU_Sim, -2)
        self.SimulationCounter.addConnectionTo(self.MU_TestBeh, 0.75)

        # Going into test behavior phase
        self.MU_TestBeh.addConnectionTo(self.MU_SAL, -2)
        self.MU_TestBeh.changeConnectionWeight(self.MU_Sim, -1)
        
        self.MU_Beh.addConnectionTo(self.MU_Sim, -1)
        
        self.MU_TestBeh.addConnectionTo(self.cognitive_expansion.motivationNetRobot.MU_StartPlan, -1)
        
        self.TestBehaviourCounter = DelayUnit("TestBehaviourCounter", group_name='cognitive_expansion', delay=WSTATIC.unstable_slow_down_window)
        self.MU_TestBeh.addConnectionTo(self.TestBehaviourCounter, 1)
        self.TestBehaviourCounter.addConnectionTo(self.MU_TestBeh, -2)
        self.TestBehaviourCounter.addConnectionTo(self.MU_Beh, 2)
        
        # Reset the behavioral search network:
        self.ResetBehavioralSearch = ModulatingMotivationUnit("Reset_Beh_Search", self.cognitive_expansion.resetBehaviorSearch, group_name='cognitive_expansion')
        self.TestBehaviourCounter.addConnectionTo(self.ResetBehavioralSearch, 1)
        
        self.old_phase = 0
        
        self.MU_NextRun = ModulatingMotivationUnit("Reset_Phases", reset_fct, bias=-2, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_HL = UnstableProblemDetector("pd_mmc_check_unstable_hl", self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot, self.cognitive_expansion.motivationNetRobot, 4, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_HL.addConnectionTo(self.MU_NextRun, 1)
        
        self.pD_MMC_unstableCheck_HR = UnstableProblemDetector("pd_mmc_check_unstable_hr", self.cognitive_expansion.motivationNetRobot.wrobot.mmcRobot, self.cognitive_expansion.motivationNetRobot, 5, group_name='cognitive_expansion')
        self.pD_MMC_unstableCheck_HR.addConnectionTo(self.MU_NextRun, 1)
        
        self.MU_Sim.addConnectionTo(self.MU_NextRun, 2)
        
        # When the internal simulation is getting instable, 
        # the just starting (and causing) swing movement should be suppressed for a short time
        self.pD_MMC_unstableHL_delay = PhasicUnit("pD_MMC_unstableHL_delay", group_name='cognitive_layer', bias=-1, time_window =  10)
        self.pD_MMC_unstableCheck_HL.addConnectionTo(self.pD_MMC_unstableHL_delay, 1)
        
        self.pD_MMC_unstableHR_delay = PhasicUnit("pD_MMC_unstableHR_delay", group_name='cognitive_layer', bias=-1, time_window =  10)
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
                
        # During the TestBeh the walking also has to be slowed down
        self.MU_Sim_delay = PhasicUnit("MU_Sim_delay", group_name='cognitive_layer', time_window =  WSTATIC.unstable_slow_down_window)
        self.MU_Sim.addConnectionTo(self.MU_Sim_delay, 1)
        self.MU_Sim_delay.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        self.MU_TestBeh.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        
        self.MU_SAL.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        self.MU_WTA.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
        self.transitionInhibitor = MotivationUnit("transInhibition")
        self.MU_WTA.addConnectionTo(self.transitionInhibitor, 1)
        self.transitionInhibitor.addConnectionTo(self.cognitive_expansion.motivationNetRobot.stance_forward_velocity, -2)
             
        self.it = 0

    def printCurrentPhaseValues(self):
        print("Beh: ", self.MU_Beh.output_value, " - SAL: ", self.MU_SAL.output_value, " - WTA: ", 
            self.MU_WTA.output_value, " - Sim: ", self.MU_Sim.output_value, " - TestBeh: ", 
            self.MU_TestBeh.output_value)
    
    def getCurrentPhase(self):
        max_act=0
        max_phase=0
        for i in range(len(self.Phase_MUs)):
            if (self.Phase_MUs[i].output_value > max_act):
                max_phase = i
            
        self.it +=1 
        
        if not(max_phase == self.old_phase):
            print("Switched to Phase ", max_phase, " = ", self.Phase_MUs[max_phase].name, " : ", self.pD_MMC_unstableCheck_HL.iteration) 
            self.printCurrentPhaseValues()
            
            if (self.MU_Sim.output_value > 0.1 or self.MU_TestBeh.output_value > 0.1):
                current_idea = 0
                for i in range(0, len(self.cognitive_expansion.motivationNetRobot.cognitive_expansion.SAL_WTA_Net.layer_SAL) ):
                    if (self.cognitive_expansion.SAL_WTA_Net.layer_WTA[i].output_value > 0.5):
                        current_idea = i
                print("========================================================")
                print("Selected idea as a new behavior: ", current_idea, " = ", 
                    self.cognitive_expansion.SAL_WTA_Net.original_MUs[current_idea].name)
                print("========================================================")
        self.old_phase = max_phase
        return max_phase
