'''
Created on 4.7.2013

@author: mschilling
'''

from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit, ModulatedParameterProviderUnit, propagateNeuralNetStep, evaluateNeuralNetStep, finalizeNeuralNetStep
from controller.reaCog.MotivationNetwork.PhasicUnit import PhasicUnit
from controller.reaCog.MotivationNetwork.DelayUnit import DelayUnit
from controller.reaCog.MotivationNetwork.ModulatingMotivationUnit import CoordinationRuleShiftModulatingMotivationUnit
from controller.reaCog.walknet.MotivationNetwork.ProblemDetectors import UnstableProblemDetector
from controller.reaCog.walknet.MotivationNetwork.ProblemDetectorStability import ProblemDetectorStability

#from controller.reaCog.CognitiveExpansion.DelayUnit import DelayUnit

from controller.reaCog.walknet.MotivationNetwork.MotivationNetLeg import MotivationNetLeg

#from controller.reaCog.Movements.BodymodelStance.mmcBodyModel3D import mmcBodyModelStance
from controller.reaCog.Movements.BodymodelStance.mmcBodyModel3D_CPP import mmcBodyModelStance
from controller.reaCog.Movements.StanceMovementSpringDamperMassModel import StanceMovementSpringDamperMassBody

from controller.reaCog.CognitiveExpansion.CognitiveExpansion import CognitiveExpansion

import Hector.RobotSettings as RSTATIC
import controller.reaCog.WalknetSettings as WSTATIC
from ..WDriveSafetyCheck import WDriveSafetyCheck as WDriveSafetyCheck

import copy, numpy

class MotivationNetRobot(object):

    def __init__(self, wrobot):
        self.wrobot = wrobot
        
        #self.summed_phases = 0.
        
        self.safe_stop_time = 50
        self.debug = False
        
        #self.wrobot.robot.driveSafetyCheck = WDriveSafetyCheck(self)
        
        if WSTATIC.stance_trajectory_generator=='bodymodel':
            # Stability factor: was -0.26
            self.bodyModelStance = mmcBodyModelStance(self, WSTATIC.stability_threshold)
        elif WSTATIC.stance_trajectory_generator=='springdampermass':
            #print(type(StanceMovementSpringDamperMassBody))
            l_zeros=[-WSTATIC.stanceheight for _ in range(6)]
            l_zeros[0]-=0.02
            l_zeros[1]-=0.02
            l_zeros[2]-=0.01
            l_zeros[3]-=0.01
            self.bodyModelStance = StanceMovementSpringDamperMassBody(self, spring_constants=100, masses=1, l_zeros=l_zeros)
        else:
            raise Exception('No valid stance trajectory generator was chose.')
        # Global motivation units
        self.stand = MotivationUnit("stand", group_name='cognitive_layer')
        self.stand.addConnectionTo(self.stand, 1)
        self.walk = MotivationUnit("walk", 1, group_name='cognitive_layer')
        self.walk.addConnectionTo(self.walk, 1)
        self.forward = MotivationUnit("fwd", 1, group_name='behavior_layer')
        self.forward.addConnectionTo(self.forward, 1)
        self.backward = MotivationUnit("bwd", group_name='behavior_layer')
        self.backward.addConnectionTo(self.backward, 1)
        
        self.stance_forward_velocity =  ModulatedParameterProviderUnit("fwd_vel", bias=1, group_name='cognitive_layer')
        self.stance_forward_velocity.set_modulated_external_param_provider(self.wrobot)
        
        temp_motivationNetLegs=[]
        for leg_name, wleg in zip(RSTATIC.leg_names, self.wrobot.wlegs):
            temp_motivationNetLeg = MotivationNetLeg(wleg, self)
            
            temp_motivationNetLeg.aep = copy.copy(getattr(WSTATIC, leg_name.split("_", 1)[0]+"_initial_aep"))
            temp_motivationNetLeg.pep = copy.copy(getattr(WSTATIC, leg_name.split("_", 1)[0]+"_initial_pep"))
            
            if "left" in leg_name:
                temp_motivationNetLeg.aep[1]*=-1
                temp_motivationNetLeg.pep[1]*=-1
                
            self.stand.addConnectionTo(temp_motivationNetLeg.stand, 1)
            self.walk.addConnectionTo(temp_motivationNetLeg.walk, 1)
            setattr(self, wleg.leg.name, temp_motivationNetLeg)
            temp_motivationNetLegs.append(temp_motivationNetLeg)
        self.motivationNetLegs=tuple(temp_motivationNetLegs)

#       self.front_left_leg.swing_net.modulatedRoutineFunctionCall( 1., self.front_left_leg.get_aep_shifted )
        
#        self.problemDetector = MotivationUnit("problem_detector", group_name='cognitive_layer')
#8      self.problemDetector.addConnectionTo(self.problemDetector, 1)
        
        self.pD_unstableCheck_HL = UnstableProblemDetector("pd_check_unstable_hl", self.bodyModelStance, self, 4, group_name='cognitive_layer')
        
#       MU_Beh = MotivationUnit("MU_Beh")
#MU_Beh.addConnectionTo( MU_Beh, 1 )
#fwd = MotivationUnit("fwd", bias=1)
#left_st = MotivationUnit("left_st")
#left_sw = MotivationUnit("left_sw", bias=1)
        
        # bias = -1 and connect to 
        self.safeStop_HL = PhasicUnit("safeStop_HL", time_window=WSTATIC.safe_stop_time, group_name='cognitive_layer')
        self.pD_unstableCheck_HL.addConnectionTo(self.safeStop_HL, 1)
        #MU_Beh.addConnectionTo(stop_left, 1)
        self.safeStop_HL.addConnectionTo(self.hind_left_leg.swing_motivation, -20)
        self.safeStop_HL.addConnectionTo(self.hind_left_leg.stance_motivation, 20)
        self.safeStop_HL.addConnectionTo(self.safeStop_HL, 1)
        
        self.safeStop_HL.addConnectionTo(self.stance_forward_velocity, -2)

        self.MU_StartPlan = DelayUnit("MU_DelayedStart", delay=WSTATIC.safe_stop_time-5, group_name='cognitive_layer')
        self.MU_StartPlan.addConnectionTo(self.MU_StartPlan, 1)
        self.safeStop_HL.addConnectionTo(self.MU_StartPlan, 1)
        #self.MU_StartPlan.addConnectionTo(self.stance_forward_velocity, -2)
        
#timed_cogn_start.addConnectionTo(stop_left, -2)
        
#8      self.pD_unstableCheck_HL.addConnectionTo(self.problemDetector, 1)
        # Used to make the robot actually fall backwards
        #self.pD_unstableCheck_HL.addConnectionTo(self.hind_left_leg.swing_motivation, 10)
        #self.pD_unstableCheck_HL.addConnectionTo(self.hind_left_leg.stance_motivation, -10)
        
        # When the real robot is getting instable, the robot should be slowed down!
#26     self.pD_unstableHL_delay = PhasicUnit("pD_unstableHL_delay", group_name='cognitive_layer', time_window = WSTATIC.unstable_slow_down_window)
#26     self.pD_unstableHL_delay.addConnectionTo(self.stance_forward_velocity, -1)
#26     self.pD_unstableCheck_HL.addConnectionTo(self.pD_unstableHL_delay, 1)
        # Enforcing that stance velocity is turned off for some time
#26     self.pD_unstableHL_delay.addConnectionTo(self.pD_unstableHL_delay, 1)
#26     self.pD_unstableHL_delay.addConnectionTo(self.hind_left_leg.stance_motivation, 10)
#26     self.pD_unstableHL_delay.addConnectionTo(self.hind_left_leg.swing_motivation, -10)
        
        self.pD_unstableCheck_HR = UnstableProblemDetector("pd_check_unstable_hr", self.bodyModelStance, self, 5, group_name='cognitive_layer')
        # New problem Detector
#NEW PD        self.pD_unstableCheck_HR = ProblemDetectorStability("pd_check_unstable_hr_new", self.bodyModelStance, self, 5, self.pD_unstableCheck_HR_old, group_name='cognitive_layer')
#NEW PD        self.pD_unstableCheck_FR = ProblemDetectorStability("pd_check_unstable_fr_new", self.bodyModelStance, self, 1, self.pD_unstableCheck_HR_old, group_name='cognitive_layer')

        
        self.safeStop_HR = PhasicUnit("safeStop_HR", time_window=WSTATIC.safe_stop_time, group_name='cognitive_layer')
        self.pD_unstableCheck_HR.addConnectionTo(self.safeStop_HR, 1)
        #MU_Beh.addConnectionTo(stop_left, 1)
        self.safeStop_HR.addConnectionTo(self.hind_right_leg.swing_motivation, -10)
        self.safeStop_HR.addConnectionTo(self.hind_right_leg.stance_motivation, 10)
        self.safeStop_HR.addConnectionTo(self.safeStop_HR, 1)
        
        self.safeStop_HR.addConnectionTo(self.stance_forward_velocity, -2)

        self.safeStop_HR.addConnectionTo(self.MU_StartPlan, 1)
        
        # Problem detector front left leg
        self.pD_unstableCheck_FL = UnstableProblemDetector("pd_check_unstable_fl", self.bodyModelStance, self, 0, group_name='cognitive_layer')
        self.safeStop_FL = PhasicUnit("safeStop_FL", time_window=WSTATIC.safe_stop_time, group_name='cognitive_layer')
        self.pD_unstableCheck_FL.addConnectionTo(self.safeStop_FL, 1)
        #MU_Beh.addConnectionTo(stop_left, 1)
        self.safeStop_FL.addConnectionTo(self.front_left_leg.swing_motivation, -10)
        self.safeStop_FL.addConnectionTo(self.front_left_leg.stance_motivation, 10)
        self.safeStop_FL.addConnectionTo(self.safeStop_FL, 1)
        self.safeStop_FL.addConnectionTo(self.stance_forward_velocity, -2)
        self.safeStop_FL.addConnectionTo(self.MU_StartPlan, 1)
        
        # Problem detector front right leg
        self.pD_unstableCheck_FR = UnstableProblemDetector("pd_check_unstable_fr", self.bodyModelStance, self, 1, group_name='cognitive_layer')
        self.safeStop_FR = PhasicUnit("safeStop_FR", time_window=15, group_name='cognitive_layer')
        self.pD_unstableCheck_FR.addConnectionTo(self.safeStop_FR, 1)
        #MU_Beh.addConnectionTo(stop_left, 1)
        self.safeStop_FR.addConnectionTo(self.front_right_leg.swing_motivation, -10)
        self.safeStop_FR.addConnectionTo(self.front_right_leg.stance_motivation, 10)
        self.safeStop_FR.addConnectionTo(self.safeStop_FR, 1)
        self.safeStop_FR.addConnectionTo(self.stance_forward_velocity, -2)
        self.safeStop_FR.addConnectionTo(self.MU_StartPlan, 1)
        
#8      self.pD_unstableCheck_HR.addConnectionTo(self.problemDetector, 1)
        # Used to make the robot actually fall backwards
        #self.pD_unstableCheck_HL.addConnectionTo(self.hind_left_leg.swing_motivation, 10)
        #self.pD_unstableCheck_HL.addConnectionTo(self.hind_left_leg.stance_motivation, -10)
        
        # When the real robot is getting instable, the robot should be slowed down!
#26     self.pD_unstableHR_delay = PhasicUnit("pD_unstableHR_delay", group_name='cognitive_layer', time_window = WSTATIC.unstable_slow_down_window)
#26     self.pD_unstableHR_delay.addConnectionTo(self.stance_forward_velocity, -1)
#26     self.pD_unstableCheck_HR.addConnectionTo(self.pD_unstableHR_delay, 1)
        # Enforcing that stance velocity is turned off for some time
#26     self.pD_unstableHR_delay.addConnectionTo(self.pD_unstableHR_delay, 1)
#26     self.pD_unstableHR_delay.addConnectionTo(self.hind_right_leg.stance_motivation, 10)
#26     self.pD_unstableHR_delay.addConnectionTo(self.hind_right_leg.swing_motivation, -10)
        
        self.initCoordinationRules()
        
        # Cognitive Expansion Ideas
#       self.idea_MU = []
        #TODO [1] is toBack
# Original idea MU
#        self.idea_MU = [self.middle_left_leg.swing_motivation_toFront, self.middle_left_leg.swing_motivation_toBack,
 #           self.hind_left_leg.swing_motivation_toFront, self.hind_left_leg.swing_motivation_toBack, 
  #          self.hind_right_leg.swing_motivation_toFront, self.hind_right_leg.swing_motivation_toBack, 
   #         self.middle_right_leg.swing_motivation_toFront, self.middle_right_leg.swing_motivation_toBack]
        self.idea_MU = [self.front_left_leg.swing_motivation_toFront, self.front_left_leg.swing_motivation_toBack,
            self.middle_left_leg.swing_motivation_toFront, self.middle_left_leg.swing_motivation_toBack,
            self.hind_left_leg.swing_motivation_toFront, self.hind_left_leg.swing_motivation_toBack, 
            self.hind_right_leg.swing_motivation_toFront, self.hind_right_leg.swing_motivation_toBack, 
            self.middle_right_leg.swing_motivation_toFront, self.middle_right_leg.swing_motivation_toBack,
            self.front_right_leg.swing_motivation_toFront, self.front_right_leg.swing_motivation_toBack]
#       for i in range(8):
#           self.idea_MU.append( MotivationUnit( "Idea_MU_" + str(i) ) )
#       self.idea_MU[2] = self.middle_left_leg.swing_motivation_toBack
#       self.idea_MU[3].output_value = 1.
#       self.idea_MU[3].addConnectionTo(self.idea_MU[3], 1)
        # TODO: put in all MUs connected to different behavior
#30     self.idea_MU[4] = self.middle_left_leg.swing_motivation_toBack
        
        #self.idea_MU[3].addConnectionTo(self.problemDetector, 1)
        
#       from controller.reaCog.MotivationNetwork.ModulatingMotivationUnit import CoordinationRuleShiftModulatingMotivationUnit
#       temp_motivation_unit = CoordinationRuleShiftModulatingMotivationUnit("rule1_hl_to_fl", # name
#               getattr(self.hind_left_leg, 'rule1').evaluateShift,
#               -0.2, # weight
#               self.front_left_leg.shiftPep,
#               startValue=0,
#               group_name=self.wrobot.hind_left_leg.leg.name + '_coordination_rules') 
        #temp_motivation_unit.addConnectionTo(temp_motivation_unit, 1)
#       self.idea_MU[0] = temp_motivation_unit
        
        self.cognitive_expansion = CognitiveExpansion(self, self.idea_MU)
        
        # Induce information into the spreading activation layer
        # = where did the problem occur?
        # Note: [index] has to point to stanceHL
        # TODO: Already in CognitiveExpansion? There safeStop mapped to SAL,
        # Should be only there with new PDs!
#        self.pD_unstableCheck_HL.addConnectionTo(self.cognitive_expansion.SAL_WTA_Net.layer_SAL[2], 1)
 #       self.pD_unstableCheck_HR.addConnectionTo(self.cognitive_expansion.SAL_WTA_Net.layer_SAL[4], 1)
        
        from controller.reaCog.CognitiveExpansion.SwitchRobot import SwitchRobot
        
        if (isinstance(self.wrobot, SwitchRobot)):
            #self.problemDetector.addConnectionTo(self.wrobot.switch, 1)
            self.MU_StartPlan.addConnectionTo(self.wrobot.switch, 1)
            #self.cognitive_expansion.Phase.unstableMMCCheckPD.addConnectionTo(self.wrobot.switch, 1)
            self.wrobot.mmcRobot.Phase = self.cognitive_expansion.Phase
            # The Problem Detector for the internal model
            
#       print("COMPUTE TRAJ")
#       import numpy
#       self.front_left_leg.swing_net.computeTrajectory(numpy.array([0.09085963, 0.28736604, -0.18]), 
#           numpy.array([0.11057309, 0.28998979, -0.08]), numpy.array([0.15, 0.27, -0.18]), 45)
        
    ## Update all the leg networks.
    def updateStanceBodyModel(self):
        if self.debug:
            mleg = self.motivationNetLegs[4]
            print("GC: ", mleg.wleg.realLeg.predictedGroundContact(), " - ", mleg.wleg.realLeg.leg.input_foot_position[2] )
            print("SWING: ", mleg.swing_motivation_toFront.output_value, mleg.swing_starter.output_value )
            print("STANCE: ", mleg.stance_motivation_toBack.output_value )
            print("Stop: ", self.safeStop_HL.output_value, self.safeStop_HL.current_count, self.pD_unstableCheck_HL.output_value)
            print("Start sim: ", self.MU_StartPlan.output_value)
            input()
    
        self.bodyModelStance.updateLegStates()
        
        #TODO: Remove this - will be handled by gc old_stance in body model
        # Right now only needed as during switching swing might turn on
        for i in range(0,6):
            if (self.motivationNetLegs[i].swing_motivation.output_value > 0.5):
                self.bodyModelStance.lift_leg_from_ground(i)
        
#       print("JOINTS: ", self.wrobot.front_left_leg.leg.alpha.inputPosition, self.wrobot.front_left_leg.leg.beta.inputPosition, self.wrobot.front_left_leg.leg.gamma.inputPosition, self.wrobot.front_left_leg.leg.computeForwardKinematics())
#       print("MMC:    ", self.bodyModelStance.get_robot_target_from_leg_vector('front_left_leg') )
            
        self.bodyModelStance.mmc_iteration_step()
        
#       print("AFTER:  ", self.bodyModelStance.get_robot_target_from_leg_vector('front_left_leg') )
        
        if (self.cognitive_expansion.Phase.MU_Beh.output_value > 0.) \
            or (self.cognitive_expansion.Phase.MU_Sim.output_value > 0.) \
            or (self.cognitive_expansion.Phase.MU_TestBeh.output_value > 0.):
            self.bodyModelStance.mmc_iteration_step()
        #self.bodyModelStance.check_static_stability()
        #self.bodyModelStance.check_static_stability_along_segment()
        
        #for leg_name, motiv_leg in zip(RSTATIC.ledeutlichg_names, self.motivationNetLegs):
        #   if getattr(RSTATIC,leg_name+"_enable"):
        #       motiv_leg.update_leg_mmc_model()
        for motiv_leg in self.motivationNetLegs:
            if (motiv_leg.wleg.leg.leg_enabled):
                motiv_leg.update_leg_mmc_model()

    def propagateAndEvaluateCoordinationRuleMotivationUnitsForLeg(self, leg_nr):
        leg_name=self.motivationNetLegs[leg_nr].wleg.leg.name
        propagateNeuralNetStep(leg_name+ '_coordination_rules')
        evaluateNeuralNetStep(leg_name+ '_coordination_rules')

    def finalizeCoordinationRuleMotivationUnitsForLeg(self, leg_nr):
        leg_name=self.motivationNetLegs[leg_nr].wleg.leg.name
        finalizeNeuralNetStep(leg_name+ '_coordination_rules')

    def initCoordinationRules(self):
        coordination_rules_forward=[]
        for rule_name in WSTATIC.coordination_rules.keys():
            if WSTATIC.coordination_rules[rule_name]["active"]:
                for sender_leg, sender_leg_nr in zip(self.motivationNetLegs, range(len(self.motivationNetLegs))):
                    for receiver_leg, receiver_leg_nr in zip(self.motivationNetLegs, range(len(self.motivationNetLegs))):
                        coordination_weight=WSTATIC.coordination_rules[rule_name]["coordination_weights"][sender_leg_nr][receiver_leg_nr]
                        if coordination_weight!=0:
                            if rule_name=='rule4':
                                mod_sender_function=getattr(sender_leg,rule_name).evaluateShift
                                mod_receiver_function=receiver_leg.shiftAep
                            else:
                                mod_sender_function=getattr(sender_leg,rule_name).evaluateShift
                                mod_receiver_function=receiver_leg.shiftPep
                                
                            temp_motivation_unit=CoordinationRuleShiftModulatingMotivationUnit(rule_name+"_"+sender_leg.wleg.leg.name+"_to_"+receiver_leg.wleg.leg.name, # name
                                                                                                mod_sender_function,
                                                                                                coordination_weight, # weight
                                                                                                mod_receiver_function,
                                                                                                startValue=1,
                                                                                                group_name=sender_leg.wleg.leg.name + '_coordination_rules') 
                            coordination_rules_forward.append(temp_motivation_unit)

        for cr in coordination_rules_forward:
            self.forward.addConnectionTo(cr, 1)

    def getPhaseRelationDeviation(self, leg_a, leg_b):
        return numpy.abs(0.5 - numpy.abs(self.motivationNetLegs[leg_a].getCurrentLegPhase() - 
            self.motivationNetLegs[leg_b].getCurrentLegPhase() ))

    def compareCurrentLegPhaseRelationsWithTripod(self):
        """
        Calculates deviation of antiphase of phase relations between neighboring legs.
        
        First: between three contralateral leg pairs
        Followed from top to bottom.
        
        Returns an numpy array with 7 phase relations.
        """
        phase_rel_deviations = numpy.array([self.getPhaseRelationDeviation(0,1),
            self.getPhaseRelationDeviation(2,3),
            self.getPhaseRelationDeviation(4,5),
            self.getPhaseRelationDeviation(0,2),
            self.getPhaseRelationDeviation(1,3),
            self.getPhaseRelationDeviation(2,4),
            self.getPhaseRelationDeviation(3,5)])
        #self.summed_phases += numpy.sum(phase_rel_deviations)/7.
        #print(self.summed_phases)        
        return phase_rel_deviations