from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
from controller.reaCog.MotivationNetwork.PhasicUnit import PhasicUnit
from controller.reaCog.MotivationNetwork.ModulatingMotivationUnit import ModulatingMotivationUnit

from controller.reaCog.walknet.MotivationNetwork.SensorUnit import BehindPEPSensorUnit
from controller.reaCog.walknet.MotivationNetwork.CollisionDetector import CollisionDetector

from controller.reaCog.Movements.SwingMovementDirectInverseKinematics import SwingMovementDirectInverseKinematics
from controller.reaCog.Movements.SwingMovementBezier import SwingMovementBezier
from controller.reaCog.Movements.StanceMovementBodyModel import StanceMovementBodyModel
from controller.reaCog.Movements.StanceMovementSpringDamperMassModel import StanceMovementSpringDamperMassLeg
import controller.reaCog.WalknetSettings as WSTATIC
import Hector.RobotSettings as RSTATIC
import controller.reaCog.walknet.MotivationNetwork.MotivationNetworkSettings as MNSTATIC
from . import CoordinationRules as CoordinationRules

import numpy
import math

#from controller.reaCog.Movements.BodymodelStance.mmcAngularLegModelRobot import mmcAngularLegModelStance
from controller.reaCog.Movements.BodymodelStance.mmcAngularLegModelRobot_CPP import mmcAngularLegModelStance

class MotivationNetLeg(object):

    def __init__(self, wleg, motivationNetRobot):
        
#       self.counter = 0
    
        self.wleg = wleg
        self.motivationNetRobot = motivationNetRobot
        
        self.aep = numpy.array([0.0,0.0,0.0], dtype=float)
        self.aep_init = numpy.array(self.aep)
        self.aep_temp = None #numpy.array([0.0,0.0,0.0], dtype=float)
        # For one experiment the swing movements are pushed further to the front.
        # Usually, next_swing_mode is 0
        # next_swing_mode in [1,9] counts when the leg is in a stable stance_motivation
        # and afterwards moves the swing target to the temporary target
        # During the next swing movement the next_swing_mode is further increased
        # and in the next stance movement the old swing target is reset.
        self.next_swing_mode = 0
        
        self.pep = numpy.array([0.0,0.0,0.0], dtype=float)
        self.dep = numpy.array([0.0,0.0,WSTATIC.depz], dtype=float)

        self._pep_shift         = numpy.array([0.0,0.0,0.0], dtype=float)
        self._pep_shift_is_up_to_date=True
        self._pep_shifts=dict()

        self._aep_shift         = numpy.array([0.0,0.0,0.0], dtype=float)
        self._aep_shift_is_up_to_date=True
        self._aep_shifts=dict()
        
        self.mmcLegStance = mmcAngularLegModelStance(self)
        
        leg_name = self.wleg.leg.name[0] + self.wleg.leg.name[1+str.find(self.wleg.leg.name, '_')]
        
        self.legMU = MotivationUnit("leg_" + leg_name,1, group_name='cognitive_layer')
        self.legMU.addConnectionTo(self.legMU, 1)
        
        self.stand = MotivationUnit("stand_" + leg_name, group_name='cognitive_layer')
        self.walk = MotivationUnit("walk_" + leg_name, group_name='cognitive_layer')
        
        self.forward = MotivationUnit("forw_" + leg_name, bias=1, group_name='cognitive_layer')
        self.backward = MotivationUnit("backw_" + leg_name, group_name='cognitive_layer')
        
        #if WSTATIC.swing_trajectory_generator=='bezier':
        #   self.swing_net = SwingMovementBezier(self)
        #   self.swing_motivation = ModulatingMotivationUnit ( ("swing_" + leg_name), self.swing_net.moveToNextPoint, threshold=0., group_name=self.wleg.leg.name+ '_coordination_rules')
        if WSTATIC.swing_trajectory_generator=='quadratic_spline':
            self.swing_net = SwingMovementDirectInverseKinematics(self)
            self.swing_motivation = MotivationUnit(("swing_" + leg_name), group_name='behavior_layer')
            self.swing_motivation_toFront = ModulatingMotivationUnit ( ("swing_toFront_" + leg_name), self.swing_net.modulatedRoutineFunctionCall, fct_param =self.get_aep_shifted, threshold=0.5, group_name='behavior_layer')
            self.swing_motivation_toBack = ModulatingMotivationUnit ( ("swing_toBack_" + leg_name), self.swing_net.modulatedRoutineFunctionCall, fct_param = self.get_swing_backward_target, threshold=0.5, group_name='behavior_layer')    
            #self.swing_motivation = ModulatingMotivationUnit ( ("swing_" + leg_name), self.swing_net.modulatedRoutineFunctionCall, fct_param = self.get_aep_shifted, threshold=0.5, group_name='_coordination_rules')
            #ModulatingMotivationUnit ( ("swing_" + leg_name), self.swing_net.modulatedRoutineFunctionCall, threshold=0., group_name=self.wleg.leg.name+ '_coordination_rules')
        else:
            raise Exception('No valid swing trajectory generator was chosen.')
                
        if WSTATIC.stance_trajectory_generator=='bodymodel':
            self.stance_net = StanceMovementBodyModel(self)
        #elif WSTATIC.stance_trajectory_generator=='springdampermass':
        #   self.stance_net = StanceMovementSpringDamperMassLeg(self)
        else:
            raise Exception('No valid stance trajectory generator was chosen.')
        
        self.stance_motivation = MotivationUnit(("stance_" + leg_name), startValue=1, group_name='behavior_layer')
        self.stance_motivation_toBack = ModulatingMotivationUnit( ("stance_toBack_" + leg_name), self.stance_net.modulatedRoutineFunctionCall, startValue=1, threshold=0.5, group_name='behavior_layer')
        self.stance_motivation_toFront = ModulatingMotivationUnit( ("stance_toFront_" + leg_name), self.stance_net.modulatedRoutineFunctionCall, startValue=1, threshold=0.5, group_name='behavior_layer')
        
        #self.stance_motivation = ModulatingMotivationUnit( ("stance_" + leg_name), self.stance_net.modulatedRoutineFunctionCall, startValue=1, threshold=0.1, group_name=self.wleg.leg.name+ '_coordination_rules')
        
        rec_w = 0.6
        
        self.swing_motivation.addConnectionTo(self.swing_motivation, rec_w)
        self.stance_motivation.addConnectionTo(self.swing_motivation, -0.75)
        self.legMU.addConnectionTo(self.swing_motivation, 0.25)
        self.swing_motivation_toFront.addConnectionTo(self.swing_motivation, 0.8)
        self.swing_motivation_toBack.addConnectionTo(self.swing_motivation, 0.8)
        
        self.swing_motivation_toFront.addConnectionTo(self.swing_motivation_toFront, rec_w)
        self.swing_motivation.addConnectionTo(self.swing_motivation_toFront, 0.45)
        self.swing_motivation_toBack.addConnectionTo(self.swing_motivation_toFront, -0.6)
        self.backward.addConnectionTo(self.swing_motivation_toFront, -0.15)
        
        self.swing_motivation_toBack.addConnectionTo(self.swing_motivation_toBack, rec_w)
        self.swing_motivation.addConnectionTo(self.swing_motivation_toBack, 0.45)
        self.swing_motivation_toFront.addConnectionTo(self.swing_motivation_toBack, -0.6)
        self.forward.addConnectionTo(self.swing_motivation_toBack, -0.15)
            
        self.stance_motivation.addConnectionTo(self.stance_motivation, 0.25)
        self.swing_motivation.addConnectionTo(self.stance_motivation, -0.75)
        self.legMU.addConnectionTo(self.stance_motivation, 0.25)
        self.stance_motivation_toFront.addConnectionTo(self.stance_motivation, 0.8)
        self.stance_motivation_toBack.addConnectionTo(self.stance_motivation, 0.8)

        self.stance_motivation_toFront.addConnectionTo(self.stance_motivation_toFront, 0.25)
        self.stance_motivation.addConnectionTo(self.stance_motivation_toFront, 0.45)
        self.stance_motivation_toBack.addConnectionTo(self.stance_motivation_toFront, -0.6)
        self.forward.addConnectionTo(self.stance_motivation_toFront, -0.15)
            
        self.stance_motivation_toBack.addConnectionTo(self.stance_motivation_toBack, rec_w)
        self.stance_motivation.addConnectionTo(self.stance_motivation_toBack, 0.45)
        self.stance_motivation_toFront.addConnectionTo(self.stance_motivation_toBack, -0.6)
        self.backward.addConnectionTo(self.stance_motivation_toBack, -0.15)
        
#           self.stance_motivation.addConnectionTo(self.stance_motivation, WSTATIC.ws/(WSTATIC.ws + 1))
#           self.swing_motivation.addConnectionTo(self.stance_motivation, -3/(WSTATIC.ws + 1))
#           self.legMU.addConnectionTo(self.stance_motivation, WSTATIC.ws/(WSTATIC.ws + 1))
            #self.backward.addConnectionTo(self.stance_motivation, -3/(WSTATIC.ws + 1))
        
#           self.swing_motivation.addConnectionTo(self.swing_motivation, WSTATIC.ws/(WSTATIC.ws)) # ws/ws+1
#           self.stance_motivation.addConnectionTo(self.swing_motivation, -3/(WSTATIC.ws + 1))
#           self.legMU.addConnectionTo(self.swing_motivation, 1/(WSTATIC.ws + 1))
            #self.forward.addConnectionTo(self.swing_motivation, -3/(WSTATIC.ws + 1))
        
        rule1_coordination_weights=WSTATIC.coordination_rules["rule1"]
        self.rule1=CoordinationRules.Rule1(     self,
                                                rule1_coordination_weights["swing_stance_pep_shift_ratio"],
                                                rule1_coordination_weights["velocity_threshold"],
                                                rule1_coordination_weights["velocity_threshold_delay_offset"],
                                                rule1_coordination_weights["lower_velocity_delay_factor"],
                                                rule1_coordination_weights["higher_velocity_delay_factor"],
                                                rule1_coordination_weights["max_delay"])
    
        rule2_coordination_weights=WSTATIC.coordination_rules["rule2"]
        self.rule2= CoordinationRules.Rule2(    self,
                                                rule2_coordination_weights["start_delay"],
                                                rule2_coordination_weights["duration"])
    
    
        rule3LinearThreshold_coordination_weights=WSTATIC.coordination_rules["rule3LinearThreshold"]
        self.rule3LinearThreshold= CoordinationRules.Rule3LinearThreshold(self,
                                                rule3LinearThreshold_coordination_weights["active_distance"],
                                                rule3LinearThreshold_coordination_weights["threshold_offset"],
                                                rule3LinearThreshold_coordination_weights["threshold_slope"])

        rule3Ipsilateral_coordination_weights=WSTATIC.coordination_rules["rule3Ipsilateral"]
        self.rule3Ipsilateral= CoordinationRules.Rule3Ipsilateral(self,
                                                rule3Ipsilateral_coordination_weights["active_distance"],
                                                rule3Ipsilateral_coordination_weights["threshold_offset"],
                                                rule3Ipsilateral_coordination_weights["threshold_slope"],
                                                rule3Ipsilateral_coordination_weights["threshold_2_offset"],
                                                rule3Ipsilateral_coordination_weights["threshold_2_slope"])
    
        #rule3SigmoidThreshold_coordination_weights=WSTATIC.coordination_rules["rule3SigmoidThreshold"]
        #self.rule3SigmoidThreshold= CoordinationRules.Rule3SigmoidThreshold(self,
        #                                       rule3SigmoidThreshold_coordination_weights["active_distance"],
        #                                       rule3SigmoidThreshold_coordination_weights["threshold_turning_point"],
        #                                       rule3SigmoidThreshold_coordination_weights["threshold_slope"])

        self.rule4= CoordinationRules.Rule4(    self)
        
        self.behindPEP = BehindPEPSensorUnit(self, group_name='sensors')
        # Inhibition of stance not needed - due to swing starter inhibition.
        self.behindPEP.addConnectionTo(self.stance_motivation, -9*WSTATIC.ws/(WSTATIC.ws + 1))
        self.behindPEP.addConnectionTo(self.swing_motivation, 9*WSTATIC.ws/(WSTATIC.ws + 1)) # was 9
        
        # ToDo: threshold for start of swing quite small - should be improved (e.g. longer 
        # activation from above)
        self.swing_starter = PhasicUnit( ("sw_starter_" + leg_name), group_name='sensors', time_window=(RSTATIC.controller_frequency*WSTATIC.minimum_swing_time), threshold=0.25 )
        self.swing_motivation.addConnectionTo(self.swing_starter, 1)
        self.swing_starter.addConnectionTo(self.swing_motivation, 9*WSTATIC.ws/(WSTATIC.ws + 1) )
        self.swing_starter.addConnectionTo(self.stance_motivation, -9*WSTATIC.ws/(WSTATIC.ws + 1) )

        self.gc = MNSTATIC.ground_contact_class_dict[MNSTATIC.groundContactMethod](self, group_name='sensors')
        self.gc.addConnectionTo(self.stance_motivation, 9/(WSTATIC.ws + 1)) # was 3
        # ToDo: GC should inhibit swing in the future again - but there should be some PhasicUnit
        # behavior = only in beginning.
        #self.gc.addConnectionTo(self.swing_motivation, -9/(WSTATIC.ws + 1))
        
#       print(self.wleg.leg.name, " SW: ", self.swing_motivation.output_value, " - ST: ", self.stance_motivation.output_value,
#           " - GC: ", self.gc.output_value, " - behPEP: ", self.behindPEP.output_value, " Starter: ", self.swing_starter.output_value)

#       self.collision_detector = CollisionDetector(self, group_name='sensors')
#       self.collision_detector.addConnectionTo(self.swing_net.notifyOfCollision)
        
    def __del__(self):
        pass

    def init_leg_mmc_model(self):
        """ As the MMC networks have to be initialised this function has to be called in
            the beginning to drive the network into a valid configuration. This is
            important for the Dual Quaternion network, but for the single leg network
            this might be dropped as there the relaxation is very fast.
        """
#       self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(self.wleg.leg.name)
        for _ in range(0,10) :
            self.mmcLegStance.set_joint_angles( self.wleg.leg.getInputPosition() )
            self.mmcLegStance.mmc_kinematic_iteration_step()
            
        if __debug__:
            print(self.wleg.leg.name, " in init mmc")
            
#       self.motivationNetRobot.bodyModelStance.
            
    def update_leg_mmc_model(self):
        """ Update the leg network.
            Sensory data is given back to the leg network and it is advanced one step.
            This is sufficient to update the leg network. At the same time
            the network acts as a filter: when applying noise (can be done in the
            mmc leg class on all sensory data) the incoming sensory data
            is fused with the current configuration into a coherent state.
        """
        pass
#       self.counter += 1
#       if (self.wleg.leg.name == 'front_right_leg'):
#           if (10 < self.counter < 13):
#               print("#### ADD INPUT ####")
#               self.swing_motivation_toFront.addIncomingValue(1)
#           print(self.wleg.leg.name, " SW: ", self.swing_motivation.output_value, " - ST: ", self.stance_motivation.output_value,
#               " - GC: ", self.gc.output_value, " - behPEP: ", self.behindPEP.output_value, " Starter: ", self.swing_starter.output_value)
#       if (self.wleg.leg.name == 'front_right_leg'):
#           print("Sw: %6.2f , St: %6.2f " % (self.swing_motivation.output_value, self.stance_motivation.output_value) )
# Add sensor integration
#       if (self.kin_calc == 2):
#           pass
#DEB            self.mmcLegStance.add_sensory_joint_values([self.alpha.getTargetAngle(),
#DEB                self.beta.getTargetAngle(), self.gamma.getTargetAngle()])
            #print("Current angles: " + str([self.alpha.targetAngle,
            #   self.beta.targetAngle, self.gamma.targetAngle]))
#DEB            self.mmcLegStance.mmc_kinematic_iteration_step()
            #print(self.mmcLeg.get_joint_angles())
            
#=========== Shifting the PEP Methods ====================      
    @property
    def pep_shifted(self):
        tempPep = self.pep+self.pep_shift
        if tempPep[0]>=self.aep_shifted[0]-WSTATIC.minimum_swing_distance:
            tempPep[0]=self.aep_shifted[0]-WSTATIC.minimum_swing_distance
        return tempPep

    @property
    def pep_shift(self):
        if not self._pep_shift_is_up_to_date:
            self._pep_shift=sum(self._pep_shifts.values())
            self._pep_shift_is_up_to_date=True
        return self._pep_shift

    def shiftPep(self, source_name, pep_shift):
        try:
            temp_pep_shift=numpy.array((float(pep_shift), 0, 0))
        except TypeError:
            if len(pep_shift)==3:
                temp_pep_shift=numpy.array((float(i) for i in pep_shift))
        if temp_pep_shift.any():
            self._pep_shifts[source_name]=temp_pep_shift
        else:
            try:
                del self._pep_shifts[source_name]
            except Exception:
                    pass
        self._pep_shift_is_up_to_date=False

    def resetPepShift(self):
        self._pep_shifts=dict()
        
    def getPepDict(self):
        return self._pep_shifts

    def get_aep_shifted(self):
        return self.aep_shifted
        
    def get_pep_shifted(self):
        return self.pep_shifted
        
    def get_swing_backward_target(self):
        if (self.wleg.leg.name[0] == 'h'):
            return (self.pep + numpy.array([0.05, 0., 0.]))
        else:
            return (self.pep + numpy.array([0.05, 0., 0.]))

    @property
    def aep_shifted(self):
        return self.aep+self.aep_shift

    @property
    def aep_shift(self):
        if not self._aep_shift_is_up_to_date:
            self._aep_shift=sum(self._aep_shifts.values())
            self._aep_shift_is_up_to_date=True
        return self._aep_shift

    def shiftAep(self, source_name, aep_shift):
        try:
            temp_aep_shift=numpy.array((float(aep_shift), 0, 0))
        except TypeError:
            if len(aep_shift)==3:
                temp_aep_shift=numpy.array((float(i) for i in aep_shift))
        if temp_aep_shift.any():
            self._aep_shifts[source_name]=temp_aep_shift
        else:
            try:
                del self._aep_shifts[source_name]
            except Exception:
                    pass
        self._aep_shift_is_up_to_date=False

    def resetAepShift(self):
        self._aep_shifts=dict()
        
    def moveNextSwingToFront(self, targetShift):
        self.next_swing_mode = 1
        self.aep_init = numpy.array(self.aep)
        self.aep_temp = numpy.array( self.aep_init + targetShift )

    ##  Function which answers if the motivation unit for swing is active.
    def inSwingPhase(self):
#       if (self.wleg.leg.name == "middle_left_leg"):
#           sw_text = "InSwing Test - "
#           sw_text = sw_text + "SW_F: " + '{:.4}'.format(float(self.swing_motivation_toFront.output_value)) + "\t - "
#           sw_text = sw_text + "SW_B: " + '{:.4}'.format(float(self.swing_motivation_toBack.output_value)) + "\t - "
#           sw_text = sw_text + "ST_F: " + '{:.4}'.format(float(self.stance_motivation_toFront.output_value)) + "\t - "
#           sw_text = sw_text + "ST_B: " + '{:.4}'.format(float(self.stance_motivation_toBack.output_value)) + "\t - "
#           sw_text = sw_text + "GC  : " + '{:.2}'.format(float(self.gc.output_value)) + "\t - "
#           sw_text = sw_text + "UnstableDelay : " + '{:.4}'.format(float(self.motivationNetRobot.pD_unstableHL_delay.output_value)) + "\t - "
#           sw_text = sw_text + "UnstableDelay : " + '{:.4}'.format(float(self.motivationNetRobot.cognitive_expansion.Phase.pD_MMC_unstableHL_delay.output_value))
#           sw_text = sw_text + "TestDelay : " + '{:.4}'.format(float(self.motivationNetRobot.cognitive_expansion.Phase.testBehaviour_delay.output_value))
#           print(sw_text)
        # TODO: Should be moved to stance_unit as modulation or somewhere else
        swing_test = ((self.swing_motivation_toFront.output_value > self.swing_motivation_toFront.threshold)
            or (self.swing_motivation_toBack.output_value > self.swing_motivation_toBack.threshold))
        if swing_test:
            self.stance_net.resetStanceTrajectory()
            if (10 <= self.next_swing_mode):
                self.next_swing_mode += 1
        else:
            self.swing_net.resetSwingTrajectory()
            if (0 < self.next_swing_mode < 9):
                self.next_swing_mode += 1
            if (self.next_swing_mode == 9):
                self.aep = self.aep_temp
                print("SET new Swing Target for ", self.wleg.leg.name)
                self.next_swing_mode += 1
            if (self.next_swing_mode > 25):
                self.aep = self.aep_init
                print("RESET Swing Target for ", self.wleg.leg.name)
                self.next_swing_mode = 0
        return swing_test

    ##  Function which answers if the motivation unit for stance is active.
    def inStancePhase(self):
        return ((self.stance_motivation_toBack.output_value > self.stance_motivation_toBack.threshold) or
            (self.stance_motivation_toFront.output_value > self.stance_motivation_toFront.threshold))

    def isSwinging(self):
        return 0 < self.swing_motivation.output_value > self.stance_motivation.output_value

    def isStancing(self):
        return not self.isSwinging()

    # Function provides relative phase information:
    # 0 is beginning of stance, 0.5 beginning of swing, 1 end of swing
    def getCurrentLegPhase(self):
        current_phase = 0.
        if WSTATIC.swing_trajectory_generator=='quadratic_spline':
            if (self.isSwinging()):
                current_phase = 0.5 + 0.5 * self.swing_net.iteration / self.swing_net.iteration_steps_swing
                if current_phase > 1.:
                    current_phase = 1.
            else:
                current_phase = 0.5 * (self.aep[0] - self.wleg.leg.input_foot_position[0])/(self.aep[0] - self.pep_shifted[0])
                if current_phase > 0.5:
                    current_phase = 0.5
                elif current_phase < 0.:
                    current_phase=0.
            #print(self.wleg.leg.name + " in " + str(current_phase))
        return current_phase
        