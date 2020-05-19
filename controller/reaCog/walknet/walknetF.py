# -*- coding: utf-8 -*-

import numpy
from time import clock

import controller.reaCog.WalknetSettings as WSTATIC
import Hector.RobotSettings as RSTATIC

from ProcessOrganisation.ProcessModule.ControllerModule import ControllerModule
from ...PartialMovements.WaitForInput import WaitForInput as WaitForInput

from .MotivationNetwork.MotivationNetRobot import MotivationNetRobot
from ..MotivationNetwork.MotivationUnit import executeNeuralNetStep, propagateNeuralNetStep, evaluateNeuralNetStep, finalizeNeuralNetStep, getGroupNamesExceptFor

from tools.FreezableF import Freezable as Freezable
import stability
from collections import namedtuple

##
#   Main controller class of Walknet.
#   The controller is a processing module. The control is evaluated in the processing
#   time step (after sensory and input values are updated and before control signals
#   are send to the robot (done in the post processing step initiated from
#   this walknet object).
#   The walknet consists of a Motivation network for the robot and there are six
#   smaller networks for the six legs. In the motivation network it is decided
#   locally which movement shall be performed by the leg (depending on the current sensory
#   state and on local influence coordinating the behavior of the legs)
#   and the MotivationUnits representing behaviors are modulating the execution
#   of these movement primitives.
##
leg_attributes=namedtuple('leg_attributes',('leg_num', 'mleg', 'foot_position', 'distance_to_pep'))
class Walknet (ControllerModule, Freezable):

    ##
    #   Initialization of the main variables.
    #   @param wrobot walknet robot structure gives access to all robot variables
    #   and functions
    def __init__(self, name, wrobot=None):
        ControllerModule.__init__(self, name, wrobot.robot)
        self.wrobot = wrobot
        self.startTS = 0
        self.start = 0

        self.first_timestamp=None

        self.motivationNetRobot = MotivationNetRobot(self.wrobot)

        self.preprocessing_update_list=[]
        joint_attributes_to_be_updated=['inputPosition', 'torsion', 'outputPosition']
        for wleg in self.wrobot.wlegs:
            if wleg.leg.leg_enabled:
                for joint in wleg.leg.joints:
                    for attribute_name in joint_attributes_to_be_updated:
                        self.preprocessing_update_list.append( (joint, attribute_name) )

        self.speed_factor_stability=1
        self.speed_factor_workspace=1
        self.speed_factor_come_to_halt=1
        self.come_to_halt_time=2

        self.controller_come_to_end=False

        self.vocalization_method=lambda x: None

#        self.wrobot.robot.legs[0].lock()
 #       self.wrobot.robot.legs[1].lock()
  #      self.wrobot.robot.legs[3].lock()
   #     self.wrobot.robot.legs[4].lock()
    #    self.wrobot.robot.legs[5].lock()
        
        self.frozen=True

    def setVocalizationMethod(self, vocalization_method):
        self.vocalization_method=vocalization_method
    ##
    #   Setting of direction and speed.
    #   Sets the parameters in the corresponding movement primitives and activates
    #   the necessary MotivationUnits.
    #   @speed velocity of the movement (should be between 0.1(=wave gait)
    #       and 0.5 (=tripod gait))
    #   @direction relative direction of walking in degrees (up to 15 degrees, still
    #       leads to close curves as this direction is in every time step applied in the
    #       body model)
    def controlRobot(self, speed, direction):
        direc = direction# * ((numpy.pi)/180)
        if speed > 0 :
            self.motivationNetRobot.backward.output_value = 0.0
            self.motivationNetRobot.forward.output_value = 1.0
            self.motivationNetRobot.bodyModelStance.pullBodyModelAtFrontIntoRelativeDirection(-direc,speed)
            self.motivationNetRobot.bodyModelStance.pullBodyModelAtBackIntoRelativeDirection(0,0)
        elif speed < 0:
            self.motivationNetRobot.backward.output_value = 1.0
            self.motivationNetRobot.forward.output_value = 0.0
            self.motivationNetRobot.bodyModelStance.pullBodyModelAtBackIntoRelativeDirection(direc,-speed)
            self.motivationNetRobot.bodyModelStance.pullBodyModelAtFrontIntoRelativeDirection(0,0)
        else:
            self.motivationNetRobot.stand.output_value = 1.0
            self.motivationNetRobot.walk.output_value = 0.0
            self.motivationNetRobot.bodyModelStance.pullBodyModelAtFrontIntoRelativeDirection(0,0)
            self.motivationNetRobot.bodyModelStance.pullBodyModelAtBackIntoRelativeDirection(0,0)

    ##
    #   Initialization of the MMC leg models.
    def init_module(self):
        if __debug__:
            print("Init walknet")

        for motiv_leg in self.motivationNetRobot.motivationNetLegs:
            if motiv_leg.wleg.leg.leg_enabled:
                motiv_leg.init_leg_mmc_model()
            self.motivationNetRobot.bodyModelStance.put_leg_on_ground(motiv_leg.wleg.leg.name, \
                motiv_leg.wleg.leg.input_foot_position)
            print("LEG INIT: ", motiv_leg.wleg.leg.name, \
                motiv_leg.wleg.leg.input_foot_position)
        self.motivationNetRobot.bodyModelStance.updateLegStates()


    def handleMessage(self, message):
        if "controller_event" in message and message["controller_event"]=="come_to_end":
            self.controller_come_to_end=True

    ##
    #   Update of the motivation network.
    #   First the coordination rules are evaluated (depending on the sensory state)
    #   and then the time step of the neural network is processed.
    def updateMotivationNetworks(self):
        # Use these commands for serial evaluation of the motivation units, neurons etc. in order to define a certain evaluation order for the coordination rules of the legs
        # or for improving the stability by overwriting the results of the neural network in cases where a leg intends to start its swing phase
        # although it is needed for maintaining the static stability.
        if WSTATIC.prohibit_swing_if_unstable or WSTATIC.use_serial_evaluation_of_leg_controllers:
            nums_of_evaluated_legs=set() # A set containing the leg numbers of the already evaluated legs (this implies that the propagate-method has been executed as well!)
            nums_of_stancing_legs=set() # A set containing the leg numbers of all legs that are in the stance phase (or that just started their swing phase)
            if WSTATIC.use_serial_evaluation_of_leg_controllers: # If the serial evaluation is active...
                # first of all, execute one step of the neural network for all groups that do not directly belong to the coordination rule motivation units.
                executeNeuralNetStep(getGroupNamesExceptFor([leg.name+'_coordination_rules' for leg in self.motivationNetRobot.wrobot.robot.legs]))
                # After that, propagate and evaluate all neurons that belong to those legs that were in swing phase at the end of the last iteration (hoping that they hit the ground in the meantime).
                for leg_num, mleg in enumerate(self.motivationNetRobot.motivationNetLegs):
                    if mleg.isSwinging(): # If the leg is in swing phase, propagate and evaluate the corresponding motivation units. If the leg already established ground contact, it can be expected to support the body in the current iteration.
                        self.motivationNetRobot.propagateAndEvaluateCoordinationRuleMotivationUnitsForLeg(leg_num)
                        nums_of_evaluated_legs.add(leg_num) # add the leg to the set of already evaluated legs (so it won't be evaluated twice)
            else: # If the serial evaluation is not used...
                for leg_num, mleg in enumerate(self.motivationNetRobot.motivationNetLegs):
                    if not mleg.isSwinging():
                        nums_of_stancing_legs.add(leg_num)  # Add all legs that have been in stance phase at the end of the last iteration to the set of stancing legs.
                                                            # Therefore, the set might contain legs that change to the swing phase in the current iteration.
                                                            # This is done since the set is used later to decide which legs are allowed to change to swing phase
                                                            # and all legs in the set are still on the ground although they would like to swing now.
                propagateNeuralNetStep() # propagate...
                evaluateNeuralNetStep() # and evaluate the whole neural network
                nums_of_evaluated_legs=set(range(6)) # add all leg numbers to the list of evaluated legs

            for leg_num, mleg in enumerate(self.motivationNetRobot.motivationNetLegs):
                    if not mleg.isSwinging():
                        nums_of_stancing_legs.add(leg_num) # Add all legs that are in stance phase to the set of stancing legs.


            stancing_leg_list=[] # This list will contain a leg_attributes-entry for every leg that was added to the nums_of_stancing_legs set.
            for leg_num in nums_of_stancing_legs: # In this loop, the legs are added to the stancing_leg_list.
                mleg=self.motivationNetRobot.motivationNetLegs[leg_num]
                foot_position=self.motivationNetRobot.wrobot.robot.getInputFootPositionOfLeg(leg_num)
                distance_to_pep=mleg.wleg.leg.input_foot_position[0]-mleg.wleg.pep[0]
                stancing_leg_list.append(leg_attributes(leg_num=leg_num, mleg=mleg, foot_position=foot_position, distance_to_pep=distance_to_pep))

            # Now, the list is sorted for the distance of the foot position to the pep of the leg (legs that have crossed their pep come first,
            # followed by legs that have not yet arrived at their pep)
            stancing_leg_list.sort(key=lambda tup: tup.distance_to_pep, reverse=False) # Sort by the distance_to_pep

            for leg_attr in stancing_leg_list[:]: # This loops over all legs that are in stance phase (or have been until the current iteration)
                #leg_attr=stancing_leg_list[leg_index]
                if leg_attr.leg_num not in nums_of_evaluated_legs: # if the leg's motivation units were not yet evaluated,...
                    self.motivationNetRobot.propagateAndEvaluateCoordinationRuleMotivationUnitsForLeg(leg_attr.leg_num) # propagate and evaluate them
                    nums_of_evaluated_legs.add(leg_attr.leg_num) # and add the leg number to set of evaluated legs so it won't be evaluated twice accidentally

                if WSTATIC.prohibit_swing_if_unstable and leg_attr.mleg.isSwinging(): # If the stability should be improved and the leg started its swing phase during this very iteration,...
                                                                            # a test will be performed to find out whether it is save to lift the leg.
                    temp_foot_positions=[] # This list will contain the positions of all legs that are on the ground, except for the one whose desire-to-swing is tested for sanity.
                    for temp_leg_attr in stancing_leg_list:
                        if leg_attr.leg_num!=temp_leg_attr.leg_num:# and leg_attr.is_stancing:
                            temp_foot_positions.append((temp_leg_attr.foot_position[0],temp_leg_attr.foot_position[1],0))

                    convex_hull_points=stability.convexHull(list(temp_foot_positions)) # create a convex hull from the provided foot positions

                    # If the center of mass lies inside the support polygon and the stability can be maintained,...
                    if (stability.isPointInsideConvexHull(convex_hull_points, self.motivationNetRobot.wrobot.center_of_mass) and
                        stability.NESM(convex_hull_points, self.motivationNetRobot.wrobot.center_of_mass)>=WSTATIC.upper_stability_threshold):
                        stancing_leg_list.remove(leg_attr) # allow the leg to swing and therefore remove it from the list of stancing legs.
                    else: # if not, prohibit lift-off!
                        leg_attr.mleg.swing_motivation.setActivation(0)
                        leg_attr.mleg.stance_motivation.setActivation(1)

            finalizeNeuralNetStep() # At the very end, the last step of the neural net execution is called. This activates the swing/stance nets of the legs.
            

        else:
            # Use this command for parallel evaluation of the motivation units, neurons etc. without serial evaluation of the leg controllers nor stability improvement
            executeNeuralNetStep()

    def pre_processing_step(self, timestamp):
        for client, attribute_name in self.preprocessing_update_list:
            client.UpdateValueIfTooOld(attribute_name)


    ##
    #   MAIN CONTROL STEP
    #   Sets the current walking velocity and direction.
    #   The control is applied first evaluating the MotivationNetwork
    #   and then processing the Movement Primitives (update the stance body model)
    def processing_step(self, timeStamp):
        if self.controller_come_to_end and self.speed_factor_come_to_halt > 0:
            self.speed_factor_come_to_halt-=0.01
        if self.speed_factor_come_to_halt<=0:
            self._notifyOfCompletion()
        if WSTATIC.reduce_speed_for_reduced_stability:
            foot_positions=self.wrobot.robot.getInputFootPositions()
            convex_hull_points=stability.convexHull(foot_positions)
            if not stability.isPointInsideConvexHull(convex_hull_points, self.motivationNetRobot.wrobot.center_of_mass):
                self.speed_factor_stability=0
            else:
                current_stability=stability.NESM(convex_hull_points, self.motivationNetRobot.wrobot.center_of_mass)
                if current_stability <= WSTATIC.lower_stability_threshold:
                    self.speed_factor_stability=0
                elif WSTATIC.lower_stability_threshold < current_stability < WSTATIC.upper_stability_threshold:
                    temp_speed_factor_stability=(current_stability-WSTATIC.lower_stability_threshold)/(WSTATIC.upper_stability_threshold-WSTATIC.lower_stability_threshold)
                    if temp_speed_factor_stability<self.speed_factor_stability:
                        self.speed_factor_stability = temp_speed_factor_stability

                elif self.speed_factor_stability<1:
                    self.speed_factor_stability+=1/RSTATIC.controller_frequency/WSTATIC.acceleration_time_for_increased_stability
                    if self.speed_factor_stability>1:
                        self.speed_factor_stability=1
                else:
                    self.speed_factor_stability=1

        if WSTATIC.reduce_speed_if_close_to_workspace_border:
            min_distance=min([leg.getDistanceToWorkspaceBorder(-1) for leg in self.wrobot.robot.legs])
            temp_speed_factor_workspace=1
            if min_distance<0:
                self.speed_factor_workspace=0
            elif 0<=min_distance<WSTATIC.workspace_threshold:
                temp_speed_factor_workspace=min_distance/WSTATIC.workspace_threshold

            if temp_speed_factor_workspace > self.speed_factor_workspace:
                self.speed_factor_workspace+=1/RSTATIC.controller_frequency/WSTATIC.acceleration_time_for_increased_stability
                if temp_speed_factor_workspace<self.speed_factor_workspace:
                    self.speed_factor_workspace=temp_speed_factor_workspace
            else:
                self.speed_factor_workspace=temp_speed_factor_workspace
        # Set the walking direction and speed
        self.controlRobot(self.motivationNetRobot.stance_forward_velocity.get_modulated_external_param("desired_speed") \
            * self.speed_factor_stability*self.speed_factor_workspace*self.speed_factor_come_to_halt, self.wrobot.desired_direction_angle)
        if self.startTS == 0:
            self.start = clock()
            self.startTS = timeStamp
        if (timeStamp > 1.0):
            # Advance the Motivation Networks
            self.updateMotivationNetworks()

            # Update Body Model!
            self.motivationNetRobot.updateStanceBodyModel()

#           print("---- ", self.wrobot.front_left_leg.leg.alpha.inputPosition, self.wrobot.front_left_leg.leg.beta.inputPosition, self.wrobot.front_left_leg.leg.gamma.inputPosition)
#           print("---- ", self.motivationNetRobot.front_left_leg.inSwingPhase() , self.motivationNetRobot.front_left_leg.inStancePhase(), self.motivationNetRobot.front_left_leg.gc.output_value)
#           print("---- ", self.wrobot.middle_left_leg.controlVelocities)
            self.wrobot.sendAllAngleVelocity()
#           print("+++ ", self.wrobot.middle_left_leg.leg.alpha.desiredValue_ISC, self.wrobot.middle_left_leg.leg.beta.desiredValue_ISC, self.wrobot.middle_left_leg.leg.gamma.desiredValue_ISC)

    ##
    #   After calculation of the processing step.
    #   Sends the data to the simulator.
    def post_processing_step(self, timestamp):
        if (self.wrobot.switch.decoupled):
            activation_list = "MotivNet: \t"
            for mleg in self.motivationNetRobot.motivationNetLegs:
                activation_list += str(mleg.gc.output_value) + "\t" + str(2*mleg.stance_motivation.output_value) + "\t" + str(3*mleg.swing_motivation.output_value) + "\t \t"