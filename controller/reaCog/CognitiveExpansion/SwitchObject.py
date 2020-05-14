from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit
from controller.reaCog.MotivationNetwork.MotivationUnit import executeNeuralNetStep
from controller.reaCog.MotivationNetwork.MotivationUnit import finalizeNeuralNetStep
from decimal import Decimal as Decimal

import controller.reaCog.WalknetSettings as WSTATIC

import numpy

class SwitchObject (MotivationUnit):

    def __init__(self, switchRobot, value = False):
        MotivationUnit.__init__(self, "switch" , group_name='cognitive_layer')
        self.old_decoupled = value
        self.count_current_state = 0
        self.decoupled = value
        self.call_side_effect_at_iteration = -1
        self.switchRobot = switchRobot

    def callImmediateSideEffect(self, value):
        if (value):
            self.switchRobot.wRobot.robot.lock()
            print("Robot got instable - stop walking and plan solution")
            
            #input()
            for wleg in self.switchRobot.wRobot.wlegs:
                print(wleg.leg.name, " POS - ", wleg.leg.input_foot_position)
                print(wleg.leg.name, " ANGLES - ", wleg.leg.getInputPosition())
                wleg.controlVelocities = numpy.zeros(3)
            #input()
            self.switchRobot.wRobot.sendAllAngleVelocity()
#           print("SEND ZERO VELOCITIES TO ROBOT")
            for motiv_leg in self.switchRobot.mmcRobot.motivationNetRobot.motivationNetLegs:
                motiv_leg.swing_net.saveCurrentSwingState()
                if (motiv_leg.aep_temp is not None):
                    motiv_leg.aep = motiv_leg.aep_init

            for mmc_leg in self.switchRobot.mmcRobot.mmcLegs:
                mmc_leg.init_leg_mmc_model()
            # Turn on visualization during planning
            self.switchRobot.mmcRobot.turnOnCurrentBodyModelDrawing()
        else:
            self.switchRobot.wRobot.robot.unlock()
            print("SWITCH BACK TO BEHAVIOUR")
            # Turn on visualization during planning
            #input()
            self.switchRobot.mmcRobot.turnOffCurrentBodyModelDrawing()

            self.resetSystemToInitialStateOfInternalSimulation()

    def resetSystemToInitialStateOfInternalSimulation(self):
        # 1. Bring the (internal simulation) model legs to the position of the real legs
        for mmc_leg in self.switchRobot.mmcRobot.mmcLegs:
            mmc_leg.init_leg_mmc_model()

        # 2. Update the leg states = which leg is on ground, has ground contact
        for i in range(0, 6):
            self.switchRobot.mmcRobot.lift_leg_from_ground(i)
            self.switchRobot.mmcRobot.old_stance_motivation[i] = False
        for motiv_leg, leg_nr in zip(self.switchRobot.mmcRobot.motivationNetRobot.motivationNetLegs,
                range(len(self.switchRobot.mmcRobot.motivationNetRobot.motivationNetLegs))):
            self.switchRobot.mmcRobot.updateSingleLegState(motiv_leg, leg_nr)
        for i in range(0,WSTATIC.mmc_mental_iterations):
            self.switchRobot.mmcRobot.mmc_iteration_step()

        # 3. For the current body state: converge onto stable behavior selection
        # - meaning: for all legs on ground go to stance
        for motiv_leg in self.switchRobot.mmcRobot.motivationNetRobot.motivationNetLegs:
            motiv_leg.swing_net.resetToSavedSwingState()

        finalizeNeuralNetStep('sensors')
        finalizeNeuralNetStep('behavior_layer')

        for i in range(0, (round(WSTATIC.minimum_swing_time * 100) + 10) ):
            for motiv_leg in self.switchRobot.mmcRobot.motivationNetRobot.motivationNetLegs:
                if (motiv_leg.gc.output_value < 0.01):
                    motiv_leg.swing_motivation.addIncomingValue( 3. )
            executeNeuralNetStep('sensors')
            executeNeuralNetStep('behavior_layer')
        for motiv_leg in self.switchRobot.mmcRobot.motivationNetRobot.motivationNetLegs:
            motiv_leg.swing_net.resetToSavedSwingState()
            motiv_leg.rule2.stance_start_time = Decimal('-Infinity')
            motiv_leg.rule2.sender_is_swinging_old=False

        # 4. Reset all accumulated (in step 3) controlVelocities
        for wleg in self.switchRobot.wRobot.wlegs:
            wleg.controlVelocities = numpy.zeros(3)
        for leg in self.switchRobot.mmcRobot.mmcLegs:
            leg.controlVelocities = numpy.zeros(3)


    ##  Output function (automatically called in processing of network).
    #   Restricts value range to 0<=output<=1.
    def applyOutputFunction(self):

#       if (self.switchRobot.mmcRobot.motivationNetRobot.cognitive_expansion.Phase.MU_TestBeh.output_value > 0.):
#           mot_str = ""
#           mot_str = mot_str + " SW: " + '{:.4}'.format(float(self.switchRobot.mmcRobot.motivationNetRobot.hind_right_leg.swing_motivation.output_value)) + "\t" 
#           mot_str = mot_str + " ST: " + '{:.4}'.format(float(self.switchRobot.mmcRobot.motivationNetRobot.hind_right_leg.stance_motivation.output_value)) + "\t" 
#           mot_str = mot_str + " GC: " + '{:.4}'.format(float(self.switchRobot.mmcRobot.motivationNetRobot.hind_right_leg.gc.output_value)) + "\t" 
#           mot_str = mot_str + " behindPEP: " + '{:.4}'.format(float(self.switchRobot.mmcRobot.motivationNetRobot.hind_right_leg.behindPEP.output_value)) + "\t" 
#           print("TESTBEHAVIOR - " + mot_str)

        MotivationUnit.applyOutputFunction(self)
        # Calls side effect number of steps after switching:
#       if (self.count_current_state == self.call_side_effect_at_iteration):
#           print("SIDE EFFECT CALLED")

        if self.output_value == 1:
            self.decoupled = True
            #print('sending all angle velocity from switch object.')
            self.switchRobot.wRobot.sendAllAngleVelocity()
        else:
            self.decoupled = False
        if self.old_decoupled == self.decoupled:
            self.count_current_state += 1
        else:
            self.old_decoupled = self.decoupled
            self.count_current_state = 0
            self.callImmediateSideEffect(self.decoupled)

# class SwitchObject (object):
# 
#   def __init__(self, value = False):
#       self.decoupled = value

# from controller.reaCog.MotivationNetwork.MotivationUnit import executeNeuralNetStep
# print("TEST")
# oneUnit = MotivationUnit("One")
# oneUnit.addConnectionTo(oneUnit, 1)
# oneUnit.addIncomingValue(1)
# 
# switchUnit = SwitchObject()
# oneUnit.addConnectionTo(switchUnit, 1)
# 
# for i in range(20):
#   print(oneUnit.output_value, "SW: ", switchUnit.decoupled, switchUnit.output_value)
#   executeNeuralNetStep()
#   if i==10:
#       oneUnit.addConnectionTo(switchUnit, -1)
#   input()
