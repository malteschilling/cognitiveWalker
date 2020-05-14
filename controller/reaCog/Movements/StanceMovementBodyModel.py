#!/usr/bin/env python

from ..MotivationNetwork.ModulatedRoutine import ModulatedRoutine
import controller.reaCog.WalknetSettings as WSTATIC
import Hector.RobotSettings as RSTATIC
import numpy

# Dual Quaternion part is not adapted to new coordinate systems
#from BodymodelStance.legNetworkDualQuaternionMmc import legNetworkDualQuaternionMmc

##
# 	Stance Network.
#
#	This stance movement network is driven by the body model.
#	When active it pulls as an input a new target vector from the body model
#	for the connected leg.
#	And from this it produces joint velocities which move the leg from
#	the current joint configuration towards this goal posture.
#
#	There are different possible ways how the target joint angles are computed
#	(this is defined in WalknetSettings):
#		0 -	using the direct computation of inverse kinematics
#		1 - using an angular MMC network and iterating for a couple of steps
#		2 - using a dual quaternion MMC network for solving inverse kinematics.
#	The different approaches each are encapsulated as the inverseKinematic_provider
#	and each of these objects has a computeInverseKinematics method which
#	provides corresponding joint angles for a given target position.
#
#	A movement is derived from ModulatedRoutine which defines an interface.
#	It implements the required modulatedRoutineFunctionCall -
#	as a consequence a movement can be connected to a MotivationUnit which
#	(when active) always executes the connected modulatedRoutineFunctionCall.
##
class StanceMovementBodyModel(ModulatedRoutine):

	##	Initialisation of the Stance Movement. Connecting to the bodyModelStance and
	#	setting the inverseKinematic_provider.
	#	@param motiv_leg Motivation network of the leg
	#	@param extr_pos extreme positions of the stance movement (also encoding direction)
	def __init__(self, motiv_leg):
		ModulatedRoutine.__init__(self)
		self.motivationNetLeg = motiv_leg
		self.init_stance_footpoint = False
		self.bodyModelStance = self.motivationNetLeg.motivationNetRobot.bodyModelStance
		# Can use different solutions for calculation of the leg kinematics:
		# 0 - explicit kinematics, 1 - dual quaternion MMC, 2 - angular MMC
		if (WSTATIC.stanceLegLevelMethod == 1):
			pass
		elif (WSTATIC.stanceLegLevelMethod == 2):
			self.inverseKinematic_provider = self.motivationNetLeg.mmcLegStance
		else:
			self.inverseKinematic_provider = self.motivationNetLeg.wleg.leg

	def __del__(self):
		pass
		
	def resetStanceTrajectory(self):
		self.init_stance_footpoint = False

	##	Function called by the ModulatingMotivationUnit when active.
	#	Invokes the execution of the routine which has to be defined by the derived
	#	classes.
	def modulatedRoutineFunctionCall(self, weight, param):
		if (self.motivationNetLeg.wleg.leg.leg_enabled) :
			if not(self.init_stance_footpoint):
				#print("Stance on ground - ", self.motivationNetLeg.wleg.leg.name, 
				#    self.motivationNetLeg.wleg.leg.input_foot_position,
				#    self.motivationNetLeg.pep_shifted[0])
				#self.motivationNetLeg.motivationNetRobot.motivationNetLegs[2].rule3Ipsilateral.printCoordRule3()
				stance_foot_pos = self.motivationNetLeg.wleg.leg.input_foot_position
				if (stance_foot_pos[0] <= self.motivationNetLeg.pep_shifted[0] ):
				    print("Stance correction: foot moved in Body Model in front of PEP ", self.motivationNetLeg.wleg.leg.name)
				    stance_foot_pos[0] = self.motivationNetLeg.pep_shifted[0] + 0.02
				self.bodyModelStance.put_leg_on_ground(self.motivationNetLeg.wleg.leg.name, \
					self.motivationNetLeg.wleg.leg.input_foot_position)
				self.init_stance_footpoint = True
#			import time
#			before = time.time()
			next_angles = self.inverseKinematic_provider.computeInverseKinematics( self.bodyModelStance.get_leg_vector(self.motivationNetLeg.wleg.leg.name))
	#			self.bodyModelStance.put_leg_on_ground(self.motivationNetLeg.wleg.leg.name, \
	#				self.bodyModelStance.transform_robot_target_to_leg_vector(self.motivationNetLeg.wleg.leg.input_foot_position, self.motivationNetLeg.wleg.leg.name) )

	#		next_angles = self.inverseKinematic_provider.computeInverseKinematics( self.bodyModelStance.get_robot_target_from_leg_vector(self.motivationNetLeg.wleg.leg.name))
#			after = time.time()
#			py_time = after-before
#			print("MMC Time: ", py_time)
#			self.bodyModelStance.put_leg_on_ground(self.motivationNetLeg.wleg.leg.name, \
#					self.motivationNetLeg.wleg.leg.input_foot_position)
#			next_angles = self.inverseKinematic_provider.computeInverseKinematics( self.bodyModelStance.get_leg_vector(self.motivationNetLeg.wleg.leg.name))
			

			current_angles = numpy.array( self.motivationNetLeg.wleg.leg.getInputPosition() )
			#if(self.motivationNetLeg.wleg.leg.name == "front_right_leg"):
			#	print("Current Stance FR: ", self.motivationNetLeg.wleg.leg.getInputPosition() )
			new_joint_velocities = (next_angles - current_angles.T)/(1/RSTATIC.controller_frequency)
			
#D			if(self.motivationNetLeg.wleg.leg.name == "hind_left_leg"):
#D				print("HL Stance: ", new_joint_velocities)
			self.motivationNetLeg.wleg.addControlVelocities(new_joint_velocities)
