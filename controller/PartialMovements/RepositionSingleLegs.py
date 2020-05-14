# -*- coding: utf-8 -*-

##@package Movements.StandUpDemoF

#Settings

#Tools
import numpy
import copy

#Communication
import Hector.RobotSettings as RSTATIC
from ProcessOrganisation.ProcessModule.ControllerModule import ControllerModule

##
#	Simple controller which just computes the forward kinematic of each individual leg 
#	and afterwards controls all legs in order to move them to the same height
#	(mean of the current position of the legs)
#
class RepositionSingleLegs (ControllerModule):
	# \brief Initialises important variables
	# \param 
	def __init__(self, name, robot, leg_names, final_target_positions, max_speed, lift_height):
		ControllerModule.__init__(self, name, robot)
		
		if isinstance(leg_names, str):
			leg_names=(leg_names,)
		if len(leg_names)==1 and len(final_target_positions)==3 and all([isinstance(tp, (float, int)) for tp in final_target_positions]):
			final_target_positions=(final_target_positions, )
		if len(leg_names)!=len(final_target_positions):
			raise Exception('The number of leg names must equal the number of target_positions!')
		
		self.leg_names = copy.copy(leg_names)
		self.max_speed=abs(max_speed)
		self.lift_height=lift_height
		
		
		remaining_legs=set(self.robot.legs)
		
		self.legs=[]
		self.final_target_positions=[]
		for leg_name, final_target_position in zip(leg_names, final_target_positions):
			for remaining_leg in remaining_legs:
				if remaining_leg.name==leg_name:
					self.legs.append(remaining_leg)
					self.final_target_positions.append(final_target_position)
					remaining_legs.remove(remaining_leg)
					break
			else:
				raise Exception('For the leg name "' + leg_name + '" no corresponding leg could be found in the robot!')
		self.legs.extend(list(remaining_legs))
		
		self.currently_moving_leg_index=0
		self.current_movement_state=None
		self.temporary_target_positions=[]
		self.current_desired_positions=[]

	def init_module(self):
		for leg in self.legs:
			self.current_desired_positions.append(leg.input_foot_position)
		
		self.temporary_target_positions=copy.copy(self.current_desired_positions)

	"""
	CONTROL STEP
	"""
	## Set new control velocities for joints
	#\param timeStamp	current simulator time
	def processing_step(self, timeStamp):
		
		if numpy.linalg.norm(self.current_desired_positions[self.currently_moving_leg_index]-self.final_target_positions[self.currently_moving_leg_index])<0.01:
			self.current_movement_state=None
			self.currently_moving_leg_index+=1
			
			
		if self.currently_moving_leg_index>=len(self.final_target_positions):
			self._notifyOfCompletion()
		elif self.current_movement_state==None or numpy.linalg.norm(self.current_desired_positions[self.currently_moving_leg_index]-self.temporary_target_positions[self.currently_moving_leg_index])<0.01:
			#print('moving leg nr ', self.currently_moving_leg_index)
			if self.current_movement_state==None:
				self.current_movement_state='lifting'
				#print('lifting')
				self.temporary_target_positions[self.currently_moving_leg_index]=self.temporary_target_positions[self.currently_moving_leg_index]+numpy.array([0,0,self.lift_height])
			elif self.current_movement_state=='lifting':
				self.current_movement_state='repositioning'
				self.temporary_target_positions[self.currently_moving_leg_index]=self.final_target_positions[self.currently_moving_leg_index]+numpy.array([0,0,self.lift_height])
			elif self.current_movement_state=='repositioning':
				self.current_movement_state='lowering'
				self.temporary_target_positions[self.currently_moving_leg_index]=self.final_target_positions[self.currently_moving_leg_index]
			
				
		for leg_index, leg in enumerate(self.legs):
			current_angles = leg.getInputAngles()
			current_position=leg.input_foot_position
			self.current_desired_positions[leg_index]=copy.copy(self.temporary_target_positions[leg_index])
			velocity_vector=(self.current_desired_positions[leg_index]-current_position)*RSTATIC.controller_frequency

			norm_of_velocity_vector=numpy.linalg.norm(velocity_vector)
			if norm_of_velocity_vector>self.max_speed:
				velocity_vector=velocity_vector/norm_of_velocity_vector*self.max_speed
				self.current_desired_positions[leg_index]=current_position+velocity_vector/RSTATIC.controller_frequency

			target_angles = leg.computeInverseKinematics(self.current_desired_positions[leg_index])
			
			alpha_speed=float(target_angles[0] - current_angles[0])*RSTATIC.controller_frequency
			beta_speed=float(target_angles[1] - current_angles[1])*RSTATIC.controller_frequency
			gamma_speed=float(target_angles[2] - current_angles[2])*RSTATIC.controller_frequency
			leg.setAngleVelocity([alpha_speed, beta_speed, gamma_speed])




