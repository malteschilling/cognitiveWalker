# -*- coding: utf-8 -*-

##@package Movements.StandUpDemoF

#Settings

#Tools
import numpy
import copy

#Communication
import Hector.RobotSettings as RSTATIC
from ProcessOrganisation.ProcessModule.ControllerModule import ControllerModule

class MoveSingleLeg (ControllerModule):
	# \brief Initialises important variables
	# \param 
	def __init__(self, name, robot, leg_names, final_target_positions, max_speed):
		ControllerModule.__init__(self, name, robot)
		
		if isinstance(leg_names, str):
			leg_names=(leg_names,)
		if len(leg_names)==1 and len(final_target_positions)==3 and all([isinstance(tp, (float, int)) for tp in final_target_positions]):
			final_target_positions=(final_target_positions, )
		if len(leg_names)!=len(final_target_positions):
			raise Exception('The number of leg names must equal the number of target_positions!')
		
		self.leg_names = copy.copy(leg_names)
		self.max_speed=abs(max_speed)
				
		remaining_legs=set(self.robot.legs)
		
		self.legs=[]
		self.final_target_positions=[]
		for leg_names, final_target_positions in zip(leg_names, final_target_positions):
			for remaining_leg in remaining_legs:
				if remaining_leg.name==leg_names:
					self.legs.append(remaining_leg)
					self.final_target_positions.append(final_target_positions)
					remaining_legs.remove(remaining_leg)
					break
			else:
					raise Exception('For the leg name "' + leg_names + '" no corresponding leg could be found in the robot!')
		for rl in remaining_legs:
			self.legs.append(rl)
			self.final_target_positions.append(None)
		#self.current_target_positions=[]
		self.current_desired_positions=[]
		
		
		self.preprocessing_update_list=[]
		joint_attributes_to_be_updated=['inputPosition', 'torsion', 'outputPosition']
		for leg in self.robot.legs:
			for joint in leg.joints:
				for attribute_name in joint_attributes_to_be_updated:
					self.preprocessing_update_list.append( (joint, attribute_name) )
		

	def init_module(self):
		for index, leg in enumerate(self.legs):
			if self.final_target_positions[index]==None:
				self.final_target_positions[index]=leg.input_foot_position
		self.current_desired_positions=[leg.input_foot_position for leg in self.legs]
		
	def pre_processing_step(self, timestamp):
		for client, attribute_name in self.preprocessing_update_list:
			client.UpdateValueIfTooOld(attribute_name)
	"""
	CONTROL STEP
	"""
	## Set new control velocities for joints
	#\param timeStamp	current simulator time
	def processing_step(self, timeStamp):
		if all([numpy.linalg.norm(cur_des_pos-tar_pos)<0.001 for cur_des_pos, tar_pos in zip(self.current_desired_positions,self.final_target_positions)]):
			self._notifyOfCompletion()
				
		for leg_index, leg in enumerate(self.legs):
			current_angles = [leg.alpha.inputPosition, leg.beta.inputPosition, leg.gamma.inputPosition]
			current_position=leg.computeForwardKinematics(current_angles)
			self.current_desired_positions[leg_index]=copy.copy(self.final_target_positions[leg_index])
			velocity_vector=(self.final_target_positions[leg_index]-current_position)*RSTATIC.controller_frequency

			norm_of_velocity_vector=numpy.linalg.norm(velocity_vector)
			if norm_of_velocity_vector>self.max_speed:
				velocity_vector=velocity_vector/norm_of_velocity_vector*self.max_speed
				self.current_desired_positions[leg_index]=current_position+velocity_vector/RSTATIC.controller_frequency

			target_angles = leg.computeInverseKinematics(self.current_desired_positions[leg_index])
			
			alpha_speed=float(target_angles[0] - current_angles[0])*RSTATIC.controller_frequency
			beta_speed=float(target_angles[1] - current_angles[1])*RSTATIC.controller_frequency
			gamma_speed=float(target_angles[2] - current_angles[2])*RSTATIC.controller_frequency
			leg.alpha.desiredValue_ISC = alpha_speed
			leg.beta.desiredValue_ISC = beta_speed
			leg.gamma.desiredValue_ISC = gamma_speed





