# -*- coding: utf-8 -*-

##@package Movements.StandUpDemoF

#Settings

#Tools
import numpy
import math
import copy
import Hector.RobotSettings as RSTATIC
from ProcessOrganisation.ProcessModule.ControllerModule import ControllerModule

##
#	Simple controller which commands the legs to lift/lower the body of the robot to a given height.
#	The height parameter defines the distance between ground and body. A positive value means that the body is over the ground. 
#	A negative value means that the body is below the ground! If the height is not provided, the legs are just leveled (each leg will move to the legs' mean height).
#	Additionally, the maximum speed of the legs must be defined. The leg that is furthest away from the target height will be moved with this maximum speed. The other legs will be moved with speeds that make sure that all legs reach their respective goal positions at the same time. 
#
class AdjustHeightOfRobot (ControllerModule):

	# \brief Initialises important variables
	# \param 
	def __init__(self, name, robot, max_speed, target_height=None):
		ControllerModule.__init__(self, name, robot)
		self.target_height=target_height
		self.max_speed=max_speed
		
		
		self.legs=self.robot.legs
		self.intermediate_positions=[]
		
		self.iteration_number=0
		self.max_num_of_iterations=0
		
	def init_module(self):
		
		if self.target_height==None:
			summed_height=0
			for leg in self.legs:
				summed_height+=leg.input_foot_position[2]
			self.target_height=-summed_height/len(self.legs)
		start_positions=[]
		end_positions=[]
		
		distances=[]
		
		max_distance=0
		for leg in self.legs:
			start_position=leg.input_foot_position
			end_position=copy.copy(start_position)
			end_position[2]=-self.target_height
			distance=numpy.linalg.norm(end_position-start_position)
			
			start_positions.append(start_position)
			end_positions.append(end_position)
			distances.append(distance)
			if max_distance < distance:
				max_distance = distance
				
		if max_distance>0:
			self.max_num_of_iterations=math.ceil((max_distance/self.max_speed)*RSTATIC.controller_frequency)
			self.max_speed=max_distance/self.max_num_of_iterations*RSTATIC.controller_frequency
			for leg_nr in range(len(self.legs)):
				norm_direction_vector=(end_positions[leg_nr]-start_positions[leg_nr])/distances[leg_nr]
				speed=self.max_speed/max_distance*distances[leg_nr]
				points=[]
				for iter_num in range(self.max_num_of_iterations):
					points.append(start_positions[leg_nr]+norm_direction_vector*speed/RSTATIC.controller_frequency*iter_num)
				points.append(end_positions[leg_nr])
				self.intermediate_positions.append(points)
		else:
			self.max_num_of_iterations=0
			self.intermediate_positions([[ep,] for ep in end_positions])
		#print("self.intermediate_positions: ", self.intermediate_positions)
		
		
	"""
	CONTROL STEP
	"""
	## Set new control velocities for joints
	#\param timeStamp	current simulator time
	def processing_step(self, timeStamp):
		self.iteration_number+=1
		target_positions=[]
		if self.iteration_number<self.max_num_of_iterations:
			target_positions=[self.intermediate_positions[leg_nr][self.iteration_number] for leg_nr in range(len(self.intermediate_positions))]
		else:
			self._notifyOfCompletion()
			target_positions=[self.intermediate_positions[leg_nr][-1] for leg_nr in range(len(self.intermediate_positions))]
			
		# Control joints for all legs
		for leg_nr, leg in enumerate(self.legs):
			current_angles = [leg.alpha.inputPosition, leg.beta.inputPosition, leg.gamma.inputPosition]
			target_angles = leg.computeInverseKinematics(target_positions[leg_nr])
			
			leg.alpha.desiredValue_ISC = float(target_angles[0] - current_angles[0])*RSTATIC.controller_frequency
			leg.beta.desiredValue_ISC = float(target_angles[1] - current_angles[1])*RSTATIC.controller_frequency
			leg.gamma.desiredValue_ISC = float(target_angles[2] - current_angles[2])*RSTATIC.controller_frequency

			
