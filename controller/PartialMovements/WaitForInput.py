# -*- coding: utf-8 -*-

##@package Movements.StandUpDemoF

#Settings

#Tools

#Communication
import Hector.RobotSettings as RSTATIC
from ProcessOrganisation.ProcessModule.ControllerModule import ControllerModule

class WaitForInput(ControllerModule):

	# \brief Initialises important variables
	# \param 
	def __init__(self, name, robot, message='Waiting for signal to stop...'):
		ControllerModule.__init__(self, name, robot)
		
		self.legs=self.robot.legs
		self.target_positions=[]
		
		self.message=message
	
	def handleMessage(self, message):
		if "controller_event" in message and message["controller_event"]=="come_to_end":
			self._notifyOfCompletion()
		
	def init_module(self):
		print(self.message)
		for leg in self.legs:
			if leg.leg_enabled:
				self.target_positions.append(leg.input_foot_position)
			else:
				self.target_positions.append(None)
		
	"""
	CONTROL STEP
	"""
	## Set new control velocities for joints
	#\param timeStamp	current simulator time
	def processing_step(self, timeStamp):
		# Control joints for all legs
		for leg, leg_nr in zip(self.legs, range(len(self.legs))):
			if leg.leg_enabled and not(isinstance(self.target_positions[leg_nr], (type(None)))):
				current_angles = [leg.alpha.inputPosition, leg.beta.inputPosition, leg.gamma.inputPosition]
				target_angles = leg.computeInverseKinematics(self.target_positions[leg_nr])
				
				leg.alpha.desiredValue_ISC = float(target_angles[0] - current_angles[0])*RSTATIC.controller_frequency
				leg.beta.desiredValue_ISC = float(target_angles[1] - current_angles[1])*RSTATIC.controller_frequency
				leg.gamma.desiredValue_ISC = float(target_angles[2] - current_angles[2])*RSTATIC.controller_frequency
