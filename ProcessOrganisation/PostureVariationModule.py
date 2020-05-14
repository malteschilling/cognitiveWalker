# -*- coding: utf-8 -*-
import numpy
import time

#Tools
from .ProcessModule.ProcessingModule import ProcessingModule
import controller.reaCog.WalknetSettings as WSTATIC
from tools.FreezableF import Freezable as Freezable
from decimal import Decimal

##
# Calculate Robot starting posture variations 
# and drive the simulated robot into that posture.
##
class PostureVariationModule (ProcessingModule, Freezable):
	__current_time=Decimal('0')
	time_precision=Decimal('0.0001')
	##
	#	Init
	#	@param name for the module
	#	@param	the Hector robot structure (to access data and communication protocol)
	# 	@posture_code describes the posture - given for each leg (FL, FR, ML, ...)
	#		the posture is described by a number: 0 = AEP, up to 'variation_step_number'
	#		is close to the standard PEP
	def __init__(self, name, robot, posture_code):
		self.name = name
		self.robot = robot
#		self.communication_interface = robot.communication_interface
		
		ProcessingModule.__init__(self, name)
		# Create a communication client for the timing of the simulation
#		self.timer=self.communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])
		self.it = -10
		self.init_leg = -1
		
		variation_step_number = 4
		variation_step_size = [ (WSTATIC.front_initial_pep - WSTATIC.front_initial_aep)/variation_step_number, \
			(WSTATIC.middle_initial_pep - WSTATIC.middle_initial_aep)/variation_step_number, \
			(WSTATIC.hind_initial_pep - WSTATIC.hind_initial_aep)/variation_step_number]
		self.start_posture = [ (WSTATIC.front_initial_aep + variation_step_size[0] * int(posture_code[0])), \
			(WSTATIC.front_initial_aep + variation_step_size[0] * int(posture_code[1])), \
			(WSTATIC.middle_initial_aep + variation_step_size[1] * int(posture_code[2])), \
			(WSTATIC.middle_initial_aep + variation_step_size[1] * int(posture_code[3])), \
			(WSTATIC.hind_initial_aep + variation_step_size[2] * int(posture_code[4])), \
			(WSTATIC.hind_initial_aep + variation_step_size[2] * int(posture_code[5])) ]
		# Adjust for left legs the y position (switch sign)
		self.start_posture[0][1] = -self.start_posture[0][1]
		self.start_posture[2][1] = -self.start_posture[2][1]
		self.start_posture[4][1] = -self.start_posture[4][1]
		self.angles = []
		self.frozen=True
		
	def processing_step(self, timeStamp):
		if ( 0 <= self.it ):
			if (self.it % 10 == 0):
				self.init_leg += 1
				if (self.init_leg <= 5):
					self.it = 0
					#print("START: ", self.robot.robot.legs[self.init_leg].getInputAngles(), " - ", self.init_leg)
					self.angles = self.robot.robot.legs[self.init_leg].computeInverseKinematics( self.start_posture[self.init_leg] )
					#print("GOAL:  ", self.angles)
			if (self.init_leg <= 5):
				self.robot.robot.legs[self.init_leg].alpha.desiredPosition = float(self.angles[0])
				self.robot.robot.legs[self.init_leg].beta.desiredPosition = float(self.angles[1])
				self.robot.robot.legs[self.init_leg].gamma.desiredPosition = float(self.angles[2])
				#print(self.robot.robot.legs[self.init_leg].getInputAngles())
		self.it += 1
		
#		self.robot.legs[0].updateJointSensorInformation()
#		print(self.robot.legs[0].getInputAngles())
#		self.robot.legs[0].beta.desiredPosition = float(-0.2)
		
		# Let the simulation run for a given time.
		#self.timer.relTimerMs=1/self.controllerFrequency
		#SimulatorTimerModule.__current_time+=Decimal(1/self.controllerFrequency ).quantize(self.time_precision)
		
#		self.robot.legs[0].updateJointSensorInformation()
#		print(self.robot.legs[0].getInputAngles())
		
#		if (self.it % 10 == 0):
#			if (0 <= self.init_leg <= 5):
#				print(self.robot.robot.legs[self.init_leg].input_foot_position)
#				print(self.robot.robot.legs[self.init_leg].alpha.inputPosition)
#				angles = self.robot.robot.legs[self.init_leg].computeInverseKinematics( self.start_posture[self.init_leg] )
#				print(angles[0])
#				self.robot.robot.legs[self.init_leg].alpha.desiredPosition = float(angles[0])
#				#print(self.robot.robot.legs[self.init_leg].name)
			
#		print("posture variation ", timeStamp)
		#start_posture = [ (front_initial_aep + variation_step_size[0] * int(posture_code[0])), \
		#	(front_initial_aep + variation_step_size[0] * int(posture_code[1])), \
		#	(middle_initial_aep + variation_step_size[1] * int(posture_code[2])), \
		#	(middle_initial_aep + variation_step_size[1] * int(posture_code[3])), \
		#	(hind_initial_aep + variation_step_size[2] * int(posture_code[4])), \
		#	(hind_initial_aep + variation_step_size[2] * int(posture_code[5])) ]
	
		
	##
	#	Post processing: Sending data to the Simulator after the control values
	#	are calculated.
	#	@param timeStamp current simulator time
#	def post_processing_step(self, timeStamp):
		# Capture every simulation step
		#if (self.it % 2):
		#	self.timer.captureFrame = float(self.__current_time)/100
		#	time.sleep(0.02)
		#self.it += 1
		# Let the simulation run for a given time.
#		self.timer.relTimerMs=1/self.controllerFrequency
#		SimulatorTimerModule.__current_time+=Decimal(1/self.controllerFrequency ).quantize(self.time_precision)
#		self.timer.robotTransparency = 0.59181716
#		self.timer.internalModelGlobalPosition = str([[2., 1.],\
#				[2., 1.],\
#				[0., 0.],\
#				[1., 0.],\
#				[1.03, 0.],\
#				[1., 0.]])
	## 
	#	Init the module - called before it is iterated over all the modules
	def init_module(self):
		if __debug__:
			print('Init posture variation timing module')