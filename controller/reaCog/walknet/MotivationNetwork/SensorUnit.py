#!/usr/bin/env python
'''
Created on 3.6.2013

@author: mschilling
'''
from ... import WalknetSettings as WSTATIC 
from controller.reaCog.MotivationNetwork.MotivationUnit import Neuron
from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import getCurrentTime as getCurrentTime

""" A Sensor Unit.
	A Sensor Unit is a specific neuron. The activation does not depend on
	the ingoing values (evaluate is simply not done), but the value is
	read out directly from a sensor and represents a state of the world/body.
	
	The value is immediately pulled before its propagation (therefore there
	is no evaluate step).
	The updateSensorValue has to be implemented by the deriving classes and is 
	called during propagate.
"""
class SensorUnit(Neuron):
	def __init__(self, name, group_name=''):
		Neuron.__init__(self, group_name=group_name)
		self.name = name

	def propagate(self):
		self.updateSensorValue()
		self.applyOutputFunction()
		Neuron.propagate(self)
		
	def evaluate(self):
		pass
		
	##	The updateSensorValue has to be implemented by the deriving classes 
	#	and is called during propagate.
	#	This function defines how the sensor value is pulled.
	#	The value is immediately pulled before its propagation (therefore there
	#	is no evaluate step).
	def updateSensorValue(self):
		pass

""" 
	Behind PEP Units is not a strict sensor unit.
	On the one hand, it pulls the current position of the leg.
	On the other hand, it uses the current PEP (shifted by the coordination rules).
	
	In addition, there is a time behaviour: when turned on, the unit stays on for 10
	time steps (used in switch from stance to swing, as in the beginning the swing
	movement would be aborted immediately as the ground contact is still active
	during the first controller time steps).
"""
class BehindPEPSensorUnit(SensorUnit):
	
	def __init__(self, motivLeg, group_name=''):
		SensorUnit.__init__(self, name="behindPEP_" + motivLeg.wleg.leg.name[0] + motivLeg.wleg.leg.name[motivLeg.wleg.leg.name.index('_')+1], group_name=group_name)
		self.motivationNetLeg = motivLeg
		self.wleg = self.motivationNetLeg.wleg
#R		self.min_active_time = WSTATIC.minimum_swing_time
#R		self.last_activation_time=float('-inf')
		self.output_value = 0
		
	def updateSensorValue(self):
		if True:#self.leg_extreme_positions.forward): # Using the if(True) pretends that the robot will always move forwards.
			self.output_value = 0
			if (self.wleg.leg.leg_enabled):
#R				if self.wleg.leg.input_foot_position[0] <= self.motivationNetLeg.pep_shifted[0]  and self.motivationNetLeg.inStancePhase():
				if self.wleg.leg.input_foot_position[0] <= self.motivationNetLeg.pep_shifted[0]:
					self.output_value = 1.
#R					self.last_activation_time = getCurrentTime()
#R				elif self.last_activation_time+self.min_active_time >= getCurrentTime():
#R					self.output_value = 1.

		# When moving backward
		#else:
		#	if self.wleg.input_foot_position[0] >= (self.wleg.aep[0] - 0.01 - self.motivationNetLeg.pep_shifted):
		#		self.output_value = 1.
		#		self.swing_min_counter = 10
		#	else:
		#		if self.swing_min_counter > 0:
		#			self.output_value = 1.
		#			self.swing_min_counter -= 1
		#		else:
		#			self.output_value = 0
		# Reset the pep shift variable for the leg
		#self.wleg.resetPepShift()
