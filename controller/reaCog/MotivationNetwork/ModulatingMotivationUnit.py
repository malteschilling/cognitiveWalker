#!/usr/bin/env python
'''
Created on 3.6.2013

@author: mschilling
'''

from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

##
# 	Modulating Motivation Unit
#	is a Motivation Unit which -when active- also invokes
#	the execution of a side-effect routine.
#	
#	Examples are the motor primitives, in that case the computation of joint velocities
#	is invoked and pushed to the motors or the activation of coordination rules.
#	The motivation unit also works as a modulating unit switching on
#	and off such behaviours.
#	
#	These modulating units have to be initialized with a ModulatedRoutine as a target
#	(which implements the modulatedRoutineFunctionCall).
##
class ModulatingMotivationUnit(MotivationUnit):
	##
	#	Init with parameters
	#	@param name String for the name of the unit
	#	@param mod_target is a function which is called when this unit is active
	#		(any function which also takes one parameter, i.e. the activation
	#		of this unit), example is ModulatedRoutine 
	#	@param threshold - defines when modulated function is called (activation
	#		> threshold)
	def __init__(self, name, mod_target, threshold = 1.0, startValue = 0, bias = 0, fct_param = None, group_name=''):
		MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
		# Source of PEP shift is a CoordinationInfluencePepShiftCalculator
		#self.source_activation = source_activation
		self.threshold = threshold
		self.modulated_target_function = mod_target
		self.mod_function_param = fct_param
	
	##	After the last step of the evaluation (application of output)
	#	the connected ModulatedRoutine is called and the side effect is invoked
	#	through the implementation of modulatedRoutineFunctionCall in the 
	#	ModulatedRoutine.
	def applyOutputFunction(self):
		MotivationUnit.applyOutputFunction(self)
		if (self.output_value >= self.threshold) :
			self.modulated_target_function(self.output_value, self.mod_function_param)

class ModulatingMotivationTimerUnit(ModulatingMotivationUnit):
	##
	#	Init with parameters
	#	@param name String for the name of the unit
	#	@param mod_target is a function which is called when this unit is active
	#		(any function which also takes one parameter, i.e. the activation
	#		of this unit), example is ModulatedRoutine 
	#	@param threshold - defines when modulated function is called (activation
	#		> threshold)
	def __init__(self, name, mod_target, threshold = 1.0, startValue = 0, bias = 0, min_activation_time = 0, fct_param = None, group_name=''):
		ModulatingMotivationUnit.__init__(self, name=name, mod_target=mod_target, threshold=threshold, startValue=startValue, bias=bias, fct_param=fct_param, group_name=group_name)
		self.min_activation_time = min_activation_time
		self.last_activation_time = float('-inf')
		self.preceding_activation = False
	
	##	After the last step of the evaluation (application of output)
	#	the connected ModulatedRoutine is called and the side effect is invoked
	#	through the implementation of modulatedRoutineFunctionCall in the 
	#	ModulatedRoutine.
	def applyOutputFunction(self):
		MotivationUnit.applyOutputFunction(self)
		if self.last_activation_time+self.min_activation_time >= getCurrentTime():
			if not(self.output_value >= self.threshold) :
				self.output_value = self.threshold
		if (self.output_value >= self.threshold) :
			self.modulated_target_function(self.output_value, self.mod_function_param)
			if not(self.preceding_activation):
				self.last_activation_time = getCurrentTime()
				self.preceding_activation = True
		else:
			self.preceding_activation = False	

##
# Lazy Modulating Motivation Unit
#	is a Motivation Unit which -when active- also invokes
#	the execution of a side-effect routine.
#	In contrast to the Modulating Motivation Unit it is only called when
#	the output value of the unit changes.	
#
#	These modulating units have to be initialized with a ModulatedRoutine as a target
#	(which implements the modulatedRoutineFunctionCall).
##
class LazyModulatingMotivationUnit(ModulatingMotivationUnit):
	##
	#	Init with parameters
	#	@param name String for the name of the unit
	#	@param mod_target is a function which is called when this unit is active
	#		(any function which also takes one parameter, i.e. the activation
	#		of this unit), example is ModulatedRoutine 
	#	@param threshold - defines when modulated function is called (activation
	#		> threshold)
	def __init__(self, name, mod_target, threshold = 1.0, startValue = 0, bias = 0, group_name=''):
		ModulatingMotivationUnit.__init__(self, name=name, mod_target=mod_target, threshold=threshold, startValue=startValue, bias=bias, group_name=group_name)
		self.old_output_value = 0.
	
	##	After the last step of the evaluation (application of output)
	#	the connected ModulatedRoutine is called and the side effect is invoked
	#	through the implementation of modulatedRoutineFunctionCall in the 
	#	ModulatedRoutine.
	def applyOutputFunction(self):
		MotivationUnit.applyOutputFunction(self)
		if (self.output_value != self.old_output_value) :
			if (self.output_value >= self.threshold) :
				self.modulated_target_function(self.output_value)
			self.old_output_value = self.output_value

##
# Motivation Unit modulating Coordination Rule Application.
#	The specific type is used for the connections given by the coordination rules
#	in the Walknet. When active, the connected coordination influence influences	
#	the targeted PEP.
##
class CoordinationRuleShiftModulatingMotivationUnit(ModulatingMotivationUnit):
	##
	#	Init with parameters
	#	@param name String for the name of the unit
	#	@param mod_target is a function which is called when this unit is active
	#		(any function which also takes one parameter, i.e. the activation
	#		of this unit), example is ModulatedRoutine 
	#	@param target_leg is the receiving leg of the rule (the PEP that should be shifted)
	#	@param threshold - defines when modulated function is called (activation
	#		> threshold)
	def __init__(self, name, mod_sender_function, weight, mod_receiver_function, threshold = 1.0, startValue = 0, bias = 0, group_name=''):
		ModulatingMotivationUnit.__init__(self, name=name, mod_target=mod_sender_function, threshold=threshold, startValue=startValue, bias=bias, group_name=group_name)
		self.weight = weight
		self.mod_receiver_function = mod_receiver_function
		self.was_active_during_last_iteration=True
	
	##	After the last step of the evaluation (application of output)
	#	the connected ModulatedRoutine is called and the side effect is invoked
	#	through the implementation of modulatedRoutineFunctionCall in the 
	#	ModulatedRoutine.
	def applyOutputFunction(self):
		MotivationUnit.applyOutputFunction(self)
		if self.output_value >= self.threshold :
			self.mod_receiver_function(self.name, self.modulated_target_function(self.output_value, self.weight))
			self.was_active_during_last_iteration=True
		else:
			if self.was_active_during_last_iteration:
				self.mod_receiver_function(self.name, 0)
				self.was_active_during_last_iteration=False
