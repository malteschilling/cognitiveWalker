#!/usr/bin/env python
'''
Created on 8.6.2013

@author: mschilling
'''

#from BehindPepMotivationUnit import CurrentPEP
from controller.reaCog.MotivationNetwork.ModulatedRoutine import ModulatedRoutine

""" CoordinationRuleApplicationRoutine
	is a ModulatedRoutine which is invoked by a ModulatingMotivationUnit.
	When the unit is active it calls (after complete evaluation of the network)
	as a side-effect the execution of the routine via
	modulatedRoutineFunctionCall.
	
	The CoordinationRuleApplicationRoutine applies 
	- the pep shift calculated in the source_coord_rule
	- to the current_pep of the target_pep 
	- (optionally weighted by factor coord_weight).
	These three parameters have to be given to the ModulatedRoutine Object.
"""
class CoordinationRuleApplicationRoutine(ModulatedRoutine):
	def __init__(self, coord_rule, target_pep, weight = 1.):
		self.source_coord_rule = coord_rule
		self.target_pep = target_pep
		self.coord_weight = weight
		self.counter = 0

	##	Function called by the ModulatingMotivationUnit when active.
	#	Invokes the execution of the routine which shifts the target_pep
	#	as calculated in the source_coord_rule (scaled by the weight factor)	
	def modulatedRoutineFunctionCall(self, weight):
		self.target_pep.shiftCurrentPEP( self.coord_weight * self.source_coord_rule )
		#self.current_pep_shift = self.example_shift + self.counter * 0.042
		#self.counter +=1
		#print("Current pep shift")