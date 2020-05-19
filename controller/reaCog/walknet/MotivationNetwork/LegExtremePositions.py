#!/usr/bin/env python
# -*- coding: utf-8 -*-

from controller.reaCog.MotivationNetwork.ModulatedRoutine import ModulatedRoutine
import copy

##
#	LegExtremePositions is a ModulatedRoutine
#	which is invoked by a ModulatingMotivationUnit.
#	When this unit is active it calls (after complete evaluation of the network)
#	as a side-effect the execution of the routine.
#	
#	The LegExtremePositions is called for setting the movement direction (forward or
#	backward walking) and the corresponding extreme positions for the legs.
##
class LegExtremePositions(ModulatedRoutine):

	def __init__(self, leg, forward=True ):
		self.forward = forward
		self.leg = leg
		self.setExtremePositionsForWalkingDirection()

	def setExtremePositionsForWalkingDirection(self):
		if self.forward:
			self.swingTarget = copy.copy(self.leg.aep)
			self.stanceTarget = copy.copy(self.leg.pep)
		else:
			self.stanceTarget = copy.copy(self.leg.aep)
			self.swingTarget = copy.copy(self.leg.pep)

	##	Function called by the ModulatingMotivationUnit when active.
	#	Invokes the execution of the routine which has to be defined by the derived 
	#	classes.
	def switchToForwardWalking(self, weight):
		self.forward = True
		self.setExtremePositionsForWalkingDirection()
		
	def switchToBackwardWalking(self, weight):
		self.forward = False
		self.setExtremePositionsForWalkingDirection()