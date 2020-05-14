#!/usr/bin/env python
# -*- coding: utf-8 -*-

from controller.reaCog.MotivationNetwork.ModulatedRoutine import ModulatedRoutine
import copy

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