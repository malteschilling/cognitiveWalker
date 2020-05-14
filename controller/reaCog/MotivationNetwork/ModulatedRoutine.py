#!/usr/bin/env python
'''
Created on 3.6.2013

@author: mschilling
'''

##
#	ModulatedRoutine
#	is a routine which is invoked by a ModulatingMotivationUnit.
#	When the unit is active it calls (after complete evaluation of the network)
#	as a side-effect the execution of the routine via
#	modulatedRoutineFunctionCall (which has to be implemented by derived classes).
#	
#	Examples are the motor primitives, in that case the computation of joint velocities
#	is invoked and pushed to the motors or the activation of coordination rules.
#	The motivation unit works as the modulating unit switching on
#	and off such behaviours.
##
class ModulatedRoutine(object):
	
	##	Function called by the ModulatingMotivationUnit when active.
	#	Invokes the execution of the routine which has to be defined by the derived 
	#	classes.
	#	@param weight is the activation of the ModulatingMotivationUnit which can
	#		can be used to weight the effect of the routine
	def modulatedRoutineFunctionCall(self, weight):
		pass