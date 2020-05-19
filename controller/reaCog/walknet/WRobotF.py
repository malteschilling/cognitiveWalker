#!/usr/bin/env python
# -*- coding: utf-8 -*-

#from Hector.RobotF import Robot
from .WLegF import WLeg
import controller.reaCog.WalknetSettings as WSTATIC
import Hector.RobotSettings as RSTATIC
import copy#, numpy, math
from .WDriveSafetyCheck import WDriveSafetyCheck
import math
import numpy
from tools.FreezableF import Freezable as Freezable
from math import atan2

##
#	Walknet robot structure.
#	The robot structures are hierarchical and nested.
#	In the normal robot structure (in the Hector directory) the Hector
#	specific definitions, access and help functions are defined.
#	In the WRobot specific variables for Walknet are defined, e.g. AEP, PEP.
#	The same holds true for the Leg structure (in WRobot there are WLegs).
##
class WRobot(Freezable):

	def __init__(self, robot):
		self.robot = robot
		
		# The Walknet has an error handling which is slowing down the whole
		# robot and shuts down all the drives afterwards.
		
		self.desired_speed = WSTATIC.default_speed
		self.desired_direction_angle = WSTATIC.default_direction_angle
		
		temp_wlegs=[]
		for leg_name, leg in zip(RSTATIC.leg_names, self.robot.legs):
			temp_wleg=WLeg(leg)
			
			setattr(self, leg_name, temp_wleg)
			temp_wlegs.append(temp_wleg)
		self.wlegs=tuple(temp_wlegs)
		if WSTATIC.use_assumed_center_of_mass:
			self._getCenterOfMass=lambda: WSTATIC.assumed_center_of_mass
		else:
			self._getCenterOfMass=self.robot.getCenterOfMass
		
		self.frozen=True

	def setDirection(self, direction):
		if direction[0]>0:
			temp_speed=numpy.linalg.norm(direction)
		else:
			temp_speed=0
		if temp_speed>WSTATIC.max_speed:
			temp_speed=WSTATIC.max_speed
		elif temp_speed<0:
			temp_speed=0
		self.desired_speed=temp_speed
		temp_direction_angle=atan2(direction[1], direction[0])
		if temp_direction_angle>WSTATIC.max_abs_direction_angle:
			temp_direction_angle=WSTATIC.max_abs_direction_angle
		elif temp_direction_angle<-WSTATIC.max_abs_direction_angle:
			temp_direction_angle=-WSTATIC.max_abs_direction_angle
		self.desired_direction_angle=temp_direction_angle
		
	##
	#	Apply the control velocities and send them to the connected robot.
	def sendAllAngleVelocity(self):
		for wleg in self.wlegs:
			if wleg.leg.leg_enabled:
				wleg.sendControlVelocities()

	def sideway(self):
		if (WSTATIC.sidewayenabled):
			jf = self.robot.jointFront.getJointAngle()
			jb =self.robot.jointBack.getJointAngle()
			sidewayangle = (jf[0] + jb[0])/2
			total = self.front_left_leg.xmlAnchors[0]- self.hind_left_leg.xmlAnchors[0]
			
			for leg_name, wleg in zip(RSTATIC.leg_names, self.wlegs):
				if getattr(RSTATIC,leg_name+"_enable"):
					wleg.turnwalk(sidewayangle,total)
	@property
	def center_of_mass(self):
		return self._getCenterOfMass()
