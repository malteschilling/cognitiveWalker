from Hector.DriveSafetyCheck import DriveSafetyCheck

##
#	Check if all drives are ready and if not
#	shut the drive down and disable the corresponding leg.
##
class WDriveSafetyCheck(DriveSafetyCheck):

	##
	#	Init check drive.
	def __init__(self, mrobot):
		DriveSafetyCheck.__init__(self, mrobot.wrobot.robot)
		self.mrobot = mrobot
		# When an error is detected the robot should be decelerated
		self.decelerate = False
		self.velocity_factor=1
		
	##	Check all drives.
	#	Is called during the precontrol step from the Robot object.
	#	\param timeStamp	current simulator time
	#	Check all the single drives and if an error has been found disable the leg.
	def safetyCheck(self, timeStamp=None):
		numOfErrors=0
		for leg in self.mrobot.wrobot.robot.legs:
			if leg.leg_enabled:
				if self.fatalErrorCheck(leg)>0:
					numOfErrors+=1
		if numOfErrors>0:
			self.decelerate = True
		# After an error the robot should be decelerated slowly.
		if (self.decelerate):
			if (self.velocity_factor >= 0.005):
				self.velocity_factor += -0.005
			# When the robot has stopped, all the legs on the ground are deactivated
			# (the legs in swing will continue their movement)
			else:
				self.velocity_factor = 0.
				if all([mleg.inStancePhase() for mleg in self.mrobot.motivationNetLegs]):
					for wleg in self.mrobot.wrobot.wlegs:
						if wleg.leg.leg_enabled and (wleg.predictedGroundContact() ):
							wleg.leg.leg_enabled = False
