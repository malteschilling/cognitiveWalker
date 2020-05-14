class SwitchLeg (object):

	def __init__(self, realLeg, mmcLeg, switchObject):
		self.__dict__["forwarding"] = False
		self.realLeg = realLeg
		# Create Body model and link up legs
		self.mmcLeg = mmcLeg
		# Switch to original model
		self.switch = switchObject
		self.__dict__["forwarding"] = True
		
	def __del__(self):
		pass

	## Umlenken von Anfragen an die Instanz von Leg
	def __getattr__(self,name):
		try :
			if self.switch.decoupled:
#				if (name == "predictedGroundContact"):
#					print("MMC GC CALL: ")
				return getattr(self.mmcLeg, name)
			else:
				return getattr(self.realLeg, name)
		except AttributeError :
			print("Konnte ",name, "nicht in WLeg und leg finden" )
			raise

	def __setattr__(self, name, value):
		if self.__dict__["forwarding"] is False:
			self.__dict__[name] = value
		elif name.startswith("__") and name.endswith("__"):
			self.__dict__[name] = value
		else:
			if name in self.__dict__:
				self.__dict__[name] = value
			else:
				if self.switch.decoupled:
					setattr(self.mmcLeg, name, value)
				else:
					setattr(self.realLeg, name, value)

#	def addControlVelocities(self, new_vel):
#		print("SWITCH VELOCITIES SEND BOTH WAYS")
#		self.realLeg.addControlVelocities(new_vel)
#		self.mmcLeg.addControlVelocities(new_vel)