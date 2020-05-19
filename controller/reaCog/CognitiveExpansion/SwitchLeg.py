##
# 	SwitchLeg
#
#	The Leg, WLeg structures handle sensory processing and sending of motor control 
#	signals. SwitchLeg wraps and hides these - it provides an interface to 
#	- either the leg control (routing calls to Leg and WLeg)
#	- or the internal leg models (when running decoupled in internal simulation)
#	This depends on the SwitchObject.
##  

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