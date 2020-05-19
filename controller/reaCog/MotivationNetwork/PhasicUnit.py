from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

##
#	PhasicUnit
#	Unit which has a timed behavior: is only staying active for 
#	a certain time (given as time_window) and switches afterwards off
#	until the input is turned off. Afterwards can be activated again.
##
class PhasicUnit(MotivationUnit):

	def __init__(self, name, startValue=0, bias=0, group_name='', time_window=100, threshold=0.5):
		MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
		self.time_window = time_window
		self.current_count = 0
		self.threshold = threshold
		
	##	Output function (automatically called in processing of network).
	#	Restricts value range to 0<=output<=1.
	def applyOutputFunction(self):
		if (self.output_value > self.threshold):
			if (self.current_count < self.time_window):
				self.current_count += 1
				self.output_value = min(1., self.output_value)
			else:
				if (self.current_count == self.time_window):
					self.current_count += 1
				self.output_value = 0
		else:
			self.output_value = 0
			self.current_count = 0