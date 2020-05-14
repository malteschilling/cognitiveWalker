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
#		if (self.name == "pD_MMC_unstableHL_delay"):
#			print("DELAY = ", self.current_count, self.output_value)
		if (self.output_value > self.threshold):
			if (self.current_count < self.time_window):
				self.current_count += 1
				self.output_value = min(1., self.output_value)
			else:
				if (self.current_count == self.time_window):
					#print("ACCELERATE ", self.name)
					self.current_count += 1
				self.output_value = 0
		else:
			self.output_value = 0
			self.current_count = 0
	
#		if (0 < self.current_count <= self.time_window):
#			self.current_count += 1
#			self.output_value = 1
#			print("Slowed down", self.current_count)
#		elif (self.output_value > 0.5) and (self.current_count == 0):
#			print("Start slow down", self.current_count)
#			self.current_count += 1
#		else:
#			if self.current_count>0:
#				print("Accelerate again")
#			self.current_count = 0
#			self.output_value = 0
