from controller.reaCog.MotivationNetwork.MotivationUnit import MotivationUnit

##
#	DelayUnit
#
#	is a single MotivationUnit (that is a simple neuron) that delays an input signal.
# 	Firing will be delayed by the set delay.
##

class DelayUnit(MotivationUnit):

	def __init__(self, name, startValue=0, bias=0, group_name='', delay=5):
		MotivationUnit.__init__(self, name=name, startValue=startValue, bias=bias, group_name=group_name)
		self.delay = delay
		self.current_count = 0

		
	##	Output function (automatically called in processing of network).
	#	Restricts value range to 0<=output<=1.
	def applyOutputFunction(self):
		if self.output_value > 0.5:
			if self.current_count >= self.delay:
				self.output_value = 1
			else:
				self.output_value = 0
				self.current_count += 1
			#print("Active ", self.name, self.output_value, self.current_count)
			
		else:
			self.output_value = 0
			self.current_count = 0