'''
Created on 3.12.2011

@author: mschilling
'''

""" 
	Wrapper for the joint angles of the internal body model
	(in the leg model).
	Using the wrapper the joints of the leg model follow the interface of
	the robot joint and can be accessed in the exactly same way.
	This is used in planning ahead when during mental simulation
	the model joints are used instead of the real ones.
"""	
class wrapperJointAngle (object):

	def __init__(self, val, logging=False):
		self.angle = [val]
		
	def getInputPositionForIterationStep(self, iteration) :
		return self.angle[iteration]
		
	def addNewInputPosition(new_angle) :
		self.angle.append(new_angle)	
		
	def setInputPosition(self, new_angle) :
		self.angle[-1] = new_angle
		
	def getInputPosition(self) :
		return self.angle[-1]