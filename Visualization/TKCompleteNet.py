'''
Created on 18.5.2013

@author: mschilling
'''
import math
from tkinter import *

from ProcessOrganisation.ProcessModule.ProcessingModule import ProcessingModule
from controller.reaCog.MotivationNetwork import MotivationUnit

list_all_mu_units = []

##
#	Update function for MotivationUnit activation visualization
def updateAllMUunitsActivationVisualization():
	for mu in list_all_mu_units:
		mu.updateActivation()

##
#	Visualization of an activation of a single MotivationUnit.
#	It is represented as a circle - the fill colour represents the activation
#	(getting green when activated)		
class TKMotivationUnitCircle():
	radius = 10
	
	def __init__(self, m_unit, canvas, x, y, text_to_left = False):
		list_all_mu_units.append(self)
		self.m_unit = m_unit
		self.name = m_unit.name
		self.activation = -1
		self.canvas = canvas
		self.canvas.create_oval((x-self.radius),(y-self.radius),(x+self.radius),(y+self.radius), outline="dark red", width=1.0, tag=self.name)
		if (text_to_left):
			self.canvas.create_text((x-(self.radius+32)), y, justify='right', text=self.name)
		else:
			self.canvas.create_text((x+self.radius+32), y, justify='left', text=self.name)
		
	def updateActivation(self):
		#print(self.name, self.m_unit.getActivation())
		if self.activation != self.m_unit.getActivation():
			self.activation = self.m_unit.getActivation()
			if self.activation > 0.5:
				self.canvas.itemconfig(self.name, fill='#090')
			else:
				self.canvas.itemconfig(self.name, fill='#fff')

##
# 	Visualization of the Motivation Network.
#	Draws a window using TKinter and shows selected Neurons of the MotivationNetwork.
#	The activation of those is color coded (white=0, green=1) and is updated 
#	in the post_processing_step.
#	
#	The visualization is a ProcessingModule which can be registered and run
#	by a central ProcessModuleExecution.
##
class TKCompleteNet(ProcessingModule):

	def __init__(self, name, motivationNetRobot):
		ProcessingModule.__init__(self, name)
		self.motivationNetRobot = motivationNetRobot
		# Given is a positioning scheme for the Motivation units
		# relatively ordered in one leg
		# This is extended to all the other legs (transformed accordingly)
		self.unit_pos_rel = [(100,40),(80,80), (120,80), (80,120), (120,120)]
		self.x_rel_spacing = 200
		self.y_rel_spacing = 160
		self.window = Tk()
		self.window.title("Motivation Net")
		self.canvas = Canvas(self.window, width = 2.5 * self.x_rel_spacing, height = 240+3 * self.y_rel_spacing)
#		self.tk_img = PhotoImage(file = 'MU_net_BG.gif')
#		self.canvas.create_image(304, 304, image=self.tk_img)
		self.canvas.pack()

## **** Graphic methods: Init window, draw and map neurons **************************
	##
	#	Initialising the drawing window.
	#	Must be called before the visualisation can be updated
	#	by calling draw_manipulator
	def init_module(self):
		cur_unit = 0
		self.MU_Circles = []
		
		self.unit_list = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		
		leg_pair_nr = 0
		for m_leg in self.motivationNetRobot.motivationNetLegs:
			self.unit_list[0+leg_pair_nr*5] = m_leg.legMU
			self.unit_list[1+leg_pair_nr*5] = m_leg.swing_motivation
			self.unit_list[2+leg_pair_nr*5] = m_leg.stance_motivation
			self.unit_list[3+leg_pair_nr*5] = m_leg.forward
			self.unit_list[4+leg_pair_nr*5] = m_leg.backward
			leg_pair_nr += 1
	
		self.unit_list[33] = self.motivationNetRobot.stand
		self.unit_list[30] = self.motivationNetRobot.walk
		self.unit_list[31] = self.motivationNetRobot.forward
		self.unit_list[32] = self.motivationNetRobot.backward
		
		# Leg MU are placed and connected
		self.draw_leg_units(self.unit_list[0:5], (0, 240), "FL LEG") 
		self.draw_leg_units(self.unit_list[5:10], (1.5*self.x_rel_spacing, 240), "FR LEG")
		self.draw_leg_units(self.unit_list[10:15], (0, 240 + self.y_rel_spacing), "ML LEG") 
		self.draw_leg_units(self.unit_list[15:20], (1.5*self.x_rel_spacing, 240+self.y_rel_spacing), "MR LEG")
		self.draw_leg_units(self.unit_list[20:25], (0, 240+2*self.y_rel_spacing), "HL LEG") 
		self.draw_leg_units(self.unit_list[25:30], (1.5*self.x_rel_spacing, 240+2*self.y_rel_spacing), "HR LEG") 
		# Draw activation outgoing of walk
		self.canvas.create_line( (self.unit_pos_rel[0][0] + 12), (self.unit_pos_rel[0][1] + 240),
			(self.unit_pos_rel[0][0] + 1.5*self.x_rel_spacing - 12), (self.unit_pos_rel[0][1] + 240),
			width=1, fill='dark red', arrow='both')
		self.canvas.create_line( (self.unit_pos_rel[0][0] + 12), (self.unit_pos_rel[0][1] + 240+ self.y_rel_spacing),
			(self.unit_pos_rel[0][0] + 1.5*self.x_rel_spacing - 12), (self.unit_pos_rel[0][1] + 240 + self.y_rel_spacing),
			width=1, fill='dark red', arrow='both')
		self.canvas.create_line( (self.unit_pos_rel[0][0] + 12), (self.unit_pos_rel[0][1] + 240+2*self.y_rel_spacing),
			(self.unit_pos_rel[0][0] + 1.5*self.x_rel_spacing - 12), (self.unit_pos_rel[0][1] + 240+2*self.y_rel_spacing),
			width=1, fill='dark red', arrow='both')
			
		# Initialise central units
		self.MU_Circles.append(TKMotivationUnitCircle(self.unit_list[30], self.canvas,
				(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]),
				(self.unit_pos_rel[0][1]), True))
		self.MU_Circles.append(TKMotivationUnitCircle(self.unit_list[31], self.canvas,
				(self.x_rel_spacing*0.75 + self.unit_pos_rel[1][0]),
				(self.unit_pos_rel[1][1]), True))
		self.MU_Circles.append(TKMotivationUnitCircle(self.unit_list[32], self.canvas,
				(self.x_rel_spacing*0.75 + self.unit_pos_rel[2][0]),
				(self.unit_pos_rel[2][1]), False))
		# Walk activates forward and backward
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0] - 4), (self.unit_pos_rel[0][1] + 10),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[1][0] + 4), (self.unit_pos_rel[1][1] - 10), 
			width=1, fill='dark red', arrow='both')
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0] + 4), (self.unit_pos_rel[0][1] + 10),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[2][0]-4),(self.unit_pos_rel[2][1]-10), 
			width=1, fill='dark red', arrow='both')
		# Inhibitory lines
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[1][0] + 13), (self.unit_pos_rel[1][1]),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[2][0]-13), (self.unit_pos_rel[2][1]), 
			width=1, fill='dark red')
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[1][0] + 13), (self.unit_pos_rel[1][1] -4),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[1][0] + 13), (self.unit_pos_rel[1][1] +4), 
			width=1, fill='dark red')
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[2][0] - 13), (self.unit_pos_rel[2][1] -4),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[2][0] - 13), (self.unit_pos_rel[2][1] +4), 
			width=1, fill='dark red')
		# Walk activates legs
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]), (self.unit_pos_rel[0][1] + 12),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]), (self.unit_pos_rel[1][1] -2), 
			width=1, fill='dark red', arrow='first')
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]), (self.unit_pos_rel[1][1] + 2),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]), (self.unit_pos_rel[0][1] + 240+2*self.y_rel_spacing ), 
			width=1, fill='dark red')
		# Stand unit and line
		self.MU_Circles.append(TKMotivationUnitCircle(self.unit_list[33], self.canvas,
				(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]*2),
				(self.unit_pos_rel[0][1]), False))
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0] + 13), (self.unit_pos_rel[0][1]),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]*2 - 13), (self.unit_pos_rel[0][1]), 
			width=1, fill='dark red')
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0] + 13), (self.unit_pos_rel[0][1] -4),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0] + 13), (self.unit_pos_rel[0][1] +4), 
			width=1, fill='dark red')
		self.canvas.create_line( (self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]*2 - 13), (self.unit_pos_rel[0][1] -4),
			(self.x_rel_spacing*0.75 + self.unit_pos_rel[0][0]*2 - 13), (self.unit_pos_rel[0][1] +4), 
			width=1, fill='dark red')
	##
	# 	Draw all motivation units of a single legMU
	#	@param units list of motivation units of a leg
	#	@param rel_pos offset where to draw the units of the leg
	#	@param name of the leg 
	def draw_leg_units(self, units, rel_pos, name):
		self.canvas.create_text((0.5*self.x_rel_spacing + rel_pos[0]), (rel_pos[1] + 10), text=name)
		for i in range(0, 5):
			self.MU_Circles.append(TKMotivationUnitCircle(units[i], self.canvas,
				(rel_pos[0] + self.unit_pos_rel[i][0]),
				(rel_pos[1] + self.unit_pos_rel[i][1]), ((i==0 and rel_pos[0]==0) or i==1 or i==3) ) )
		# Leg activates swing and stance
		self.canvas.create_line( (self.unit_pos_rel[0][0] + rel_pos[0] - 4), (self.unit_pos_rel[0][1] + rel_pos[1] + 10),
			(self.unit_pos_rel[1][0] + rel_pos[0]+4), (self.unit_pos_rel[1][1] + rel_pos[1] - 10), 
			width=1, fill='dark red', arrow='both')
		self.canvas.create_line( (self.unit_pos_rel[0][0] + rel_pos[0] + 4), (self.unit_pos_rel[0][1] + rel_pos[1] + 10),
			(self.unit_pos_rel[2][0] + rel_pos[0]-4), (self.unit_pos_rel[2][1] + rel_pos[1] - 10), 
			width=1, fill='dark red', arrow='both')
		# Inhibitory lines
		self.canvas.create_line( (self.unit_pos_rel[1][0] + rel_pos[0] + 13), (self.unit_pos_rel[1][1] + rel_pos[1]),
			(self.unit_pos_rel[2][0] + rel_pos[0]-13), (self.unit_pos_rel[2][1] + rel_pos[1]), 
			width=1, fill='dark red')
		self.canvas.create_line( (self.unit_pos_rel[1][0] + rel_pos[0] + 13), (self.unit_pos_rel[1][1] + rel_pos[1]-4),
			(self.unit_pos_rel[1][0] + rel_pos[0]+13), (self.unit_pos_rel[1][1] + rel_pos[1]+4), 
			width=1, fill='dark red')
		self.canvas.create_line( (self.unit_pos_rel[2][0] + rel_pos[0] - 13), (self.unit_pos_rel[2][1] + rel_pos[1]-4),
			(self.unit_pos_rel[2][0] + rel_pos[0]-13), (self.unit_pos_rel[2][1] + rel_pos[1]+4), 
			width=1, fill='dark red')
			
	##
	# 	The draw method for the manipulator leg.
	#	It is called from the outside iteration loop.
	def post_processing_step(self, args=None):
		updateAllMUunitsActivationVisualization()
		self.canvas.update()