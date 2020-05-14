from controller.reaCog.MotivationNetwork.ModulatedRoutine import ModulatedRoutine
import numpy
import controller.reaCog.WalknetSettings as WSTATIC

##
# 	Biological inspired swing net
#	(see Cruse and Schumm publication).
# 	!HAS TO BE ADAPTED FOR SLOW VELOCITY TRAJECTORIES!
class SwingNet3b(object):

	## Initialisation of the SwingNet
	def __init__(self, leg , extr_pos):
		ModulatedRoutine.__init__(self)
		self.leg_extreme_positions = extr_pos
		self.leg = leg
		self.jointPointer = numpy.zeros(3)
		self.trajectory = None
		self.iteration =0
		self.targetPoint = numpy.zeros(3)
		self.__targetNet = []
		self.maxIteration = 1
		self.alphaaux = 0.0
		self.alphaout = 0
		self.betaaux = 0.0
		self.betaout = 0
		self.gammaaux = 0.0
		self.gammaout = 0
		self.betaHpout = 0
		self.betaHpaux = 0.0


	def __del__(self):
		pass


	##	Function called by the ModulatingMotivationUnit when active.
	#	Invokes the execution of the routine which has to be defined by the derived 
	#	classes.
	def modulatedRoutineFunctionCall(self, weight):
		if weight > 0.5:
			angles = self.leg.getTargetAngles_OLD_CS()
			#if (self.leg.name == "mhr") :
			#	print("SWING ", self.leg.name, " / ", angles)
			self.trajectory = numpy.zeros(3)
			self.targetPoint = self.leg_extreme_positions.swingTarget
			#self.targetPoint[0]+= self.leg.aep_shifted
			if len(self.targetPoint) == 3 :
				self.targetPoint = numpy.append(self.targetPoint,0)
		#print(self.targetPoint)
			winkel = self.leg.inverseKinematics_OLD_CS(numpy.dot(self.leg._phi_psi_trans_inv,self.targetPoint))
#			print(self.leg.name, " WINKEL: ", winkel, " - ", self.leg.computeInverseKinematics(self.targetPoint))
#			print("POS: ", self.targetPoint, " - ", self.leg.computeForwardKinematics(self.leg.computeInverseKinematics(self.targetPoint)))

			## === alpha ===
			timecon = 3.0
			velos =  10. #9. # 0.1 # 9
			#if robot.mutivationNet.backward > 0.5:
			if WSTATIC.default_speed < 0.:
				self.alphaaux = (winkel[0] - self.leg.swingNet_pepShiftX - angles[0]) - self.alphaout # for backward walking
			else:
				self.alphaaux = (winkel[0] - self.leg.swingNet_aepShiftX - angles[0]) - self.alphaout # for forward walking
			self.alphaout += (self.alphaaux / timecon)
			self.trajectory[0] = self.alphaout * velos
			
		# 	timecon = 3.0
# 			velos = 9
# 
# 			if RSTATIC.default_speed < 0.:
# 				self.alphaaux = (winkel[0] - self.leg.swingNet_pepShiftX - angles[0]) - self.alphaout # for backward walking
# 			else:
# 				self.alphaaux = (winkel[0] - self.leg.swingNet_aepShiftX - angles[0]) - self.alphaout # for forward walking
# 			self.alphaout += (self.alphaaux / timecon)
# 			self.trajectory[0] = self.alphaout * velos

			## === Beta ===
		
			self.betaHpaux += (self.betaHpout / self.leg.swingNet_hpTimeConst)
			self.betaHpout = (weight - self.betaHpaux)
			if self.betaHpout < 0.: self.betaHpout = 0.  # rectifier
			timeconTP = 3. #3
			velosTP = 3.# 1.7
			#if robot.mutivationNet.backward > 0.5:
			if WSTATIC.default_speed < 0.:
				self.betaaux = ((winkel[1] - self.leg.swingNet_pepShiftZ - angles[1] ) ) - self.betaout # for backward walking
			else:
				self.betaaux = ((winkel[1] - self.leg.swingNet_aepShiftZ - angles[1] ) ) - self.betaout # for forward walking
			self.betaout += (self.betaaux / timeconTP)
			self.trajectory[1] = (self.betaout * velosTP) + (self.betaHpout * self.leg.swingNet_hpVelocity)

			# ## === Gamma === ##
# 			timecon = 3.0
# 			velos = 3.0
# 			self.gammaaux = (winkel[2] - angles[2]) - self.gammaout
# 			self.gammaout += (self.gammaaux / timecon)
# 			self.trajectory[2] = self.gammaout * velos
# 
# 			## === Beta ===
# 			timeconHP = 3
# 			self.betaHpaux += (self.betaHpout / timecon)
# 			self.betaHpout = (weight - self.betaHpaux)
# 			#print("hochpass Beta", self.betaHpout)
# 
# 			velosHP= 50
# 			velosTP = 1.7
# 			timeconTP = 3
# 			self.betaaux = ((winkel[1] - 0.9 - angles[1]) + (self.betaHpout * velosHP)) - self.betaout
# 			self.betaout += (self.betaaux / timecon)
# 			self.trajectory[1] = (self.betaout * velosTP)

			## === Gamma === ##
			timecon = 3.0
			velos = 3. # 0.5 # 3?
			self.gammaaux = (winkel[2] - angles[2] ) - self.gammaout
			self.gammaout += (self.gammaaux / timecon)
			self.trajectory[2] = self.gammaout * velos  + (self.betaHpout * self.leg.swingNet_hpGammaVelos) * 0.1 #separate parameter for gamma and leg needed 0.05  #0.2 # ML, HL

			for j in range(3):
				if (self.trajectory[j] > 6.2):
					self.trajectory[j] = 6.2

			self.leg.addControlVelocities(self.trajectory)
			#if (self.leg.name == "mml"):
#			print("** MU SWING 3b ", self.leg.name, ": ", self.trajectory, "**** ", weight)

		if weight == 0 :
			self.betaaux = 0.
			self.betaout = 0.
			self.betaHpout = 0.
			self.betaHpaux = 0.
			self.gammaaux = 0.
			self.gammaout = 0.
			self.alphaaux = 0.
			self.alphaout = 0.
