#!/usr/bin/env python

import numpy, math
import Hector.RobotSettings as RSTATIC
import controller.reaCog.WalknetSettings as WSTATIC

##
#	A Body Model for a hexapod walker, based on MMC computation.
#	Now extended to 3 dimensions, (body segments are 3D and have now an orientation).
#
#	Higher level of the hierarchical body model for a six legged robot:
#	On this level
#		- there are three segments described through each six vectors
#		- each leg is represented only as a vector to the tip of the leg
#			for each leg there are two vectors - one starting at the front of the segment
#			(the leg vector) and one starting at the back of the segment (diag vector)
#			both ending at the tip of the leg
#	The body model is constructed as a Mean of Multiple Computation network:
#		- each variable vector is described by multiple kinematic relations
#			= computations
#		- these different computations are integrated using a mean calculation
#	The resulting set of equations directly defines a neural network connection matrix.
#	This can be computed in an iterative fashion - introduction of a recurrent connection
#	damps the overall relaxation procedure and prevents oscillations.
#	The network is able to compute forward, inverse and any mixed kinematic problem and
#	is quite fast (it only needs a few iteration steps).
#	It acts as an autoassociator - so a problem is given to the network as an incomplete
#	set of variables and the network fills in complementing missing values.
#	The network used here is special as it uses no specific reference coordinate system.
#	Described in the network are only relative vectors between certain body parts.
#	The footdiag vectors connecting the feet of the standing legs are actually
#	constituting a fixed reference frame in which coordinate systems and
#	representation of the environment can be grounded.
#	To move the body actually, there are special vectors in the body model which
#	explicitly represent a disturbance of the dynamics of the network (the pull vector
#	which is driving the delta vectors of the segments). One can envision this as
#	a passive motion paradigm: the body model is pulled into direction of this vector
#	and the model is following this movement (one can think of this as a stick model which
#	legs are attached to the ground).
#
#	The body model is used in connection to leg models. The leg vectors are enforced onto
#	the lower leg networks which compute corresponding joint angles for the legs from
#	this. Overall this is embedded in a processing loop:
#		- the leg network is updated by sensory data
#			(iterates once to integrate the values)
#		- the leg network updates the body model
#		- the body model gets a movement command (usually constant)
#			and is iterated (one step is sufficient)
#		- the body model pushes down the new leg vectors for the standing legs into
#			the leg networks (these are iterated for a few iteration steps and provide
#			joint angles)
#		- the joint motors are controlled by the new motor commands
##

class mmcBodyModelStance:

	###	Initialisation of the body model.
	#	The segment vectors are encoded here, the leg vectors are initialised here
	#	but are actually set when a leg is really put on the ground (initially all are
	#	assumed in the air, so an update of the legs is forced in the first iteration)
	def __init__(self, motiv_net, stab_thr):
		self.motivationNetRobot = motiv_net
		# The stability is determined in the following way:
		# The connection between the two most hind legs touching the ground
		# (one leg on each side) is constructing a vector to the back.
		# The segment_factor is the fraction of the last body segment meeting this
		# vector (counted from in between middle and hind segment):
		# 	0 = between the two segments (0 part of the hind segment)
		#	-1 = at the end of the hind segment_factor
		# The robot is determined instable when the segment_factor is greater
		# than this threshold.
		self.stability_threshold = stab_thr

		# Used for storing stability calculation in visualization
		self.temp_stability_fact = 0.
		
		# The height is currently set to a fixed value
		# Here a height net might be introduced?
		# The body model is now always enforcing this specific height.
		self.height = WSTATIC.stanceheight
		self.width = WSTATIC.stancewidth
				
		# Additional Vectors (from front of a segment to footpoint)
		self.front_vect = [numpy.array([-0.19, (self.width + 0.07),-self.height]),numpy.array([-0.19, -(self.width + 0.07),-self.height]),
					numpy.array([-0.33,(self.width + 0.07),-self.height]),numpy.array([-0.33,-(self.width + 0.07),-self.height]),
					numpy.array([-0.2, (self.width + 0.07),-self.height]),numpy.array([-0.2, -(self.width + 0.07),-self.height])]
		# Segment defining vectors: is constructed as a diamond for the mounting points of
		# the legs: segm_leg_ant = from coxa to front (anterior)
		# 			segm_leg_post = from coxa to back (posterior)
		self.segm_post_ant = [numpy.array([0.22,0.0,0.0]),numpy.array([0.35, 0.0, 0.0]),numpy.array([0.36,0.0,0.0])]
		self.segm_post_ant_norm = [numpy.linalg.norm( segm_vect ) for segm_vect in self.segm_post_ant]			
		
		# Real Leg vectors 
		self.leg_vect = [numpy.array([ 0.  ,  self.width, -self.height]), numpy.array([ 0.  , -self.width, -self.height]), 
				numpy.array([ 0.  ,  self.width, -self.height]), numpy.array([ 0.  , -self.width, -self.height]), 
				numpy.array([ 0.  ,  self.width, -self.height]), numpy.array([ 0.  , -self.width, -self.height])]
	
		self.segm_leg_ant = [(self.leg_vect[i] - self.front_vect[i]) for i in range(0,6)]
		#self.segm_leg_ant = [(self.leg_vect[0] - self.front_vect[0]), (self.leg_vect[1] - self.front_vect[1])] 
		self.segm_leg_ant_norm = [numpy.linalg.norm( segm_vect ) for segm_vect in self.segm_leg_ant]
		#self.segm_leg_post = [(self.segm_leg_ant[0] - self.segm_post_ant[0]), (self.segm_leg_ant[1] - self.segm_post_ant[0])]
		self.segm_leg_post = [(self.segm_leg_ant[i] - self.segm_post_ant[i//2]) for i in range(0,6)]
		self.segm_leg_post_norm = [numpy.linalg.norm( segm_vect )  for segm_vect in self.segm_leg_post]
		self.segm_diag_to_right = [ (self.segm_leg_ant[i*2] - self.segm_leg_ant[i*2 + 1]) for i in range(0, len(self.segm_leg_ant)//2)]
		self.segm_diag_norm = [numpy.linalg.norm( segm_vect )  for segm_vect in self.segm_diag_to_right]
		
		# Help vectors for the drawing routines: We have to keep track of the feet
		# positions in a global coordinate system (and of one segment, too)
		self.segm1_in_global = numpy.array([-0.02,0.0,self.height])
		self.foot_global = [(self.segm1_in_global + self.segm_post_ant[1] + self.segm_post_ant[0] + self.front_vect[0]),
			(self.segm1_in_global + self.segm_post_ant[1] + self.segm_post_ant[0] + self.front_vect[1]),
			(self.segm1_in_global + self.segm_post_ant[1] + self.front_vect[2]),
			(self.segm1_in_global + self.segm_post_ant[1] + self.front_vect[3]),
			(self.segm1_in_global + self.front_vect[4]),
			(self.segm1_in_global + self.front_vect[5])]

		# Ground contact - which feet are on the ground
		self.gc = [False, False, False, False, False, False]
		self.old_stance_motivation = [False, False, False, False, False, False]
		
		# Vectors between footpoints - these are a fixed coordinate system which
		# shall not be altered. The standing feet are connected through the ground
		# and their relation shall be constant.
		# The footdiags table has to be adapted whenever the configuration of the walker
		# changes - when a leg is lifted from the ground this leg vector is not forming
		# closed kinematic chains with the other legs anymore and the footdiags can
		# not be exploited anymore.
		# When a leg is touching the ground, new footdiags to the other standing
		# legs have to be established.
		self.footdiag = [ [], [0], [0,0], [0,0,0], [0,0,0,0], [0,0,0,0,0] ]
		for i in range(0, len(self.footdiag) ) :
			for j in range(0,i) :
				self.footdiag[i][j] = self.set_up_foot_diag(i,j)
		# The explicit disturbance vectors of the network
		self.delta_front = [numpy.array([0,0,0]), numpy.array([0,0,0]), numpy.array([0,0,0])]
		self.delta_back = [[numpy.array([0,0,0])], [numpy.array([0,0,0])], [numpy.array([0,0,0])]]
		# These are operated through a pull at the front
		self.pull_front = numpy.array([0.0, 0.00,0.0])
		self.pull_back = numpy.array([0.0,0.0,0.0])
		self.step = 0
		self.damping = 5
		
	""" **** Set up methods and calculation of vector methods ***************************
	"""
	##	Calculation of the vector connection the feet of two standing legs.
	#	When a leg is put on the ground a new connecting vector has to be established.
	#	This is calculated as the difference between the leg vectors.
	def set_up_foot_diag(self, start, target) :
		diag_vec = self.leg_vect[target] - self.leg_vect[start] + \
			self.get_segm_vectors_between_legs(start, target)
		return diag_vec

	##	Providing the segment vectors connecting two legs.
	#	An equation is described by a series of vectors forming a closed kinematic
	#	chain (two leg vectors, the footdiag and -possibly- segment vectors in
	#	between). This function is calculating the segment vectors between the
	#	given legs.
	def get_segm_vectors_between_legs(self, start_leg, end_leg):
		leg_diff = (end_leg//2-start_leg//2)
		if leg_diff == 0 :
			return ( self.segm_leg_post[start_leg] - self.segm_leg_post[end_leg] )
		elif leg_diff == 1:
			return ( self.segm_leg_post[start_leg] - self.segm_leg_ant[end_leg] )
		elif leg_diff == -1:
			return ( self.segm_leg_ant[start_leg] - self.segm_leg_post[end_leg] )
		elif leg_diff == 2 :
			return ( self.segm_leg_post[start_leg] - self.segm_post_ant[1] - self.segm_leg_ant[end_leg] )
		elif leg_diff == -2:
			return ( self.segm_leg_ant[start_leg] + self.segm_post_ant[1] - self.segm_leg_post[end_leg] )

	##	Providing the segment vectors connecting two front (the additional) vectors.
	#	An equation is described by a series of vectors forming a closed kinematic
	#	chain (two front vectors, the footdiag and -possibly- segment vectors in
	#	between). This function is calculating the segment vectors between the
	#	given legs.
	def get_segm_vectors_between_front(self, start_leg, end_leg):
		leg_diff = (end_leg//2-start_leg//2)
		if leg_diff == 0 :
			return numpy.array([0., 0., 0.])
		elif abs(leg_diff) == 1 :
			return numpy.array(-1 * leg_diff * self.segm_post_ant[min(start_leg, end_leg)//2])
		elif abs(leg_diff) == 2 :
			return ( -0.5 * leg_diff * (self.segm_post_ant[0]
				+ self.segm_post_ant[1]))
				
	##	Sets ground contact to false and removes this leg from the body
	#	model computations. As the leg is not part of the closed kinematic chains
	#	after being lifted from the ground it shall not participate.
	def lift_leg_from_ground(self, leg_nr) :
		if (self.gc[leg_nr]) :
			self.gc[leg_nr] = False
			
	##	Sets the ground contact of the leg and initiates the calculation of connection
	#	vectors to all other standing legs (footdiag) which are used by the
	#	network.
	def put_leg_on_ground(self, leg_name, leg_vec) :
		leg_nr = RSTATIC.leg_names.index(leg_name)
		i = 0
		if (self.gc[leg_nr] == False) :
			# Set leg and diag vector
			self.leg_vect[leg_nr] = numpy.array(leg_vec)
			self.front_vect[leg_nr] = self.leg_vect[leg_nr] - self.segm_leg_ant[leg_nr]
			# Construction of all foot vectors - the ones to legs in the air are not used!
			for i in range(0,leg_nr) :
				self.footdiag[leg_nr][i] = self.set_up_foot_diag(leg_nr, i)
			for i in range(leg_nr+1,6):
				self.footdiag[i][leg_nr] = self.set_up_foot_diag(i, leg_nr)
			# Derive the global position (needed for the graphics output)
			for i in range(0,6):
				if (self.gc[i]):
					if (i > leg_nr):
						self.foot_global[leg_nr] = self.foot_global[i] + self.footdiag[i][leg_nr]
					else:
						self.foot_global[leg_nr] = self.foot_global[i] - self.footdiag[leg_nr][i]
					break
			self.gc[leg_nr] = True
			
	##	Update the current state of the legs.
	#	Only legs in stance mode are part of the body model (which is used for computation
	#	of the stance movement). When a leg switches between states it has to be added
	#	or removed from the body model.
	def updateLegStates(self):
		for motiv_leg, leg_nr in zip(self.motivationNetRobot.motivationNetLegs, range(len(self.motivationNetRobot.motivationNetLegs))):
			if (motiv_leg.inSwingPhase()):
				if (self.old_stance_motivation[leg_nr]):
					self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(leg_nr)
					#print(motiv_leg.wleg.leg.name, " starts swing")
				self.old_stance_motivation[leg_nr] = False
			else:
				self.old_stance_motivation[leg_nr] = True

	""" **** Graphic methods: Simple drawing of the body model **************************
	"""
	##	Extract the global positions of the feet, the segments, diagonals ...
	def get_leg_triangle(self, leg):
		return([ [ self.foot_global[leg][0], \
			(self.foot_global[leg][0] - self.leg_vect[leg][0]), \
			(self.foot_global[leg][0] - self.leg_vect[leg][0] + self.segm_leg_post[leg][0]), \
			(self.foot_global[leg][0] - self.leg_vect[leg][0] + self.segm_leg_post[leg][0] + self.segm_post_ant[leg//2][0]), \
			(self.foot_global[leg][0] - self.leg_vect[leg][0])], \
			[ self.foot_global[leg][1], \
			(self.foot_global[leg][1] - self.leg_vect[leg][1]), \
			(self.foot_global[leg][1] - self.leg_vect[leg][1] + self.segm_leg_post[leg][1]), \
			(self.foot_global[leg][1] - self.leg_vect[leg][1] + self.segm_leg_post[leg][1] + self.segm_post_ant[leg//2][1]), \
			(self.foot_global[leg][1] - self.leg_vect[leg][1])], \
			[ self.foot_global[leg][2], \
			(self.foot_global[leg][2] - self.leg_vect[leg][2]), \
			(self.foot_global[leg][2] - self.leg_vect[leg][2] + self.segm_leg_post[leg][2]), \
			(self.foot_global[leg][2] - self.leg_vect[leg][2] + self.segm_leg_post[leg][2] + self.segm_post_ant[leg//2][2]), \
			(self.foot_global[leg][2] - self.leg_vect[leg][2])] ])
			
	""" **** Computation of the MMC equations *******************************************
	"""
	##	Compute the leg vectors: For all standing legs
	#	the new leg vectors are computed, summed and the mean is calculated
	#	(the old value is also integrated, weighted by the damping value)
	def compute_leg_computations_and_integrate(self, leg_nr):
		equation_counter = 1
#B		segm_leg_vect = self.segm_leg_ant[leg_nr] + self.front_vect[leg_nr]
		segm_leg_vect = -self.delta_back[leg_nr//2] + self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr//2] + self.front_vect[leg_nr]
		segm_leg_vect += self.damping * self.leg_vect[leg_nr]
		equation_counter += self.damping
		for target_leg in range(0, len(self.gc)):
			if self.gc[target_leg] and target_leg != leg_nr:
				part_vec = numpy.array(self.leg_vect[target_leg]) \
					+ self.get_segm_vectors_between_legs( leg_nr, target_leg)
				if target_leg < leg_nr:
					part_vec -= self.footdiag[leg_nr][target_leg]
				elif target_leg > leg_nr:
					part_vec += self.footdiag[target_leg][leg_nr]
				segm_leg_vect += part_vec
				equation_counter += 1
		return (segm_leg_vect/equation_counter)

	##	Compute the front vectors: For all standing legs
	#	the new front vectors are computed, summed and the mean is calculated
	#	(the old value is also integrated, weighted by the damping value)
	def compute_front_computations_and_integrate(self, leg_nr):
		equation_counter = 1
		new_front_vect = -self.delta_front[leg_nr//2] - self.segm_post_ant[leg_nr//2] + self.leg_vect[leg_nr] - self.segm_leg_post[leg_nr]
		new_front_vect += self.damping * self.front_vect[leg_nr] #NEW
		equation_counter += self.damping #NEW
		# Computations of connections between legs in ground contacts
		# Compute equations to all other standing legs using the footdiag - has been in compute_leg!
		for target_leg in range(0, len(self.gc)):
			if self.gc[target_leg] and target_leg != leg_nr:
				part_vec = numpy.array(self.front_vect[target_leg]) \
					+ self.get_segm_vectors_between_front( leg_nr, target_leg)
				if target_leg < leg_nr:
					part_vec -= self.footdiag[leg_nr][target_leg]
				elif target_leg > leg_nr:
					part_vec += self.footdiag[target_leg][leg_nr]
				new_front_vect += part_vec
				equation_counter += 1
		return (new_front_vect/equation_counter)
		
	##	Compute the segment vectors:
	#	Using equations including the two legs connected to the segment,
	#	integrating the explicit displacement given as delta
	#	and the recurrent old value of the vector.
	def compute_segment_leg_ant_computations_and_integrate(self, leg_nr):
		equation_counter = 1
		new_segm_leg_ant = self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr//2]
		# Neighboring leg with respect to leg_nr
#		leg_neighbor = leg_nr + (1 - 2*(leg_nr%2))
#		new_segm_leg_ant += ((-1)**leg_nr) * self.segm_diag_to_right[0] + \
#			self.segm_leg_post[leg_neighbor] + self.segm_post_ant[leg_nr//2]
#		equation_counter += 1
		new_segm_leg_ant += self.segm_leg_ant[leg_nr] * self.damping
		equation_counter += self.damping
		new_segm_leg_ant = new_segm_leg_ant/equation_counter
		return ((self.segm_leg_ant_norm[leg_nr]/numpy.linalg.norm(new_segm_leg_ant))*new_segm_leg_ant)

	##	Compute the segment vectors:
	#	Using equations including the two legs connected to the segment,
	#	integrating the explicit displacement given as delta
	#	and the recurrent old value of the vector.		
	def compute_segment_leg_post_computations_and_integrate(self, leg_nr):
		equation_counter = 1
		new_segm_leg_post = self.segm_leg_ant[leg_nr] - self.segm_post_ant[leg_nr//2]
#		leg_neighbor = leg_nr + (1 - 2*(leg_nr%2))
#		new_segm_leg_post += ((-1)**leg_nr) * self.segm_diag_to_right[0] + \
#			self.segm_leg_ant[leg_neighbor] - self.segm_post_ant[leg_nr//2]
#		equation_counter += 1
		new_segm_leg_post += self.segm_leg_post[leg_nr] * self.damping
		equation_counter += self.damping
		new_segm_leg_post = new_segm_leg_post/equation_counter
		return ((self.segm_leg_post_norm[leg_nr]/numpy.linalg.norm(new_segm_leg_post))*new_segm_leg_post)

	##	Compute the segment vectors:
	#	Using equations including the two legs connected to the segment,
	#	integrating the explicit displacement given as delta
	#	and the recurrent old value of the vector.		
	def compute_segm_post_ant_computations_and_integrate(self, seg_nr):
		equation_counter = 3
		new_segm_post_ant = self.segm_post_ant[seg_nr] + self.delta_front[seg_nr] - self.delta_back[seg_nr]
		new_segm_post_ant += -self.segm_leg_post[2*seg_nr] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[1+seg_nr*2]
		new_segm_post_ant += -self.segm_leg_post[1 + 2*seg_nr] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[seg_nr*2]
		
		new_segm_post_ant += self.damping * self.segm_post_ant[seg_nr]
		equation_counter += self.damping

		new_segm_div = new_segm_post_ant/equation_counter
		return ((self.segm_post_ant_norm[seg_nr]/numpy.linalg.norm(new_segm_div))*new_segm_div)

	##	Compute the segment vectors:
	#	Using equations including the two legs connected to the segment,
	#	integrating the explicit displacement given as delta
	#	and the recurrent old value of the vector.		
	def compute_segm_diag_computations_and_integrate(self, seg_nr):
		equation_counter = 2
		
		new_segm_diag = self.segm_leg_post[2*seg_nr] + self.segm_post_ant[seg_nr] - self.segm_leg_ant[1+seg_nr*2]
		new_segm_diag -= self.segm_leg_post[1+2*seg_nr] + self.segm_post_ant[seg_nr] - self.segm_leg_ant[seg_nr*2]
		
		new_segm_diag += self.damping * self.segm_diag_to_right[seg_nr]
		equation_counter += self.damping
		new_segm_diag = new_segm_diag/equation_counter
		return ((self.segm_diag_norm[seg_nr]/numpy.linalg.norm(new_segm_diag))*new_segm_diag)

	##	The MMC Method:
	#	- the multiple computations are computed for each variable
	#	- the mean for each variable is calculated
	#	The new values are appended to the list of element values.
	#	For each variable new values are calculated through
	#	different equations.
	##			
	def mmc_iteration_step(self):
		self.delta_front[0] = self.pull_front
		self.delta_front[1] = (self.leg_vect[0] - self.segm_leg_post[0] - self.front_vect[0] - self.segm_post_ant[0])
		self.delta_front[2] = (self.leg_vect[2] - self.segm_leg_post[2] - self.front_vect[2] - self.segm_post_ant[1])

		self.delta_back[2] = self.pull_back
		self.delta_back[1] = -(self.leg_vect[4] - self.segm_leg_post[4] - self.front_vect[4] - self.segm_post_ant[2])
		self.delta_back[0] = -(self.leg_vect[2] - self.segm_leg_post[2] - self.front_vect[2] - self.segm_post_ant[1])

		front_vect = [self.compute_front_computations_and_integrate(i) for i in range(0,6)]
		leg_vect = [self.compute_leg_computations_and_integrate(i) for i in range(0,6)]
		segm_leg_ant = [self.compute_segment_leg_ant_computations_and_integrate(i) for i in range(0,6)]
		segm_leg_post = [self.compute_segment_leg_post_computations_and_integrate(i) for i in range(0,6)]
		segm_post_ant = [self.compute_segm_post_ant_computations_and_integrate(i) for i in range(0,3)]
		# To make the whole body stiff (one body segment) simply align the three body segments.
		#segm_post_ant[1] = ((self.segm_post_ant_norm[1]/numpy.linalg.norm(segm_post_ant[0]))*segm_post_ant[0])
		#segm_post_ant[2] = ((self.segm_post_ant_norm[2]/numpy.linalg.norm(segm_post_ant[0]))*segm_post_ant[0])
		segm_diag_to_right = [self.compute_segm_diag_computations_and_integrate(i) for i in range(0,3)]
		for i in range(0, 6):
			self.segm_leg_ant[i] = segm_leg_ant[i]
			self.segm_leg_post[i] = segm_leg_post[i]
			self.front_vect[i] = front_vect[i]
			self.leg_vect[i] = leg_vect[i]
		for i in range(0, 3):
			self.segm_diag_to_right[i] = segm_diag_to_right[i]
			self.segm_post_ant[i] = segm_post_ant[i]
		self.step += 1

	""" **** Get, set methods - connection to the robot simulator ***********************
	"""
	##	Pull the body model into a direction relative to the current first
	#	segment. Takes an angle (0 os straight ahead) and a velocity factor
	#	(around 0.1-0.2 should be fine) to come up with a corresponding pull vector.
	def pullBodyModelAtFrontIntoRelativeDirection(self, pull_angle, speed_fact):
		pull_angle_BM = pull_angle + math.atan2( self.segm_post_ant[0][1], self.segm_post_ant[0][0])
		self.pull_front[0] = speed_fact * math.cos(pull_angle_BM) # pull x
		self.pull_front[1] = speed_fact * math.sin(pull_angle_BM) # pull y

	##	Pull the body model into a direction relative to the last body
	#	segment. Takes an angle (0 means straight backwards) and a velocity factor
	#	(around 0.1 - positive means backwards walking!)
	#	to come up with a corresponding pull vector.
	def pullBodyModelAtBackIntoRelativeDirection(self, pull_angle, speed_fact):
		pull_angle_BM = pull_angle + math.atan2( -self.segm_post_ant[2][1], -self.segm_post_ant[2][0])
		self.pull_back[0] = speed_fact * math.cos(pull_angle_BM) # pull x
		self.pull_back[1] = speed_fact * math.sin(pull_angle_BM) # pull y

	##	Get the angles between the inner segments which are used for the
	#	segment joints.
	def get_segment_angles(self):
		front_angle = math.atan2(self.segm_post_ant[0][1],self.segm_post_ant[0][0])
		middle_angle = math.atan2(self.segm_post_ant[1][1],self.segm_post_ant[1][0])
		back_angle = math.atan2(self.segm_post_ant[2][1],self.segm_post_ant[2][0])
		angle1=front_angle - middle_angle
		if angle1<-3.14:
			angle1+=2*math.pi
		elif angle1>3.14:
			angle1-=2*math.pi
		angle2=middle_angle - back_angle
		if angle2<-3.14:
			angle2+=2*math.pi
		elif angle2>3.14:
			angle2-=2*math.pi
		return ([angle1,angle2])
		
	def get_leg_vector(self, leg_name):
		leg_nr = RSTATIC.leg_names.index(leg_name)
		target_vec_wn = [self.leg_vect[leg_nr][0], self.leg_vect[leg_nr][1], self.leg_vect[leg_nr][2], 0]
		return target_vec_wn
		
	def get_ground_contact(self, leg_nr):
		return self.gc[leg_nr]

	##
	#	Check if the BM configuration is static stable.
	#	In this version two line equations are used (in parametric version)
	#		- diagonal between left and right most hind leg with gc
	#		- line along the middle segment
	#	It is calculated where those two lines intersect, i.e. with respect to the 
	#	middle segment: the segment factor specifies this intersection as expressed
	#	in the line equation of this line: a negative value means that this 
	#	point is behind the middle segment (= the diagonal between the most hind legs
	#	lies behind the center of gravity). For a positive value the factor describes
	#	how CoG and the hind line of the polygon of static stability relate.
	def check_static_stability_along_segment(self):
		stability = True
		
		# Used for storing stability calculation in visualization
		self.temp_stability_fact = 0.5
		
#		print("STABILITY")
		left_leg, right_leg = 4, 5
		
		# Test if CoG moves moves behind the connecting line
		# connecting the leg on each side which
		# - has gc
		# - is the leg furthest backward having gc on that side
		# inSwingPhase(self)
		while left_leg>=0 and (getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).inSwingPhase():
			left_leg -= 2
		while right_leg>0 and (getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).inSwingPhase():
			right_leg -= 2
		# If there is no gc at all on one side it should be unstable
		if (left_leg < 0) or (right_leg < 0):
#			print("INSTABLE")
			stability = False
		else:
			#left_leg_obj = getattr(self.wRobot, RSTATIC.leg_names[left_leg])
			#right_leg_obj = getattr(self.wRobot, RSTATIC.leg_names[right_leg])
			#print(left_leg, right_leg, self.motivationNet.wrobot.hind_right_leg.predictedGroundContact(), self.motivationNet.hind_right_leg.inStancePhase() )
			
			#print(left_leg_obj.input_foot_position, right_leg_obj.input_foot_position)
			#print(-self.mmcStanceModel.front_vect[left_leg][-1], self.mmcStanceModel.front_vect[right_leg][-1])
			diag_vect = -self.front_vect[left_leg] + self.front_vect[right_leg] \
						+ self.get_segm_vectors_between_front(left_leg, right_leg)
			left_foot_cog_vect = -self.front_vect[left_leg]
			if (left_leg == 2):
				left_foot_cog_vect = -self.front_vect[left_leg] - self.segm_post_ant[1]
			if (left_leg == 0):
				left_foot_cog_vect = -self.front_vect[left_leg] - self.segm_post_ant[1] - self.segm_post_ant[0]
			left_foot_cog_vect[2] = 0.
#			print("Stability: ", left_leg, right_leg, diag_vect, left_foot_cog_vect)
			# The stability is determined in the following way:
			# The connection between the two most hind legs touching the ground
			# (one leg on each side) is constructing a vector to the back.
			# The segment_factor is the fraction of the last body segment meeting this
			# vector (counted from in between middle and hind segment):
			# 	0 = between the two segments (0 part of the hind segment)
			#	-1 = at the end of the hind segment_factor
			# The robot is determined instable when the segment_factor is greater
			# than this threshold.
			segment_factor = (diag_vect[1]*left_foot_cog_vect[0] - diag_vect[0]*left_foot_cog_vect[1]) \
				/ (diag_vect[0]*self.segm_post_ant[1][1] - diag_vect[1]*self.segm_post_ant[1][0]) 
			# Correction factor of the parameter:
			# If the most hind leg is a middle leg, the factor has to be increased by one
			# - if both are front legs, it has to be increased by two.
# For legs further to the front: is now already counteracted above = in left_foot_cog_vect
#			segment_factor += (2-max(left_leg,right_leg)//2)
#			print("Stability Problem Detector ", segment_factor)

			# Used for storing stability calculation in visualization
			self.temp_stability_fact = segment_factor
			if self.temp_stability_fact > self.stability_threshold:
				print("Instable", self.temp_stability_fact)
				input()
			
			if segment_factor > self.stability_threshold:
				#print("Stability along middle segment - legs: ", left_leg, right_leg, " - factor ", segment_factor)
				stability = False
				#input("Press Enter to continue...")
		return stability