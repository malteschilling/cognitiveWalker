'''
Created on 3.12.2011

@author: mschilling
'''
import math
import numpy


import Hector.RobotSettings as RSTATIC
import controller.reaCog.WalknetSettings as WSTATIC

##
#	 A Body Model for a hexapod walker, based on MMC computation.
#	Higher level of the hierarchical body model for a six legged robot:
#	On this level
#		- there are three segment vectors
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

	##	Initialisation of the body model.
	#	The segment vectors are encoded here, the leg vectors are initialised here
	#	but are actually set when a leg is really put on the ground (initially all are
	#	assumed in the air, so an update of the legs is forced in the first iteration)
	#	You can start the body interval with an optional argument for drawing
	#	a simplified version of it: first argument is the update interval
	#	(1 = every iteration step, ...) and with the kind of view you want
	#	(0 = simple top view, 1 = 3D like view).
	##
	def __init__(self, motiv_net):
		self.motivationNetRobot = motiv_net
		# The height is currently set to a fixed value
		# Here a height net might be introduced?
		# The body model is now always enforcing this specific height.
		self.height = WSTATIC.stanceheight
		# Enconding information about how the actual leg target vectors are calculated
		# from the leg vector (displacement from leg target vector to leg vector)
		self.leg_to_target = [numpy.array([0.19, -0.07, 0.0]), numpy.array([0.19, 0.07, 0.0]),
			numpy.array([0.33, -0.07, 0.0]), numpy.array([0.33, 0.07, 0.0]),
			numpy.array([0.2, -0.07, 0.0]), numpy.array([0.2, 0.07, 0.0])]
		# All the variables are stored in lists over time - i.e. for each time step
		# all the values are saved (this could be changed in the long run).
		# Segment vectors
		self.segm = [[numpy.array([0.22,0.0,0.0])],[numpy.array([0.35, 0.0, 0.0])],[numpy.array([0.36,0.0,0.0])]]
		self.length_segm = [numpy.linalg.norm(self.segm[0][0]), numpy.linalg.norm(self.segm[1][0]), numpy.linalg.norm(self.segm[2][0])]
		# Leg Vectors (from front of a segment to footpoint)
		self.front_vect = [[numpy.array([-0.19, 0.3,-self.height])],[numpy.array([-0.19, -0.3,-self.height])],
					[numpy.array([-0.33,0.3,-self.height])],[numpy.array([-0.33,-0.3,-self.height])],
					[numpy.array([-0.2, 0.3,-self.height])],[numpy.array([-0.2, -0.3,-self.height])]]
		# Diag vectors (from back of a segment to footpoint)
		self.rear_vect = [[(self.segm[0][0] + self.front_vect[0][0])], [(self.segm[0][0] + self.front_vect[1][0])], \
					 [(self.segm[1][0] + self.front_vect[2][0])], [(self.segm[1][0] + self.front_vect[3][0])], \
					 [(self.segm[2][0] + self.front_vect[4][0])], [(self.segm[2][0] + self.front_vect[5][0])] ]
		# Help vectors for the drawing routines: We have to keep track of the feet
		# positions in a global coordinate system (and of one segment, too)
		self.segm1_in_global = numpy.array([-0.02,0.0,self.height])
		self.foot_global = [(self.segm1_in_global + self.segm[1][0] + self.segm[0][0] + self.front_vect[0][0]),
			(self.segm1_in_global + self.segm[1][0] + self.segm[0][0] + self.front_vect[1][0]),
			(self.segm1_in_global + self.segm[1][0] + self.front_vect[2][0]),
			(self.segm1_in_global + self.segm[1][0] + self.front_vect[3][0]),
			(self.segm1_in_global + self.front_vect[4][0]),
			(self.segm1_in_global + self.front_vect[5][0])]
		# Ground contact - which feet are on the ground
		self.gc = [[False, False, False, False, False, False]]
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
		i = 0
		j = 0
		for i in range(0,6) :
			for j in range(0,i) :
				self.footdiag[i][j] = self.set_up_foot_diag(i,j,0)
		# The explicit disturbance vectors of the network
		self.delta_front = [[numpy.array([0,0,0])], [numpy.array([0,0,0])], [numpy.array([0,0,0])]]
		self.delta_back = [[numpy.array([0,0,0])], [numpy.array([0,0,0])], [numpy.array([0,0,0])]]
		# These are operated through a pull at the front
		self.pull_front = numpy.array([0.0,0.0,0.0])
		self.pull_back = numpy.array([0.0,0.0,0.0])
		self.step = 0
		self.damping = 10

	""" **** Set up methods and calculation of vector methods ***************************
	"""
	##	Calculation of the vector connection the feet of two standing legs.
	#	When a leg is put on the ground a new connecting vector has to be established.
	#	This is calculated as the difference between the leg vectors.
	def set_up_foot_diag(self, start, target, step) :
		diag_vec = self.front_vect[target][step] - (self.front_vect[start][step] \
			- self.get_segm_vectors_between_legs(start, target, step) )
		return diag_vec

	##	Providing the segment vectors connecting two legs.
	#	An equation is described by a series of vectors forming a closed kinematic
	#	chain (two leg vectors, the footdiag and -possibly- segment vectors in
	#	between). This function is calculating the segment vectors between the
	#	given legs.
	def get_segm_vectors_between_legs(self, start_leg, end_leg, step):
		leg_diff = (end_leg//2-start_leg//2)
		if leg_diff == 0 :
			return numpy.array([0., 0., 0.])
		elif leg_diff == -1 :
			return numpy.array(self.segm[end_leg//2][step])
		elif leg_diff == 1 :
			return numpy.array(self.segm[start_leg//2][step])
		elif abs(leg_diff) == 2 :
			return (self.segm[min(start_leg, end_leg)//2][step]
				+ self.segm[max(start_leg, end_leg)//2 - 1][step])

	##	Sets ground contact to false and removes this leg from the body
	#	model computations. As the leg is not part of the closed kinematic chains
	#	after being lifted from the ground it shall not participate.
	def lift_leg_from_ground(self, leg_nr) :
		if (self.gc[-1][leg_nr]) :
			self.gc[-1][leg_nr] = False

	##	Sets the ground contact of the leg and initiates the calculation of connection
	#	vectors to all other standing legs (footdiag) which are used by the
	#	network.
	def put_leg_on_ground(self, leg_name, leg_vec) :
		leg_nr = RSTATIC.leg_names.index(leg_name)
		i = 0
		if (self.gc[-1][leg_nr] == False) :
			#print("PUT ON GROUND")
			#print(leg_vec)
			# Set leg and diag vector
			self.front_vect[leg_nr][-1] = numpy.array(leg_vec)
			self.rear_vect[leg_nr][-1] = self.segm[leg_nr//2][-1] + leg_vec
			# Construction of all foot vectors - the ones to legs in the air are not used!
			for i in range(0,leg_nr) :
				self.footdiag[leg_nr][i] = self.set_up_foot_diag(leg_nr, i, -1)
			for i in range(leg_nr+1,6):
				self.footdiag[i][leg_nr] = self.set_up_foot_diag(i, leg_nr, -1)
			# Derive the global position (needed for the graphics output)
			for i in range(0,6):
				if (self.gc[-1][i]):
					if (i > leg_nr):
						self.foot_global[leg_nr] = self.foot_global[i] + self.footdiag[i][leg_nr]
					else:
						self.foot_global[leg_nr] = self.foot_global[i] - self.footdiag[leg_nr][i]
					#print("New global " + str(self.foot_global[leg_nr]))
					break
			self.gc[-1][leg_nr] = True

	def condAppend(self, var_list, new_element, appendOrInsert=True):
		if appendOrInsert:
			var_list.append(new_element)
		else:
			var_list[0]=new_element

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
	def get_manipulator_coordinates(self, leg, step):
		return [[self.foot_global[leg][0], (self.foot_global[leg][0] - self.front_vect[leg][step][0]), \
				 (self.foot_global[leg][0] - self.front_vect[leg][step][0] - self.segm[leg//2][step][0]), \
				 (self.foot_global[leg][0] - self.front_vect[leg][step][0] - self.segm[leg//2][step][0] + self.rear_vect[leg][step][0])], \
				[self.foot_global[leg][1], (self.foot_global[leg][1] - self.front_vect[leg][step][1]), \
				 (self.foot_global[leg][1] - self.front_vect[leg][step][1] - self.segm[leg//2][step][1]), \
				 (self.foot_global[leg][1] - self.front_vect[leg][step][1] - self.segm[leg//2][step][1] + self.rear_vect[leg][step][1])], \
				[self.foot_global[leg][2], (self.foot_global[leg][2] - self.front_vect[leg][step][2]), \
				 (self.foot_global[leg][2] - self.front_vect[leg][step][2] - self.segm[leg//2][step][2]), \
				 (self.foot_global[leg][2] - self.front_vect[leg][step][2] - self.segm[leg//2][step][2] + self.rear_vect[leg][step][2])]]


	""" **** Computation of the MMC equations *******************************************
	"""
	##	Compute the leg vectors: For all standing legs
	#	the new leg vectors are computed, summed and the mean is calculated
	#	(the old value is also integrated, weighted by the damping value)
	def compute_leg_computations_and_integrate(self, leg_nr, step):
		leg_comp = []
		# Computation including the explicit displacement of the leg
		# (which is given by the pull vector or by the error from the preceding
		# segment)
		leg_comp.append( (-self.delta_front[leg_nr//2][step] - self.segm[leg_nr//2][step]
			   				  + self.rear_vect[leg_nr][step]) )
		leg_comp.append( (self.damping * self.front_vect[leg_nr][-1]) )
		target_leg = 0
		# Compute equations to all other standing legs using the footdiag
		for target_leg in range(0, 6):
			if self.gc[step][target_leg] and target_leg != leg_nr :
				part_vec = self.get_segm_vectors_between_legs( leg_nr, target_leg, step )
				if target_leg < leg_nr:
					part_vec -= self.footdiag[leg_nr][target_leg]
				elif target_leg > leg_nr:
					part_vec = -part_vec + self.footdiag[target_leg][leg_nr]
				part_vec += self.front_vect[target_leg][step]
				leg_comp.append(part_vec)
		sum_comp = numpy.array([0., 0., 0.])
		for comp in leg_comp:
			sum_comp += comp
		return(sum_comp/(len(leg_comp) + self.damping - 1))

	##	Compute the diag vectors: For all standing legs
	#	the new diag vectors are computed, summed and the mean is calculated
	#	(the old value is also integrated, weighted by the damping value)
	def compute_diag_computations_and_integrate(self, leg_nr, step):
		diag_comp = []
		diag_comp.append( (-self.delta_back[leg_nr//2][step] + self.segm[leg_nr//2][step] + self.front_vect[leg_nr][step]) )
		diag_comp.append( (self.damping * self.rear_vect[leg_nr][-1]) )
		target_leg = 0
		# Compute connections to other standing legs - looking up the footdiag
		for target_leg in range(0, 6):
			if self.gc[step][target_leg] and target_leg != leg_nr :
				part_vec = self.get_segm_vectors_between_legs( leg_nr + 2, target_leg + 2, step)
				if target_leg < leg_nr:
					part_vec -= self.footdiag[leg_nr][target_leg]
				elif target_leg > leg_nr:
					part_vec = -part_vec + self.footdiag[target_leg][leg_nr]
				part_vec += self.rear_vect[target_leg][step]
				diag_comp.append(part_vec)
		sum_comp = numpy.array([0., 0., 0.])
		for comp in diag_comp:
			sum_comp += comp
		return(sum_comp/(len(diag_comp) + self.damping - 1))

	##	Compute the segment vectors:
	#	Using equations including the two legs connected to the segment,
	#	integrating the explicit displacement given as delta
	#	and the recurrent old value of the vector.
	def compute_segment_computations_and_integrate(self, segm_nr, step):
		segm_comp = []
		if (self.gc[step][segm_nr*2]) :
			segm_comp.append(self.rear_vect[segm_nr*2][step] - self.front_vect[segm_nr*2][step])
		if (self.gc[step][segm_nr*2 + 1]) :
			segm_comp.append(self.rear_vect[segm_nr*2 + 1][step] - self.front_vect[segm_nr*2 + 1][step])
		# Integrate explicit displacement vector delta.
		segm_comp.append(self.segm[segm_nr][step] + self.delta_front[segm_nr][step] - self.delta_back[segm_nr][step])
		segm_comp.append(self.damping * self.segm[segm_nr][-1])
		sum_comp = numpy.array([0., 0., 0.])
		for comp in segm_comp:
			sum_comp += comp
		return( ( (sum_comp/(len(segm_comp) + self.damping - 1))*(self.length_segm[segm_nr]/
			numpy.linalg.norm(sum_comp/(len(segm_comp) + self.damping - 1)) )) )

	##	The MMC Method:
	#	- the multiple computations are computed for each variable
	#	- the mean for each variable is calculated
	#	The new values are appended to the list of element values.
	#	For each variable new values are calculated through
	#	different equations.
	##
	def mmc_iteration_step(self):
		#print("Step")
		# Calculating the delta:
		# For the first segment the pull value is used, for the following segments
		# the error vector from the preceding segment from the last iteration
		# is pushed down to this segment
		# Calculate the delta vectors at the front for the displacements for each segment
		new_delta_front_1 = (self.rear_vect[0][-1] - self.front_vect[0][-1] - self.segm[0][-1])
		new_delta_front_2 = (self.rear_vect[2][-1] - self.front_vect[2][-1] - self.segm[1][-1])
		# Calculate the delta vector pulling at the back for the displacements
		#for each segment
		new_delta_back_1 = -(self.rear_vect[4][-1] - self.front_vect[4][-1] - self.segm[2][-1])
		new_delta_back_0 = -(self.rear_vect[2][-1] - self.front_vect[2][-1] - self.segm[1][-1])
		# Computing the segment vectors: Adding up directly the three equations:
		# left leg and diagonal, right leg and diagonal, old segment and displacement delta
		new_segm = [self.compute_segment_computations_and_integrate(i, -1) for i in range(0,3)]
		# Calculating the new leg vectors and the diagonals
		# Calculate only legs on ground!
		new_f = [self.compute_leg_computations_and_integrate(i, -1) for i in range(0,6)]
		new_r = [self.compute_diag_computations_and_integrate(i,-1) for i in range(0,6)]
		#self.gc.append(list(self.gc[-1]))
		# Example pull vectors in which the direction is constantly changing
		#self.pull.x = self.pull.x * math.cos(0.002) - self.pull[1] * math.sin(0.002)
		#self.pull[1] = self.pull[1] * math.cos(0.002) + self.pull.x * math.sin(0.002)
		self.step += 1
		if WSTATIC.logging_values :
			self.delta_front[0].append(numpy.array(self.pull_front))
			self.delta_front[1].append(new_delta_front_1)
			self.delta_front[2].append(new_delta_front_2)
			self.delta_back[2].append(numpy.array(self.pull_back))
			self.delta_back[1].append(new_delta_back_1)
			self.delta_back[0].append(new_delta_back_0)
			for i in range(0,3):
				self.segm[i].append(new_segm[i])
			for i in range(0,6):
				self.front_vect[i].append(new_f[i])
				self.rear_vect[i].append(new_r[i])
			# Make a copy of the gc list
			self.condAppend(self.gc, (list(self.gc[-1])), WSTATIC.logging_values )
		else:
			self.delta_front[0][0] = numpy.array(self.pull_front)
			self.delta_front[1][0] = new_delta_front_1
			self.delta_front[2][0] = new_delta_front_2
			self.delta_back[2][0] = numpy.array(self.pull_back)
			self.delta_back[1][0] = new_delta_back_1
			self.delta_back[0][0] = new_delta_back_0
			for i in range(0,3):
				self.segm[i][0] = new_segm[i]
			for i in range(0,6):
				self.front_vect[i][0] = new_f[i]
				self.rear_vect[i][0] = new_r[i]


	""" **** Get, set methods - connection to the robot simulator ***********************
	"""
	##	Pull the body model into a direction relative to the current first
	#	segment. Takes an angle (0 os straight ahead) and a velocity factor
	#	(around 0.1-0.2 should be fine) to come up with a corresponding pull vector.
	def pullBodyModelAtFrontIntoRelativeDirection(self, pull_angle, speed_fact):
		pull_angle_BM = pull_angle + math.atan2( self.segm[0][-1][1], self.segm[0][-1][0])
		self.pull_front[0] = speed_fact * math.cos(pull_angle_BM) # pull x
		self.pull_front[1] = speed_fact * math.sin(pull_angle_BM) # pull y

	##	Pull the body model into a direction relative to the last body
	#	segment. Takes an angle (0 means straight backwards) and a velocity factor
	#	(around 0.1 - positive means backwards walking!)
	#	to come up with a corresponding pull vector.
	def pullBodyModelAtBackIntoRelativeDirection(self, pull_angle, speed_fact):
		pull_angle_BM = pull_angle + math.atan2( -self.segm[2][-1][1], -self.segm[2][-1][0])
		self.pull_back[0] = speed_fact * math.cos(pull_angle_BM) # pull x
		self.pull_back[1] = speed_fact * math.sin(pull_angle_BM) # pull y

	##	Get the angles between the inner segments which are used for the
	#	segment joints.
	def get_segment_angles(self):
		front_angle = math.atan2(self.segm[0][-1][1],self.segm[0][-1][0])
		middle_angle = math.atan2(self.segm[1][-1][1],self.segm[1][-1][0])
		back_angle = math.atan2(self.segm[2][-1][1],self.segm[2][-1][0])
#~ #~
		angle1=front_angle - middle_angle
		if angle1<-3.14:
			angle1+=2*math.pi
		elif angle1>3.14:
			angle1-=2*math.pi
#~ #~
		angle2=middle_angle - back_angle
		if angle2<-3.14:
			angle2+=2*math.pi
		elif angle2>3.14:
			angle2-=2*math.pi
#~ #~
		return ([angle1,angle2])

	##	Get the target leg vectors for the leg networks (computation of the leg
	#	inverse kinematics): used are the leg and the segm vectors.
	#	The transformation is described by the displacement leg_to_target
	#	(there the coordinate reference system at the tip of the segment (aligned
	#	with the segment) is described with respect to the robot leg coordinate
	#	system).
	#	At the end there is another transformation needed - applying the phi,psi
	#	Transformation (is done right now here, but should be in the robot?).
	def get_robot_target_from_leg_vector(self, leg_name):
		leg_nr = RSTATIC.leg_names.index(leg_name)
		target_vec = numpy.array([0.,0.,0.])
		segm_angle = math.atan2(self.segm[leg_nr//2][-1][1],self.segm[leg_nr//2][-1][0])
		target_vec[0] = self.front_vect[leg_nr][-1][0] * math.cos(-segm_angle) \
			- self.front_vect[leg_nr][-1][1] * math.sin(-segm_angle) \
			+ self.leg_to_target[leg_nr][0]
		target_vec[1] = self.front_vect[leg_nr][-1][0] * math.sin(-segm_angle) \
			+ self.front_vect[leg_nr][-1][1] * math.cos(-segm_angle) \
			+ self.leg_to_target[leg_nr][1]
		target_vec[2] = self.front_vect[leg_nr][-1][2] + self.leg_to_target[leg_nr][2]
		target_vec_wn = [target_vec[0], target_vec[1], target_vec[2], 0]
		return target_vec_wn

	##	Push new target leg vector into the body model.
	#	Returns a valid leg vector
	#	(is the inverse function to get_robot_target_from_leg_vector).
	#	This value can be used to update the leg vector by sensory measured values.
	#	At the end there is another transformation needed - applying the phi,psi
	#	Transformation (is done right now here, but should be in the robot?).
	def transform_robot_target_to_leg_vector(self, target_vec_wn, leg_name):
		leg_nr = RSTATIC.leg_names.index(leg_name)
		new_leg_vec = numpy.array([0., 0.,0.])
		#target_vec = vec3(((-1)**(leg_nr+1)*target_vec_wn[1]), ((-1)**(leg_nr)*target_vec_wn[0]), target_vec_wn[2])
		target_vec = numpy.array([target_vec_wn[0], target_vec_wn[1], target_vec_wn[2]])
		segm_angle = math.atan2(self.segm[leg_nr//2][-1][1],self.segm[leg_nr//2][-1][0])
		new_leg_vec[0] = target_vec[0] * math.cos(segm_angle) \
			- target_vec[1] * math.sin(segm_angle) \
			- self.leg_to_target[leg_nr][0] * math.cos(segm_angle) \
			+ self.leg_to_target[leg_nr][1] * math.sin(segm_angle)
		new_leg_vec[1] = target_vec[0] * math.sin(segm_angle) \
			+ target_vec[1] * math.cos(segm_angle) \
			- self.leg_to_target[leg_nr][0] * math.sin(segm_angle) \
			- self.leg_to_target[leg_nr][1] * math.cos(segm_angle)
		new_leg_vec[2] = -self.height #target_vec[2]
		return new_leg_vec
		
	def get_normalised_angle_between_numpy_vectors(self, vect_1, vect_2):
		angle_1 = math.atan2(vect_1[1],vect_1[0])
		angle_2 = math.atan2(vect_2[1],vect_2[0])
		diff_angle = angle_1 - angle_2
		if (diff_angle < -2*math.pi): diff_angle += 2*math.pi
		if (diff_angle > 2*math.pi): diff_angle -= 2*math.pi
		return diff_angle

	##
	#	Check if the BM configuration is static stable
	def check_static_stability_angle(self):
		# First: Look for the right and left leg which are most upfront and have gc
		left_leg, right_leg = 0, 1
		stability = True
		# First: test if CoG moves ahead of the connecting line
		# connecting the leg on each side which
		# - has gc
		# - is the foremost leg having gc on that side
		while left_leg<6 and not(self.gc[-1][left_leg]):
			left_leg += 2
		while right_leg<6 and not(self.gc[-1][right_leg]):
			right_leg += 2
		if (left_leg > 5) or (right_leg > 5):
			stability = False
		else:
			# Two angles are calculated from the gc position of the left leg
			# First, the angle of the line towards the right leg
			# Second, the angle of the vector from the gc towards the CoG
			# The model is static stable if the second vector is more rotated clockwise
			# (meaning the CoG lies "below" the other connecting line)
			diag_vect = -self.front_vect[left_leg][-1] + self.front_vect[right_leg][-1] \
				- self.get_segm_vectors_between_legs(left_leg, right_leg, -1)
			left_foot_cog_vect = -self.front_vect[left_leg][-1]
			i = 4
			while i>left_leg:
				left_foot_cog_vect -= self.segm[i // 2 - 1][-1]
				i -= 2
			if ( self.get_normalised_angle_between_numpy_vectors(diag_vect, left_foot_cog_vect) < 0):
				stability = False
				print("INSTABLE")
			else:
				left_leg, right_leg = 4, 5
				# Second: test if CoG moves moves behind the connecting line
				# connecting the leg on each side which
				# - has gc
				# - is the leg furthest backward having gc on that side
				while left_leg>=0 and not(self.gc[-1][left_leg]):
					left_leg -= 2
				while right_leg>0 and not(self.gc[-1][right_leg]):
					right_leg -= 2
				if (left_leg < 0) or (right_leg < 0):
					stability = False
				else:
					# Two angles are calculated from the gc position of the left leg
					# First, the angle of the line towards the right leg
					# Second, the angle of the vector from the gc towards the CoG
					# The model is static stable if the second vector is less rotated clockwise
					# (meaning the CoG lies "below" the other connecting line)
					diag_vect = -self.front_vect[left_leg][-1] + self.front_vect[right_leg][-1] \
						- self.get_segm_vectors_between_legs(left_leg, right_leg, -1)
					left_foot_cog_vect = -self.front_vect[left_leg][-1]
					i = 4
					while i>left_leg:
						left_foot_cog_vect -= self.segm[i // 2 - 1][-1]
						i -= 2
					print("Stability: ", left_leg, right_leg, diag_vect, left_foot_cog_vect)
					if ( self.get_normalised_angle_between_numpy_vectors(diag_vect, left_foot_cog_vect) > 0):
						stability = False
						print("Instable at back - ", self.step)
						print(left_leg, right_leg, self.get_normalised_angle_between_numpy_vectors(diag_vect, left_foot_cog_vect))
						print(diag_vect, left_foot_cog_vect)
						print(self.front_vect[2][-1], self.front_vect[5][-1], self.get_segm_vectors_between_legs(left_leg, right_leg, -1))
						input("Press Enter to continue...")
		return stability
		
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
		left_leg, right_leg = 4, 5
		# Test if CoG moves moves behind the connecting line
		# connecting the leg on each side which
		# - has gc
		# - is the leg furthest backward having gc on that side
		while left_leg>=0 and not(self.gc[-1][left_leg]):
			left_leg -= 2
		while right_leg>0 and not(self.gc[-1][right_leg]):
			right_leg -= 2
		# If there is no gc at all on one side it should be unstable
		if (left_leg < 0) or (right_leg < 0):
			stability = False
		else:
			#print(left_leg, right_leg)
			diag_vect = -self.front_vect[left_leg][-1] + self.front_vect[right_leg][-1] \
						+ self.get_segm_vectors_between_legs(left_leg, right_leg, -1)
			left_foot_cog_vect = -self.front_vect[left_leg][-1]
			left_foot_cog_vect[2] = 0.
			#print("Stability: ", left_leg, right_leg, diag_vect, left_foot_cog_vect)
			segment_factor = (diag_vect[1]*left_foot_cog_vect[0] - diag_vect[0]*left_foot_cog_vect[1]) \
				/ (diag_vect[0]*self.segm[1][-1][1] - diag_vect[1]*self.segm[1][-1][0]) 
			# Correction factor of the parameter:
			# If the most hind leg is a middle leg, the factor has to be increased by one
			# - if both are front legs, it has to be increased by two.
			segment_factor += (2-max(left_leg,right_leg)//2)
			#print("Stability along middle segment - factor ", segment_factor)
			if segment_factor > 0.25:
				#print("Stability along middle segment - legs: ", left_leg, right_leg, " - factor ", segment_factor)
				stability = False
				#input("Press Enter to continue...")
		return stability

# Small test of the mmcBodyModel
# test = mmcBodyModel(1)
# i = 0
# test.gc[0] = [True,True,True,True,True,True]
# test.pullBodyModelAtFrontIntoRelativeDirection(0.32175, 0.31623)
# for i in range(i,100):
#   test.mmc_iteration_step()
# input("Press Enter to continue...")
