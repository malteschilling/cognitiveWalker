'''
Created on 22.1.2012

Changed 20.1.2014 - now adapted to the hector convention of joints
(gamma rotated ninety degrees, sign changes)

@author: mschilling
'''
import math
import numpy

from numpy.random import normal
##
#	Three segmented manipulator with three rotational joints.
#	Class for representing a Mean of Multiple Computation Network of an insect leg.
#	It is restricted to three degrees of freedom (non-redundant).
#
#	As arguments the segment length can be set-up.
#	Afterwards all the necessary variables are set-up.
#
#	Uses trigonometric relations to calculate the different variables.
#	In case of the joint computations, multiple values are integrated
#	using the arc-tangens function which is unambiguous (in contrast to asin, ...).
#
#	The network is quite performant - it needs only a few iteration steps for
#	relaxation (~ 6 for completely	new posture, for small movements two or three
#	can be sufficient). On a single core 1.6 Ghz this allowed for more then 30.000
#	iteration steps per second (a single iteration step costs about the same as
#	the direct calculation of the inverse kinematics).
#
#	The leg network can be initialised with a list (3 elements) of the segments
#	lengths - the arm is initialised in an outstretched position.
#
#	The network is considered in leg coordinate system = after application of the
#	phiPsi transformation.
#
#	The network is storing all changing variables in lists over time (iteration steps).
##
class mmcAngularLegModelStance:
	def __init__(self, motiv_leg, logging = False):
		# Initialisation of the segment lengths (as argument list given to the object)
		self.motivationNetLeg = motiv_leg
		self.leg_segm = [self.motivationNetLeg.wleg.leg.segment_lengths[0], self.motivationNetLeg.wleg.leg.segment_lengths[1], self.motivationNetLeg.wleg.leg.segment_lengths[2]]
		self.leg_target = [numpy.array([(self.leg_segm[0] + self.leg_segm[1] + self.leg_segm[2]),0.0,0.0])]
		self.projection_length = [numpy.linalg.norm(self.leg_target[0])]
		self.joint = [[-0.01, 0.01, 0.01]]
		self.step = 0
		self.damping = 1
		self.sensor_integration = 0
		self.current_sensor_angles = [[], [], []]
		# Noise can be added to sensory data which shall be integrated
		# The noise is normal distributed around zero and the standard deviation has to
		# be provided. For std dev = 0 no noise is applied.
		# The noise is only applied when providing sensory data
		# using the add_sensory_joint_values function is used.
		self.noise_std_dev = 0.0
		self.noise = [0,0,0]
		self.logging_values = logging
		self.lc = self.leg_segm[0]
		self.lf = self.leg_segm[1]
		self.lt = self.leg_segm[2]
		
		# Direction of the beta drives.
		# For left and right side these are of course different.
		# AND: In the hind legs those are pointing in the opposite direction
		# and the signs of beta and gamma have to be reversed.
		if (self.motivationNetLeg.wleg.leg.beta_direction):
			self.betaDirectionFactor = 1.
		else:
			self.betaDirectionFactor = -1.
		
		self.new_target = numpy.array([0., 0., 0.])
		self.new_joints = numpy.array([0., 0., 0.])
		
	##	Compute the joint angles.
	#	Exploiting trigonometric relations the joint angles are calculated in
	#	multiple ways. These computations would involve computing asin and acos
	#	values. This is problematic as these are ambiguous. Therefore, we integrate
	#	these computations in a first step by computing the quotient of the sine and
	#	cosine value which equals the tangens value. From this we can compute the
	#	unambiguous arc tangens value for the joint value.
	#
	#	The function is called with the iteration step on which the computations
	#	should be applied. Usually, it should be called with thisObject.step.
	def compute_joints_and_integrate(self, step):
		self.new_joints = [0.0, 0.0, 0.0]
		# Computation of the alpha angle
		mc_joint = [-math.atan2(self.leg_target[step][1], self.leg_target[step][0])]
		if ((abs(self.projection_length[step]) > 0.1) and
			(abs(self.leg_target[step][1]/self.projection_length[step]) < 1.0)) :
			mc_joint.append(-math.asin(self.leg_target[step][1]/self.projection_length[step]))
		# Sensor integration: if sensor input is provided it is added here
		if (len(self.current_sensor_angles[0]) > 0) :
			mc_joint.append(sum(self.current_sensor_angles[0]))
		# Recurrent connection
		mc_joint.append(self.damping * self.joint[step][0])
		self.new_joints[0] = sum(mc_joint)/(self.damping + len(mc_joint) - 1)

		# Computation of the beta angle
		# Integration of two trigonometric relations
		mc_joint = [(math.atan2( (self.leg_target[step][2]
					+ self.leg_segm[2] * math.cos(self.joint[step][1] - self.joint[step][2]) ),
				(self.projection_length[step] - self.leg_segm[0]
					- self.leg_segm[2] * math.sin(self.joint[step][1] - self.joint[step][2] ) ) ))]
		# Integration of sensor data if given
		if (len(self.current_sensor_angles[1]) > 0) :
			mc_joint.append(sum(self.current_sensor_angles[1]))
		# Recurrent connection
		mc_joint.append(self.damping * self.joint[step][1])
		self.new_joints[1] = sum(mc_joint)/(self.damping + len(mc_joint) - 1)
		# Enforce joint constraints
		if (self.new_joints[1] < 0.01):
			self.new_joints[1] = 0.01

		# Computation of the gamma angle
		# Integration of two trigonometric relations
		mc_joint = [-(math.atan2( (self.leg_target[step][2]
					- self.leg_segm[1] * math.sin(self.joint[step][1]) ),
				(self.projection_length[step] - self.leg_segm[0]
					- self.leg_segm[1] * math.cos(self.joint[step][1]) )) - self.joint[step][1]) - math.pi/2]
		# Integration of sensor data if given
		if (len(self.current_sensor_angles[2]) > 0) :
			mc_joint.append(sum(self.current_sensor_angles[2]))
		# Recurrent connection
		mc_joint.append(self.damping * self.joint[step][2])
		self.new_joints[2] = sum(mc_joint)/(self.damping + len(mc_joint) - 1)
		if (self.new_joints[2] < -1.5608):
			self.new_joints[2] = -1.5608
		#return self.new_joints

	##	Compute the target vector.
	#	Application of basically the forward kinematics
	#	- but also includes recurrent connections (the old value is partially
	#	maintained).
	def compute_target(self, step):
		self.new_target = [0., 0., 0.]
		if (abs(self.joint[step][0]) > 0.1) :
			self.new_target[0] = (self.leg_target[step][1] / math.tan(-self.joint[step][0])
				+ self.projection_length[step] * math.cos(self.joint[step][0])
				+ self.damping * self.leg_target[step][0]) / (2 + self.damping)
		else:
			self.new_target[0] = (self.projection_length[step] * math.cos(self.joint[step][0])
				+ self.damping * self.leg_target[step][0]) / (1 + self.damping)
		self.new_target[1] = (self.leg_target[step][0] * math.tan(-self.joint[step][0])
			+ self.projection_length[step] * math.sin(-self.joint[step][0])
			+ self.damping * self.leg_target[step][1]) / (2 + self.damping)
		self.new_target[2] = (self.leg_segm[1] * math.sin(self.joint[step][1])
					- self.leg_segm[2] * math.cos(self.joint[step][1] - self.joint[step][2])
				+ self.damping * self.leg_target[step][2]) / (1 + self.damping)
		#return self.new_target

	##	Compute of the projection length of the leg.
	#	The second and third joint work in a plane - spanned by the z-axis
	#	and rotated around this axis. The overall length of the segment in this plane
	#	is a projection of the x and y values onto this plane.
	def compute_projection_length(self, step):
		new_l = [(self.leg_segm[0] + self.leg_segm[1] * math.cos(self.joint[step][1])
					+ self.leg_segm[2] * math.sin(self.joint[step][1] - self.joint[step][2]) )]
		if (abs(self.joint[step][0]) > 0.1) :
			new_l.append( (self.leg_target[step][1] / math.sin(-self.joint[step][0]) ) )
		if (abs(self.joint[step][0]) < 1.47) :
			new_l.append( (self.leg_target[step][0] / math.cos(self.joint[step][0]) ) )
		new_l.append( self.damping * self.projection_length[step] )
		return (sum(new_l)/(self.damping + len(new_l) - 1))

	##	The MMC Method:
	#	- the multiple computations are computed for each variable
	#	- the mean for each variable is calculated
	#	The new values are appended to the list of element values.
	#	For each variable new values are calculated through
	#	different equations.
	def mmc_kinematic_iteration_step(self):
		self.compute_target(-1)
		new_projection_lengths = self.compute_projection_length(-1)
		self.compute_joints_and_integrate(-1)
		if self.logging_values :
			self.leg_target.append(self.new_target)
			self.projection_length.append(new_projection_lengths)
			self.joint.append(self.new_joints)
		else:
			self.leg_target[0] = self.new_target
			self.projection_length[0] = new_projection_lengths
			self.joint[0] = self.new_joints
		self.step += 1
		self.current_sensor_angles = [[], [], []]

	##	Return the current joint angles.
	#	After iteration of the network the joint variables can be returned.
	def get_joint_angles(self):
		return ([self.joint[-1][0],self.betaDirectionFactor*self.joint[-1][1],self.betaDirectionFactor*self.joint[-1][2]])

	##	Set the leg target vector (as a vec3).
	#	When the network shall be used to compute the inverse kinematic function
	#	the target vector is enforced onto the network before an iteration step
	#	using this function.
	def set_leg_target(self, new_target):
		self.leg_target[-1] = new_target

	##	Set new joint angles(as a list).
	#	When the network shall be used to compute the forward kinematic function
	#	the target joint angles are enforced onto the network before an iteration step
	#	using this function.
	def set_joint_angles(self, angles):
		self.joint[-1] = [angles[0],self.betaDirectionFactor*angles[1],self.betaDirectionFactor*angles[2]]

	##	Sets the sensor feedback values of the joints.
	#	These are integrated in the MMC network.
	def set_sensory_feedback_joint_values(self, angles):
		self.current_sensor_angles[0] = [angles[0]]
		self.current_sensor_angles[1] = [self.hindLegFactor*angles[1]]
		self.current_sensor_angles[2] = [self.hindLegFactor*angles[2]]

	##	Adds sensory feedback for joints.
	#	When we want to do sensory integration we add sensory influences
	#	which are added to the list of sensory feedback and noise is applied.
	#	This function allows to introduce multiple sensor data which
	#	is integrated by the network: on a temporal scale and constrained
	#	by the actual kinematics.
	def add_sensory_joint_values(self, angles):
		if (self.noise_std_dev > 0):
			self.noise = normal(0,self.noise_std_dev, 3)
		self.current_sensor_angles[0].append(angles[0] + self.noise[0])
		self.current_sensor_angles[1].append(angles[1] + self.noise[1])
		self.current_sensor_angles[2].append(angles[2] + self.noise[2])

	##	Converge to inverse kinematic solution.
	#	Access method for the stance network - for a given target position
	#	the network converges over a couple of timesteps (around 5 sufficient, 10
	#	very good) to a solution.
	def computeInverseKinematics(self, target_point):
		target_in_leg = numpy.dot(self.motivationNetLeg.wleg.leg._phi_psi_trans_inv, target_point)
		for _ in range(0,10):
			self.set_leg_target( target_in_leg )
			self.mmc_kinematic_iteration_step()
		return self.get_joint_angles()
