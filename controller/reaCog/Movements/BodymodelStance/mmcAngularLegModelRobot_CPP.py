'''
Created on 22.1.2012

Changed 20.1.2014 - now adapted to the hector convention of joints
(gamma rotated ninety degrees, sign changes)
Switched to c++ for computations.

@author: mschilling
'''
import numpy

from .SWIG_MMC_Lib.mmcLeg import MmcLeg
##
#	Three segmented manipulator with three rotational joints.
#	Class for representing a Mean of Multiple Computation Network of an insect leg.
#	It is restricted to three degrees of freedom (non-redundant).
#
#	Version in which the computations are done efficiently in C++.
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
		
#		self.sensor_integration = 0
#		self.current_sensor_angles = [[], [], []]
		# Noise can be added to sensory data which shall be integrated
		# The noise is normal distributed around zero and the standard deviation has to
		# be provided. For std dev = 0 no noise is applied.
		# The noise is only applied when providing sensory data
		# using the add_sensory_joint_values function is used.
#		self.noise_std_dev = 0.0
#		self.noise = [0,0,0]

		# Direction of the beta drives.
		# For left and right side these are of course different.
		# AND: In the hind legs those are pointing in the opposite direction
		# and the signs of beta and gamma have to be reversed.
		if (self.motivationNetLeg.wleg.leg.beta_direction):
			self.betaDirectionFactor = 1.
		else:
			self.betaDirectionFactor = -1.
		# Segment lengths, beta direction, damping factor
		self.swig_stance_leg_model = MmcLeg(self.leg_segm, self.betaDirectionFactor, 1)

	##	Return the current joint angles.
	#	After iteration of the network the joint variables can be returned.
	def get_joint_angles(self):
		return self.swig_stance_leg_model.get_joint_angles()

	##	Set new joint angles(as a list).
	#	When the network shall be used to compute the forward kinematic function
	#	the target joint angles are enforced onto the network before an iteration step
	#	using this function.
	def set_joint_angles(self, angles):
		self.swig_stance_leg_model.set_joint_angles(angles)
		
	##	The MMC Method:
	#	- the multiple computations are computed for each variable
	#	- the mean for each variable is calculated
	#	The new values are appended to the list of element values.
	#	For each variable new values are calculated through
	#	different equations.
	def mmc_kinematic_iteration_step(self):
		self.swig_stance_leg_model.mmc_kinematic_iteration_step()

	##	Sets the sensor feedback values of the joints.
	#	These are integrated in the MMC network.
#	def set_sensory_feedback_joint_values(self, angles):
#		self.current_sensor_angles[0] = [angles[0]]
#		self.current_sensor_angles[1] = [self.hindLegFactor*angles[1]]
#		self.current_sensor_angles[2] = [self.hindLegFactor*angles[2]]

	##	Adds sensory feedback for joints.
	#	When we want to do sensory integration we add sensory influences
	#	which are added to the list of sensory feedback and noise is applied.
	#	This function allows to introduce multiple sensor data which
	#	is integrated by the network: on a temporal scale and constrained
	#	by the actual kinematics.
#	def add_sensory_joint_values(self, angles):
#		if (self.noise_std_dev > 0):
#			self.noise = normal(0,self.noise_std_dev, 3)
#		self.current_sensor_angles[0].append(angles[0] + self.noise[0])
#		self.current_sensor_angles[1].append(angles[1] + self.noise[1])
#		self.current_sensor_angles[2].append(angles[2] + self.noise[2])

	##	Converge to inverse kinematic solution.
	#	Access method for the stance network - for a given target position
	#	the network converges over a couple of timesteps (around 5 sufficient, 10
	#	very good) to a solution.
	def computeInverseKinematics(self, target_point):
		target_in_leg = numpy.dot(self.motivationNetLeg.wleg.leg._phi_psi_trans_inv, target_point)
		return self.swig_stance_leg_model.compute_inverse_kinematics( target_in_leg )
