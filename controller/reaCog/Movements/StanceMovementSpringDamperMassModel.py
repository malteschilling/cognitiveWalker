import numpy
import copy
import Hector.RobotSettings as RSTATIC
import controller.reaCog.WalknetSettings as WSTATIC
from ..MotivationNetwork.ModulatedRoutine import ModulatedRoutine

class SpringDamperMass(object):
	def __init__(self,spring_constant, mass, unloaded_spring_length, damping_constant=None):
		self.mass=mass
		self.spring_mass_constant=numpy.sqrt(spring_constant/mass)
		if damping_constant:
			self.damping_constant=damping_constant
		else:
			self.damping_constant=2*self.mass*self.spring_mass_constant
		self.unloaded_spring_length=unloaded_spring_length
		self.position=0
		self.velocity=0
		
	def computeNextPosition(self, delta_t):
		self.position=numpy.e**(-self.spring_mass_constant*delta_t)*(self.velocity*delta_t+(self.position-self.unloaded_spring_length)+self.spring_mass_constant*delta_t*(self.position-self.unloaded_spring_length))+self.unloaded_spring_length
		return self.position

class StanceMovementSpringDamperMassBody(object):
	def __init__(self, mrobot, spring_constants, masses, l_zeros, damping_constants=None):
		self.mrobot=mrobot
		self.wrobot=self.mrobot.wrobot
		self.robot=self.wrobot.robot
		self.wlegs=self.wrobot.wlegs
		self.legs=self.robot.legs
		
		if not isinstance(spring_constants, (list, tuple)):
			spring_constants=[spring_constants for _ in self.legs]
		if not isinstance(damping_constants, (list, tuple)):
			damping_constants=[damping_constants for _ in self.legs]
		if not isinstance(masses, (list, tuple)):
			masses=[masses for _ in self.legs]
		if not isinstance(l_zeros, (list, tuple)):
			l_zeros=[l_zeros for _ in self.legs]
		#self.updateLegStates()
		self.sdm_systems=[SpringDamperMass(sc, m, l0, dc) for sc, m, l0, dc in zip(spring_constants, masses, l_zeros, damping_constants)]
		self.foot_positions=numpy.tile(numpy.array([numpy.nan,numpy.nan, numpy.nan]),(6,1))
		self.leg_stanced_in_last_iteration=[False]*6
		self.transformationMatrix=numpy.eye(4)
		
		
	def updateLegStates(self):
		#current_foot_positions=numpy.array(self.mrobot.wrobot.robot.getInputFootPositions())
		for leg_num, mleg in enumerate(self.mrobot.motivationNetLegs):
			inv_transformation_matrix=numpy.eye(4)
			if mleg.wleg.leg.leg_enabled and mleg.isStancing() and not self.leg_stanced_in_last_iteration[leg_num]:
				current_foot_position=self.mrobot.wrobot.robot.getInputFootPositionOfLeg(leg_num)
				self.foot_positions[leg_num,:]=current_foot_position
				self.sdm_systems[leg_num].position=current_foot_position[2]
				#if not inv_transformation_matrix:
				#	inv_transformation_matrix=numpy.linalg.inv(self.transformationMatrix)
				#temp=numpy.append(current_foot_position,1)
				#assumed_position_from_last_iteration=numpy.dot(inv_transformation_matrix, temp)
				self.sdm_systems[leg_num].velocity=0#current_foot_position[2]-assumed_position_from_last_iteration[2]
			elif mleg.isSwinging():
				self.leg_stanced_in_last_iteration[leg_num]=False
				
	def mmc_iteration_step(self):
		pass
	
	def computeNewFootPositions(self, stance_speed):
		self.updateLegStates()
		direction_vector=numpy.array([stance_speed/RSTATIC.controller_frequency,0,0])
		current_foot_positions=copy.copy(self.foot_positions)
		ind=numpy.logical_not(numpy.isnan(current_foot_positions[:,0])).nonzero()[0]
		if len(ind)==0: # if currently less than three legs are on the ground
			return self.foot_positions
		
		temp_new_foot_positions=copy.copy(current_foot_positions)	
		for leg_num, sdm in enumerate(self.sdm_systems):
			if not numpy.isnan(temp_new_foot_positions[leg_num,0]):
				temp_new_foot_positions[leg_num]-=direction_vector
				temp_new_foot_positions[leg_num,2]=sdm.computeNextPosition(1/RSTATIC.controller_frequency)
			
		old_stancing_foot_positions=current_foot_positions[ind,:]
		new_stancing_foot_positions=temp_new_foot_positions[ind,:]
		# Rotate/Translate the foot positions
		self.transformationMatrix=rigid_transform_3D(old_stancing_foot_positions, new_stancing_foot_positions)
		tempF=lambda x: numpy.dot(self.transformationMatrix,numpy.append(x,1))[0:3]
		new_stancing_foot_positions=numpy.apply_along_axis(tempF, 1, old_stancing_foot_positions)
		self.foot_positions[ind,:]=new_stancing_foot_positions
		# Compute the new positions and velocities of the sdm systems
		for i, ofp, nfp in zip(ind, old_stancing_foot_positions, new_stancing_foot_positions):
			self.sdm_systems[i].position=nfp[2]
			self.sdm_systems[i].velocity=nfp[2]-ofp[2]
		return self.foot_positions
	
	def getLegsRelativeTargetPosition(self, leg_name=None, leg_num=None):
		if not leg_num:
			leg_num=RSTATIC.leg_names.index(leg_name)
		if self.foot_positions[leg_num]!=None:
			return self.robot.transformFromRobotCoordinatesToLegCoordinates(leg_num, self.foot_positions[leg_num])
		else:
			return self.legs[leg_num].input_foot_position 

	def pullBodyModelAtFrontIntoRelativeDirection(self, pull_angle, speed_fact):
		self.computeNewFootPositions(speed_fact)
		
	def pullBodyModelAtBackIntoRelativeDirection(self, pull_angle, speed_fact):
		pass

class StanceMovementSpringDamperMassLeg(ModulatedRoutine):
	def __init__(self, motiv_leg):
		ModulatedRoutine.__init__(self)
		self.motivationNetLeg = motiv_leg
		self.leg=self.motivationNetLeg.wleg.leg
		self.bodyModelStance = self.motivationNetLeg.motivationNetRobot.bodyModelStance
		#self.mu_old_output_value = 0.
	
	def modulatedRoutineFunctionCall(self, weight):
		if (weight > 0.5) and (self.motivationNetLeg.wleg.leg.leg_enabled) :
			#if (self.mu_old_output_value <= 0.5):
				#self.bodyModelStance.put_leg_on_ground(self.motivationNetLeg.wleg.leg.name, \
			#		self.bodyModelStance.transform_robot_target_to_leg_vector(self.motivationNetLeg.wleg.leg.input_foot_position, self.motivationNetLeg.wleg.leg.name) )
			
			next_angles = self.leg.computeInverseKinematics(self.bodyModelStance.getLegsRelativeTargetPosition(leg_name=self.leg.name))
			current_angles = numpy.array( self.motivationNetLeg.wleg.leg.getInputAngles() )

			new_joint_velocities = (next_angles - current_angles.T)/(1/RSTATIC.controller_frequency)
			
			self.motivationNetLeg.wleg.addControlVelocities(new_joint_velocities)
		#self.mu_old_output_value = weight
		
# Input: expects Nx3 matrix of points
# Returns M
# M = 4x4 homogenous rotation/translation matrix
def rigid_transform_3D(A, B):
	assert numpy.all(A.shape == B.shape)
	N = A.shape[0]; # total points

	centroid_A = numpy.mean(A, axis=0)
	centroid_B = numpy.mean(B, axis=0)
	
	# centre the points
	AA = A - numpy.tile(centroid_A, (N, 1))
	BB = B - numpy.tile(centroid_B, (N, 1))

	# dot is matrix multiplication for array
	H = numpy.dot(numpy.transpose(AA), BB)
	U, S, Vt = numpy.linalg.svd(H)
	R = numpy.dot(Vt.T , U.T)
	# special reflection case
	if numpy.linalg.det(R) < 0:
	   Vt[2,:] *= -1
	   R = numpy.dot(Vt.T , U.T)
	t = numpy.dot(-R,centroid_A) + centroid_B
	M=numpy.zeros((4,4))
	M[0:3,0:3]=R
	M[0:3,3]=t.T
	#exit()
	return M
