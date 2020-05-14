/* File : mmcLeg.cxx */

#include "mmcLeg.h"
#include <iostream>
#include <boost/numeric/ublas/io.hpp>

MmcLeg::MmcLeg(std::vector<double> segm_l, double beta_d, double damp): 
		leg(-0.01, 0.01, 0.01), segm_length(3), temp_leg(0,0,0), damping(damp), 
		number_computations(0), iteration_step(0), return_vect(3) {
			
	leg.target <<= segm_l[0] + segm_l[1] + segm_l[2], 0., 0.;
	leg.projection_length = segm_l[0] + segm_l[1] + segm_l[2];
	leg.betaDirectionFact = beta_d;
	temp_leg = leg;
	
	segm_length <<= segm_l[0], segm_l[1], segm_l[2];
	
}

/**
 * Return the current joint angles.
 * After iteration of the network the joint variables can be returned.
 */
std::vector<double> MmcLeg::get_joint_angles() {
	return_vect[0] = leg.alpha;
	return_vect[1] = leg.betaDirectionFact * leg.beta;
	return_vect[2] = leg.betaDirectionFact * leg.gamma;
	return return_vect;
}

/**
 * Set new joint angles(as a list).
 * When the network shall be used to compute the forward kinematic function
 * the target joint angles are enforced onto the network before an iteration step
 * using this function.
 */
void MmcLeg::set_joint_angles(std::vector<double> target) {
	leg.alpha = target[0];
	leg.beta  = leg.betaDirectionFact * target[1];
	leg.gamma = leg.betaDirectionFact * target[2];
}

/**
 * Compute the joint angles.
 * Exploiting trigonometric relations the joint angles are calculated in
 * multiple ways. These computations would involve computing asin and acos
 * values. This is problematic as these are ambiguous. Therefore, we integrate
 * these computations in a first step by computing the quotient of the sine and
 * cosine value which equals the tangens value. From this we can compute the
 * unambiguous arc tangens value for the joint value.
 */
void MmcLeg::compute_joints_and_integrate()
{
	//Computation of the alpha angle
	calc_joint_value = -atan2(leg.target[1], leg.target[0]);
	number_computations = 1;
	if ((fabs(leg.projection_length) > 0.1) &&
		(fabs(leg.target[1]/leg.projection_length) < 1.0))
	{
		calc_joint_value -= asin(leg.target[1]/leg.projection_length);
		number_computations += 1;
	}
	// Sensor integration: if sensor input is provided it is added here
	/*if (len(self.current_sensor_angles[0]) > 0) :
		mc_joint += sum(self.current_sensor_angles[0])
		number_computations += 1
	*/
	// Recurrent connection
	temp_leg.alpha = (damping * leg.alpha + calc_joint_value)/(damping + number_computations);
	// Computation of the beta angle
	// Integration of two trigonometric relations
	calc_joint_value = atan2( (leg.target[2]
					+ segm_length[2] * cos(leg.beta - leg.gamma) ),
				(leg.projection_length - segm_length[0]
					- segm_length[2] * sin(leg.beta - leg.gamma) ) );
	number_computations = 1;
	/* Integration of sensor data if given
	if (len(self.current_sensor_angles[1]) > 0) :
		mc_joint += sum(self.current_sensor_angles[1])
		number_computations += 1 */
	// Recurrent connection
	temp_leg.beta = (damping * leg.beta + calc_joint_value)/(damping + number_computations);
	// Enforce joint constraints
	if (temp_leg.beta < 0.01) {
		temp_leg.beta = 0.01;
	}
	// Computation of the gamma angle
	// Integration of two trigonometric relations
	calc_joint_value = -atan2( (leg.target[2] - segm_length[1] * sin(leg.beta) ),
				(leg.projection_length - segm_length[0]
					- segm_length[1] * cos(leg.beta) )) + leg.beta - M_PI_2;
	number_computations = 1;
	/*// Integration of sensor data if given
	if (len(self.current_sensor_angles[2]) > 0) :
		mc_joint = sum(self.current_sensor_angles[2])
		number_computations += 1*/
	// Recurrent connection
	temp_leg.gamma = (damping * leg.gamma + calc_joint_value)/(damping + number_computations);
	if (temp_leg.gamma < -1.5608) {
		temp_leg.gamma = -1.5608;
	}
}

/**
 * Compute the target vector.
 * Application of basically the forward kinematics
 * but also includes recurrent connections (the old value is partially
 * maintained).
 */
void MmcLeg::compute_target() {
	//new_target = [0., 0., 0.]
	temp_leg.target <<= 0., 0., 0.;
	
	if (fabs(leg.alpha) > 0.1) {
		temp_leg.target[0] = (leg.target[1] / tan(-leg.alpha)
				+ leg.projection_length * cos(leg.alpha)
				+ damping * leg.target[0]) / (2 + damping);
	} else {
		temp_leg.target[0] = (leg.projection_length * cos(leg.alpha)
				+ damping * leg.target[0]) / (1 + damping);
	}
	temp_leg.target[1] = (leg.target[0] * tan(-leg.alpha)
			+ leg.projection_length * sin(-leg.alpha)
			+ damping * leg.target[1]) / (2 + damping);
	temp_leg.target[2] = (segm_length[1] * sin(leg.beta)
			- segm_length[2] * cos(leg.beta - leg.gamma)
			+ damping * leg.target[2]) / (1 + damping);
}

/**
 * Compute of the projection length of the leg.
 * The second and third joint work in a plane - spanned by the z-axis
 * and rotated around this axis. The overall length of the segment in this plane
 * is a projection of the x and y values onto this plane.
 */
void MmcLeg::compute_projection_length() {
	int equation_counter = 1;
	temp_leg.projection_length = segm_length[0] + segm_length[1] * cos(leg.beta)
			+ segm_length[2] * sin(leg.beta - leg.gamma);
	if (fabs(leg.alpha) > 0.1) {
		temp_leg.projection_length += leg.target[1] / sin (-leg.alpha);
		equation_counter ++;
	}
	if (fabs(leg.alpha) < 1.47) {
		temp_leg.projection_length += leg.target[0] / cos (leg.alpha);
		equation_counter++;
	}
	temp_leg.projection_length += damping * leg.projection_length;
	equation_counter += damping;
	temp_leg.projection_length /= equation_counter;
}

/**
 * The MMC Method:
 * - the multiple computations are computed for each variable
 * - the mean for each variable is calculated
 * The new values are appended to the list of element values.
 * For each variable new values are calculated through
 * different equations.
 */
void MmcLeg::mmc_kinematic_iteration_step() {
	compute_target();
	compute_projection_length();
	/*new_projection_lengths = self.compute_projection_length(-1)
		self.compute_leg_and_integrate(-1)
		if self.logging_values :
			self.leg_target.append(self.new_target)
			self.projection_length.append(new_projection_lengths)
			self.joint.append(self.temp_leg)
		else:
			self.leg_target[0] = self.new_target
			self.projection_length[0] = new_projection_lengths
			self.joint[0] = self.temp_leg
		self.step += 1
		self.current_sensor_angles = [[], [], []]*/
	compute_joints_and_integrate();
	leg = temp_leg;
	iteration_step++;
}

/**
 * Converge to inverse kinematic solution.
 * Access method for the stance network - for a given target position
 * the network converges over a couple of timesteps (around 5 sufficient, 10
 * very good) to a solution.
 */
std::vector<double> MmcLeg::compute_inverse_kinematics(std::vector<double> target) {
	int i;
	for(i=0; i<10; i++) {
		//set_leg_target( target_point )
		leg.target <<= target[0], target[1], target[2];
		mmc_kinematic_iteration_step();
	}
	return (this->get_joint_angles() );
}