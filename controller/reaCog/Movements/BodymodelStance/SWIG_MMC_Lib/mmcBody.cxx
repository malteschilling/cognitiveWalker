/* File : mmcBody.cxx */

#include "mmcBody.h"
#include <iostream>
#include <boost/numeric/ublas/io.hpp>

/**
 * A Body Model for a hexapod walker, based on MMC computation.
 * Now extended to 3 dimensions, (body segments are 3D and have now an orientation).
 *
 * Higher level of the hierarchical body model for a six legged robot:
 * On this level
 * 	- there are three segments described through each six vectors
 * 	- each leg is represented only as a vector to the tip of the leg
 * 		for each leg there are two vectors - one starting at the front of the segment
 * 		(the front vector) and one starting at the coxa (where the leg is mounted)
 * 		representing the leg (leg vector)
 * 		both ending at the tip of the leg
 * The body model is constructed as a Mean of Multiple Computation network:
 * 	- each variable vector is described by multiple kinematic relations
 * 		= computations
 * 	- these different computations are integrated using a mean calculation
 * The resulting set of equations directly defines a neural network connection matrix.
 * This can be computed in an iterative fashion - introduction of a recurrent connection
 * damps the overall relaxation procedure and prevents oscillations.
 * The network is able to compute forward, inverse and any mixed kinematic problem and
 * is quite fast (it only needs a few iteration steps).
 * It acts as an autoassociator - so a problem is given to the network as an incomplete
 * set of variables and the network fills in complementing missing values.
 * The network used here is special as it uses no specific reference coordinate system.
 * Described in the network are only relative vectors between certain body parts.
 * The footdiag vectors connecting the feet of the standing legs are actually
 * constituting a fixed reference frame in which coordinate systems and
 * representation of the environment can be grounded.
 * To move the body actually, there are special vectors in the body model which
 * explicitly represent a disturbance of the dynamics of the network (the pull vector
 * which is driving the delta vectors of the segments). One can envision this as
 * a passive motion paradigm: the body model is pulled into direction of this vector
 * and the model is following this movement (one can think of this as a stick model which
 * legs are attached to the ground).
 *
 * The body model is used in connection to leg models. The leg vectors are enforced onto
 * the lower leg networks which compute corresponding joint angles for the legs from
 * this. Overall this is embedded in a processing loop:
 * 	- the leg network is updated by sensory data
 * 		(iterates once to integrate the values)
 * 	- the leg network updates the body model
 * 	- the body model gets a movement command (usually constant)
 * 		and is iterated (one step is sufficient)
 * 	- the body model pushes down the new leg vectors for the standing legs into
 * 		the leg networks (these are iterated for a few iteration steps and provide
 * 		joint angles)
 * 	- the joint motors are controlled by the new motor commands
 */

MmcBody::MmcBody(double body_h_fr, double body_h_m, double body_h_h, double stance_w, double damp, double stab_thr): 
		legs(), segms(), temp_legs(), temp_segms(), segm1_in_global(3), 
		pull_front(3), pull_back(3), damping(damp), stability_threshold(stab_thr), 
		body_height_front(body_h_fr), body_height_middle(body_h_m), body_height_hind(body_h_h), stance_width(stance_w), 
		visualization_data(3, std::vector<double>(5) ), leg_return_vect(4) {
	// Init the vectors describing the legs.
	// Vectors from the front of the body segment to the foot point.
	legs[0].front_vect <<= -0.19, (stance_width + 0.07), -body_height_front;
	legs[1].front_vect <<= -0.19, -(stance_width + 0.07), -body_height_front;
	legs[2].front_vect <<= -0.33, (stance_width + 0.07), -body_height_middle;
	legs[3].front_vect <<= -0.33, -(stance_width + 0.07), -body_height_middle;
	legs[4].front_vect <<= -0.2, (stance_width + 0.07), -body_height_hind;
	legs[5].front_vect <<= -0.2, -(stance_width + 0.07), -body_height_hind;
	// Leg vector: from coxa to foot point
	legs[0].leg_vect <<= 0., stance_width, -body_height_front;
	legs[1].leg_vect <<= 0., -stance_width, -body_height_front;
	legs[2].leg_vect <<= 0., stance_width, -body_height_middle;
	legs[3].leg_vect <<= 0., -stance_width, -body_height_middle;
	legs[4].leg_vect <<= 0., stance_width, -body_height_hind;
	legs[5].leg_vect <<= 0., -stance_width, -body_height_hind;
	
	// Setting up the body segments (back to front):	
	segms[0].post_to_ant_vect <<= 0.22, 0., 0.;
	segms[1].post_to_ant_vect <<= 0.35, 0., (body_height_front-body_height_middle);
	segms[2].post_to_ant_vect <<= 0.36, 0., (body_height_middle-body_height_hind);
	
	// Setting up the global positions.
	// Overall the first segment is used for the spanned reference system.
	segm1_in_global <<= -0.02, 0.0, body_height_middle;
	// Construction of the global foot positions:
	legs[0].foot_global <<= segm1_in_global + segms[1].post_to_ant_vect 
			+ segms[0].post_to_ant_vect + legs[0].front_vect;
	legs[1].foot_global <<= segm1_in_global + segms[1].post_to_ant_vect 
			+ segms[0].post_to_ant_vect + legs[1].front_vect;
	legs[2].foot_global <<= segm1_in_global + segms[1].post_to_ant_vect + legs[2].front_vect;
	legs[3].foot_global <<= segm1_in_global + segms[1].post_to_ant_vect + legs[3].front_vect;
	legs[4].foot_global <<= segm1_in_global + legs[4].front_vect;
	legs[5].foot_global <<= segm1_in_global + legs[5].front_vect;
	
	int i;
	// Construction of the partial segments connected to the leg vectors.
	for (i=0; i<6; i++) {
		legs[i].segm_leg_ant_vect <<= legs[i].leg_vect - legs[i].front_vect;
		legs[i].segm_leg_ant_norm = norm_2(legs[i].segm_leg_ant_vect);
		legs[i].segm_leg_post_vect <<= legs[i].segm_leg_ant_vect - segms[i/2].post_to_ant_vect;
		legs[i].segm_leg_post_norm = norm_2(legs[i].segm_leg_post_vect);
		temp_legs[i] = legs[i];
	}
	// Setting up the remaining segment vectors.
	for (i=0; i<3; i++) {
		segms[i].diag_to_right_vect <<= legs[i*2].segm_leg_ant_vect - legs[i*2+1].segm_leg_ant_vect;
		segms[i].diag_to_right_norm = norm_2(segms[i].diag_to_right_vect);
		segms[i].post_to_ant_norm = norm_2(segms[i].post_to_ant_vect);
		temp_segms[i] = segms[i];
		segms[i].delta_front <<= 0., 0.0, 0.;
		segms[i].delta_back <<= 0., 0.0, 0.;
	}
	// Construction of the diagonal vectors between the feet:
	int start_leg, end_leg;
	for (start_leg = 0; start_leg < 6; start_leg++) {
		for (end_leg = 0; end_leg < start_leg; end_leg++) {
			foot_diag[start_leg][end_leg] = boost::numeric::ublas::vector<double>(3);
			this->get_segm_vectors_between_legs(start_leg, end_leg);
			foot_diag[start_leg][end_leg] <<= legs[end_leg].leg_vect - legs[start_leg].leg_vect + temp_vect;
		}
	}
	pull_front <<= 0., 0.0, 0.;
	pull_back <<= 0., 0.0, 0.;
}

// For two given legs the concatenated segment vectors are returned that connect the two
// leg vectors.
void MmcBody::get_segm_vectors_between_legs(int start_leg, int target_leg) {
	//boost::numeric::ublas::vector<double> segm_diff;
	temp_vect = boost::numeric::ublas::zero_vector<double>(3);
	switch (target_leg/2 - start_leg/2) {
		case 1:
			temp_vect += legs[start_leg].segm_leg_post_vect - legs[target_leg].segm_leg_ant_vect;
			break;
		case -1:
			temp_vect += legs[start_leg].segm_leg_ant_vect - legs[target_leg].segm_leg_post_vect;
			break;
		case 2:
			temp_vect += legs[start_leg].segm_leg_post_vect 
				- legs[target_leg].segm_leg_ant_vect - segms[1].post_to_ant_vect;
			break;
		case -2:
			temp_vect += legs[start_leg].segm_leg_ant_vect 
				- legs[target_leg].segm_leg_post_vect + segms[1].post_to_ant_vect;
			break;
		default:
			temp_vect += legs[start_leg].segm_leg_post_vect - legs[target_leg].segm_leg_post_vect;
	}
	//return temp_vect;
}

// For two given legs the concatenated segment vectors are returned that connect the two
// front vectors.
void MmcBody::get_segm_vectors_between_front(int start_leg, int target_leg) {
	//boost::numeric::ublas::vector<double> segm_diff;
	temp_vect = boost::numeric::ublas::zero_vector<double>(3);
	switch (target_leg/2 - start_leg/2) {
		case 1:
			temp_vect -= segms[start_leg/2].post_to_ant_vect;
			break;
		case -1:
			temp_vect += segms[target_leg/2].post_to_ant_vect;
			break;
		case 2:
			temp_vect -= segms[0].post_to_ant_vect + segms[1].post_to_ant_vect;;
			break;
		case -2:
			temp_vect += segms[0].post_to_ant_vect + segms[1].post_to_ant_vect;
			break;
	}
	//return temp_vect;
}

/**
 * Remove leg vector from body model when the leg leaves the ground.
 */
void MmcBody::lift_leg_from_ground(int leg_nr) {
	legs[leg_nr].gc = false;
}

bool MmcBody::get_ground_contact(int leg_nr) {
	return legs[leg_nr].gc;
}

/**
 * Put leg on ground.
 * Incorporate new leg vector into body model at touchdown.
 */
void MmcBody::put_leg_on_ground(int leg_nr, std::vector<double> leg_vec) {
	//std::cout << "PUT LEG ON GROUND " << (leg_nr) << std::endl;
	if (!(legs[leg_nr].gc)) {
		legs[leg_nr].leg_vect <<= leg_vec[0], leg_vec[1], leg_vec[2];
		legs[leg_nr].front_vect = legs[leg_nr].leg_vect - legs[leg_nr].segm_leg_ant_vect;
		// Construction of all foot vectors - the ones to legs in the air are not used!
		int i;
		for (i= 0; i < leg_nr; i++) {
			if (legs[i].gc) {
				//std::cout << "Build diagonal " << (i) << std::endl;
				foot_diag[leg_nr][i] = boost::numeric::ublas::vector<double>(3);
				this->get_segm_vectors_between_legs(leg_nr, i);
				foot_diag[leg_nr][i] <<= legs[i].leg_vect - legs[leg_nr].leg_vect + temp_vect;
			}
		}
		for (i=leg_nr+1; i < 6; i++) {
			if (legs[i].gc) {
				//std::cout << "Build diagonal " << (i) << std::endl;
				foot_diag[i][leg_nr] = boost::numeric::ublas::vector<double>(3);
				this->get_segm_vectors_between_legs(i, leg_nr);
				foot_diag[i][leg_nr] <<= legs[leg_nr].leg_vect - legs[i].leg_vect + temp_vect;
			}
		}	
		// Derive the global position (needed for the graphics output)
		i=0;
		while ( (i<6) and !(legs[i].gc) ) {
			i++;
		}
		if (i<6) {
			if (i > leg_nr) {
				legs[leg_nr].foot_global = legs[i].foot_global + foot_diag[i][leg_nr];
			} else {
				legs[leg_nr].foot_global = legs[i].foot_global - foot_diag[leg_nr][i];
			}
		}
		legs[leg_nr].gc = true;
	}
}

/**
 * Compute the leg vectors: For all standing legs
 * the new leg vectors are computed, summed and the mean is calculated
 * (the old value is also integrated, weighted by the damping value).
 */
void MmcBody::compute_leg_computations_and_integrate(int leg_nr) {
	int equation_counter = 1;
	temp_legs[leg_nr].leg_vect <<= -segms[leg_nr/2].delta_back + legs[leg_nr].segm_leg_post_vect
		+ segms[leg_nr/2].post_to_ant_vect + legs[leg_nr].front_vect;
	temp_legs[leg_nr].leg_vect += damping * legs[leg_nr].leg_vect;
	equation_counter += damping;
	int target_leg;
	for (target_leg = 0; target_leg < 6; target_leg++) {
		if ((legs[target_leg].gc) and (target_leg != leg_nr)) {
			this->get_segm_vectors_between_legs(leg_nr, target_leg);
			temp_legs[leg_nr].leg_vect += legs[target_leg].leg_vect + temp_vect;
			if (target_leg < leg_nr) {
				temp_legs[leg_nr].leg_vect -= foot_diag[leg_nr][target_leg];
			} else {
				temp_legs[leg_nr].leg_vect += foot_diag[target_leg][leg_nr];
			}
			equation_counter++;
		}
	}
	temp_legs[leg_nr].leg_vect /= equation_counter;
};

/**
 * Compute the front vectors: For all standing legs
 * the new help vectors from front to footpoint are computed, 
 * summed and the mean is calculated
 * (the old value is also integrated, weighted by the damping value)
 */
void MmcBody::compute_front_computations_and_integrate(int leg_nr) {
	int equation_counter = 1;
	temp_legs[leg_nr].front_vect <<= legs[leg_nr].leg_vect - legs[leg_nr].segm_leg_post_vect 
		- segms[leg_nr/2].delta_front - segms[leg_nr/2].post_to_ant_vect;
	temp_legs[leg_nr].front_vect += damping * legs[leg_nr].front_vect;
	equation_counter += damping;
	int target_leg;
	for (target_leg = 0; target_leg < 6; target_leg++) {
		if ((legs[target_leg].gc) and (target_leg != leg_nr)) {
			this->get_segm_vectors_between_front(leg_nr, target_leg);
			temp_legs[leg_nr].front_vect += legs[target_leg].front_vect + temp_vect;
			if (target_leg < leg_nr) {
				temp_legs[leg_nr].front_vect -= foot_diag[leg_nr][target_leg];
			} else {
				temp_legs[leg_nr].front_vect += foot_diag[target_leg][leg_nr];
			}
			equation_counter++;
		}
	}
	temp_legs[leg_nr].front_vect /= equation_counter;
}

/**
 * Compute the segment vectors:
 * Using equations including the two legs connected to the segment,
 * integrating the explicit displacement given as delta
 * and the recurrent old value of the vector.
 */
void MmcBody::compute_segment_leg_ant_computations_and_integrate(int leg_nr) {
	//int equation_counter = 1;
	temp_legs[leg_nr].segm_leg_ant_vect <<= legs[leg_nr].segm_leg_post_vect + segms[leg_nr/2].post_to_ant_vect;
	temp_legs[leg_nr].segm_leg_ant_vect += damping * legs[leg_nr].segm_leg_ant_vect;
	//equation_counter += damping;
	temp_legs[leg_nr].segm_leg_ant_vect *= legs[leg_nr].segm_leg_ant_norm/norm_2(temp_legs[leg_nr].segm_leg_ant_vect);
}

/**
 * Compute the segment vectors:
 * Using equations including the two legs connected to the segment,
 * integrating the explicit displacement given as delta
 * and the recurrent old value of the vector.
 */
void MmcBody::compute_segment_leg_post_computations_and_integrate(int leg_nr) {
	temp_legs[leg_nr].segm_leg_post_vect <<= legs[leg_nr].segm_leg_ant_vect 
		- segms[leg_nr/2].post_to_ant_vect;
	temp_legs[leg_nr].segm_leg_post_vect += damping * legs[leg_nr].segm_leg_post_vect;
	temp_legs[leg_nr].segm_leg_post_vect *= legs[leg_nr].segm_leg_post_norm/norm_2(temp_legs[leg_nr].segm_leg_post_vect);
}

void MmcBody::compute_segm_post_ant_computations_and_integrate(int seg_nr) {
	temp_segms[seg_nr].post_to_ant_vect <<= segms[seg_nr].post_to_ant_vect + segms[seg_nr].delta_front - segms[seg_nr].delta_back;
	temp_segms[seg_nr].post_to_ant_vect += segms[seg_nr].diag_to_right_vect 
		- legs[seg_nr*2].segm_leg_post_vect + legs[1 + seg_nr*2].segm_leg_ant_vect;
	temp_segms[seg_nr].post_to_ant_vect += legs[seg_nr*2].segm_leg_ant_vect 
		- segms[seg_nr].diag_to_right_vect - legs[1+seg_nr*2].segm_leg_post_vect;
	temp_segms[seg_nr].post_to_ant_vect += damping * segms[seg_nr].post_to_ant_vect;
	temp_segms[seg_nr].post_to_ant_vect *= segms[seg_nr].post_to_ant_norm/norm_2(temp_segms[seg_nr].post_to_ant_vect);
}

void MmcBody::compute_segm_diag_computations_and_integrate(int seg_nr) {

	//using namespace std::chrono;
	//high_resolution_clock::time_point t1 = high_resolution_clock::now();
//	int equation_counter = 2;
	temp_segms[seg_nr].diag_to_right_vect <<= legs[seg_nr*2].segm_leg_post_vect 
		+ segms[seg_nr].post_to_ant_vect - legs[1+seg_nr*2].segm_leg_ant_vect;
	temp_segms[seg_nr].diag_to_right_vect -= legs[1+seg_nr*2].segm_leg_post_vect 
		+ segms[seg_nr].post_to_ant_vect - legs[seg_nr*2].segm_leg_ant_vect;
	temp_segms[seg_nr].diag_to_right_vect += damping * segms[seg_nr].diag_to_right_vect;
//	equation_counter += damping;
//	temp_segms[seg_nr].diag_to_right_vect /= equation_counter;
	temp_segms[seg_nr].diag_to_right_vect *= segms[seg_nr].diag_to_right_norm/norm_2(temp_segms[seg_nr].diag_to_right_vect);

	//high_resolution_clock::time_point t2 = high_resolution_clock::now();
	//duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
	//std::cout << "It took me " << time_span.count() << " seconds.";
	//std::cout << std::endl;
};

void MmcBody::set_pull_front(double pull_angle, double speed_fact) {
	double pull_angle_BM = pull_angle + atan2( segms[0].post_to_ant_vect[1], segms[0].post_to_ant_vect[0]);
	pull_front <<= (speed_fact * cos(pull_angle_BM)), (speed_fact * sin(pull_angle_BM)), 0.;
}

void MmcBody::set_pull_back(double pull_angle, double speed_fact) {
	double pull_angle_BM = pull_angle + atan2( -segms[2].post_to_ant_vect[1], -segms[2].post_to_ant_vect[0]);
	pull_front <<= (speed_fact * cos(pull_angle_BM)), (speed_fact * sin(pull_angle_BM)), 0.;
}

double MmcBody::check_static_stability(int left_leg, int right_leg) {
	bool stability = true;
	temp_vect = boost::numeric::ublas::zero_vector<double>(3);
	temp_vect_2 = boost::numeric::ublas::zero_vector<double>(3);
	// Loads the segm vector into temp_vect
	this->get_segm_vectors_between_front(left_leg, right_leg);
	// temp_vect_2 was diag_vect in python
	temp_vect_2 = -legs[left_leg].front_vect + legs[right_leg].front_vect + temp_vect; 
		
	// temp_vect was left_foot_vect on ground in python
	temp_vect = -legs[left_leg].front_vect;
	if (left_leg == 2) {
		temp_vect = -legs[left_leg].front_vect - segms[1].post_to_ant_vect;
	}
	if (left_leg == 0) {
		temp_vect = -legs[left_leg].front_vect - segms[1].post_to_ant_vect - segms[0].post_to_ant_vect;
	}	
	temp_vect[2] = 0.;
//	std::cout << "Stab  CPP: " << left_leg << " - " << right_leg << " = " << temp_vect_2 << " ; " << temp_vect << std::endl;
//	std::cout << "Hindlegs: " << legs[4].front_vect << " - " << legs[5].front_vect << std::endl;
//	std::cout << "Hindlegs: " << legs[4].leg_vect << " - " << legs[5].leg_vect << std::endl;
	
//	std::cout << "FOOT_DIAG " << foot_diag[right_leg][left_leg] << std::endl;
	double segment_factor = (temp_vect_2[1]*temp_vect[0] - temp_vect_2[0]*temp_vect[1]) / 
			(temp_vect_2[0]*segms[1].post_to_ant_vect[1] - temp_vect_2[1]*segms[1].post_to_ant_vect[0]);
			
	// Correction factor of the parameter:
	// If the most hind leg is a middle leg, the factor has to be increased by one
	// - if both are front legs, it has to be increased by two.
/* For legs further to the front: is now already counteracted above = in left_foot_cog_vect

	if (std::max(left_leg,right_leg) < 4) {
		segment_factor +=1;
		if (std::max(left_leg,right_leg) < 2) {
			segment_factor +=1;
		}
	}
*/
//	std::cout << "Stability Problem Detector " << segment_factor << std::endl;
/*	if (segment_factor > stability_threshold) {
		std::cout << "Stability Problem Detected " << segment_factor << std::endl;
		stability = false;
	}*/
	return segment_factor;
}

/**
 * The MMC Method:
 *	- the multiple computations are computed for each variable
 *	- the mean for each variable is calculated
 * The new values are appended to the list of element values.
 * For each variable new values are calculated through different equations.
 */
void MmcBody::mmc_iteration_step()
{
	segms[0].delta_front = pull_front;
	segms[1].delta_front = (legs[0].leg_vect - legs[0].segm_leg_post_vect 
		- legs[0].front_vect - segms[0].post_to_ant_vect);
	segms[2].delta_front = (legs[2].leg_vect - legs[2].segm_leg_post_vect 
		- legs[2].front_vect - segms[1].post_to_ant_vect);
		
	segms[2].delta_back = pull_back;
	segms[1].delta_back = -(legs[4].leg_vect - legs[4].segm_leg_post_vect 
		- legs[4].front_vect - segms[2].post_to_ant_vect);
	segms[0].delta_back = -(legs[2].leg_vect - legs[2].segm_leg_post_vect 
		- legs[2].front_vect - segms[1].post_to_ant_vect);
		
	int i;
	for (i=0; i<6; i++) {
		compute_front_computations_and_integrate(i);
		compute_leg_computations_and_integrate(i);
		compute_segment_leg_ant_computations_and_integrate(i);
		compute_segment_leg_post_computations_and_integrate(i);
	}
	for (i=0; i<3; i++) {
		compute_segm_diag_computations_and_integrate(i);
		compute_segm_post_ant_computations_and_integrate(i);
	}
	// When the intersegment drives are stiff the second and third segment
	// are simply projected on the scaled vector of the first segment_factor
	//temp_segms[1].post_to_ant_vect = temp_segms[0].post_to_ant_vect * (segms[1].post_to_ant_norm/norm_2(temp_segms[0].post_to_ant_vect));
	//temp_segms[2].post_to_ant_vect = temp_segms[0].post_to_ant_vect * (segms[2].post_to_ant_norm/norm_2(temp_segms[0].post_to_ant_vect));
	for (i=0; i<6; i++) {
		legs[i].front_vect <<= temp_legs[i].front_vect;
		legs[i].leg_vect <<= temp_legs[i].leg_vect;
		legs[i].segm_leg_ant_vect <<= temp_legs[i].segm_leg_ant_vect;
		legs[i].segm_leg_post_vect <<= temp_legs[i].segm_leg_post_vect;
	}
	for (i=0; i<3; i++) {
		segms[i].diag_to_right_vect <<= temp_segms[i].diag_to_right_vect;
		segms[i].post_to_ant_vect <<= temp_segms[i].post_to_ant_vect;
	}
};

/*
 * Return vector representing robot leg (4-d)
 */
std::vector<double> MmcBody::get_leg_vector(int leg_nr) {
	leg_return_vect[0] = legs[leg_nr].leg_vect[0];
	leg_return_vect[1] = legs[leg_nr].leg_vect[1];
	leg_return_vect[2] = legs[leg_nr].leg_vect[2];
	return leg_return_vect;
}

std::vector< std::vector<double> > MmcBody::get_leg_triangle(int leg_nr) {
	visualization_data[0] = { legs[leg_nr].foot_global[0],
		legs[leg_nr].foot_global[0] - legs[leg_nr].leg_vect[0],
		legs[leg_nr].foot_global[0] - legs[leg_nr].leg_vect[0] + legs[leg_nr].segm_leg_post_vect[0],
		legs[leg_nr].foot_global[0] - legs[leg_nr].leg_vect[0] + legs[leg_nr].segm_leg_post_vect[0] + segms[leg_nr/2].post_to_ant_vect[0],
		legs[leg_nr].foot_global[0] - legs[leg_nr].leg_vect[0] };
	visualization_data[1] = { legs[leg_nr].foot_global[1],
		legs[leg_nr].foot_global[1] - legs[leg_nr].leg_vect[1],
		legs[leg_nr].foot_global[1] - legs[leg_nr].leg_vect[1] + legs[leg_nr].segm_leg_post_vect[1],
		legs[leg_nr].foot_global[1] - legs[leg_nr].leg_vect[1] + legs[leg_nr].segm_leg_post_vect[1] + segms[leg_nr/2].post_to_ant_vect[1],
		legs[leg_nr].foot_global[1] - legs[leg_nr].leg_vect[1] };
	visualization_data[2] = { legs[leg_nr].foot_global[2],
		legs[leg_nr].foot_global[2] - legs[leg_nr].leg_vect[2],
		legs[leg_nr].foot_global[2] - legs[leg_nr].leg_vect[2] + legs[leg_nr].segm_leg_post_vect[2],
		legs[leg_nr].foot_global[2] - legs[leg_nr].leg_vect[2] + legs[leg_nr].segm_leg_post_vect[2] + segms[leg_nr/2].post_to_ant_vect[2],
		legs[leg_nr].foot_global[2] - legs[leg_nr].leg_vect[2] };
	return visualization_data;
}