/* File : mmcBody.h */
#include <math.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/assignment.hpp>

struct LEG_VECTORS {
	LEG_VECTORS():
		front_vect(3), leg_vect(3), segm_leg_ant_vect(3), segm_leg_post_vect(3), foot_global(3), gc(false)
	{}
	
    boost::numeric::ublas::vector<double> front_vect;
    boost::numeric::ublas::vector<double> leg_vect;
    
    boost::numeric::ublas::vector<double> segm_leg_ant_vect;
    double segm_leg_ant_norm;
    boost::numeric::ublas::vector<double> segm_leg_post_vect;
    double segm_leg_post_norm;
    
    boost::numeric::ublas::vector<double> foot_global;
    bool gc;
};

struct SEGM_VECTORS {
	SEGM_VECTORS():
		diag_to_right_vect(3), post_to_ant_vect(3), delta_front(3), delta_back(3)
	{}
	
    boost::numeric::ublas::vector<double> diag_to_right_vect;
    double diag_to_right_norm;
    boost::numeric::ublas::vector<double> post_to_ant_vect;
    double post_to_ant_norm;
    
    boost::numeric::ublas::vector<double> delta_front;
    boost::numeric::ublas::vector<double> delta_back;
};


class MmcBody {
	public:
		MmcBody(double body_h_front, double body_h_middle, double body_h_hind,
		    double stance_w, double damp, double stab_thr);

		virtual ~MmcBody() {
		};
		
	LEG_VECTORS legs[6];
	SEGM_VECTORS segms[3];
	LEG_VECTORS temp_legs[6];
	SEGM_VECTORS temp_segms[3];
	
	boost::numeric::ublas::vector<double> segm1_in_global;
	
	boost::numeric::ublas::vector<double> foot_diag[6][6];
	
	boost::numeric::ublas::vector<double> pull_front;
	boost::numeric::ublas::vector<double> pull_back;
	boost::numeric::ublas::vector<double> temp_vect, temp_vect_2;
	
	std::vector< std::vector<double> > visualization_data;
	std::vector<double> leg_return_vect;
	
	double damping;
	double body_height_front;
	double body_height_middle;
	double body_height_hind;
	double stance_width;
	double stability_threshold;
    
	// For two given legs the concatenated segment vectors are returned that connect the two
	// leg vectors.
    void get_segm_vectors_between_legs (int start_leg, int target_leg);
    
    // For two given legs the concatenated segment vectors are returned that connect the two
	// front vectors.
	void get_segm_vectors_between_front(int start_leg, int target_leg);
    
    /**
     * Remove leg vector from body model when the leg leaves the ground.
     */
	void lift_leg_from_ground(int leg_nr);
	
	/**
	 * Put leg on ground.
	 * Incorporate new leg vector into body model at touchdown.
	 */
	void put_leg_on_ground(int leg_nr, std::vector<double> leg_vec);
	
	/**
	 * Test ground contact in the body model.
	 */
	bool get_ground_contact(int leg_nr);
    
    /**
     * Compute the leg vectors: For all standing legs
	 * the new leg vectors are computed, summed and the mean is calculated
	 * (the old value is also integrated, weighted by the damping value).
	 */
	void compute_leg_computations_and_integrate(int leg_nr);
	
	/**
	 * Compute the front vectors: For all standing legs
	 * the new help vectors from front to footpoint are computed, 
	 * summed and the mean is calculated
	 * (the old value is also integrated, weighted by the damping value)
	 */
	void compute_front_computations_and_integrate(int leg_nr);
	
	/**
	 * Compute the segment vectors:
	 * Using equations including the two legs connected to the segment,
	 * integrating the explicit displacement given as delta
	 * and the recurrent old value of the vector.
	 */
	void compute_segment_leg_ant_computations_and_integrate(int leg_nr);
	
	/**
	 * Compute the segment vectors:
	 * Using equations including the two legs connected to the segment,
	 * integrating the explicit displacement given as delta
	 * and the recurrent old value of the vector.
	 */
	void compute_segment_leg_post_computations_and_integrate(int leg_nr);
	
	void compute_segm_post_ant_computations_and_integrate(int seg_nr);
	
    void compute_segm_diag_computations_and_integrate(int seg_nr);
    
    void set_pull_front(double pull_angle, double speed_fact);
    
    void set_pull_back(double pull_angle, double speed_fact);
    
    double check_static_stability(int left_leg, int right_leg);
    
    /**
	 * The MMC Method:
	 *	- the multiple computations are computed for each variable
	 *	- the mean for each variable is calculated
	 * The new values are appended to the list of element values.
	 * For each variable new values are calculated through different equations.
	 */
    void mmc_iteration_step(); 

    std::vector< std::vector<double> > get_leg_triangle(int leg_nr);
	
	/*
	 * Return vector representing robot leg (4-d)
	 */
    std::vector<double> get_leg_vector(int leg_nr);
};