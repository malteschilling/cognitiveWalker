/* File : mmcLeg.h */
#include <math.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/assignment.hpp>

struct LEG_VARS{
	LEG_VARS(double a, double b, double c):
		alpha(a), beta(b), gamma(c), projection_length(0.), target(3), betaDirectionFact(1)
	{}
	
    double alpha;
    double beta;
    double gamma;
    
    double projection_length;
    
    boost::numeric::ublas::vector<double> target;
    
    int betaDirectionFact;
    
};

/*struct MMC_VECTOR {
	MMC_VECTOR(double a, double b, double c):
		x(a), y(b), z(c)
	{}
	
    double x;
    double y;
    double z;
};*/

class MmcLeg {
	public:
		MmcLeg(std::vector<double> segm_l, double beta_d, double damp);

		virtual ~MmcLeg() {
		};
		
	LEG_VARS leg, temp_leg;
	boost::numeric::ublas::vector<double> segm_length;
	double damping;
	double calc_joint_value;
	int number_computations, iteration_step;
	std::vector<double> return_vect;

	/**
	 * Return the current joint angles.
	 * After iteration of the network the joint variables can be returned.
	 */
	std::vector<double> get_joint_angles();

	/**
	 * Set new joint angles(as a list).
	 * When the network shall be used to compute the forward kinematic function
	 * the target joint angles are enforced onto the network before an iteration step
	 * using this function.
	 */
	void set_joint_angles(std::vector<double> target);
    
	/** Compute the joint angles.
	 * Exploiting trigonometric relations the joint angles are calculated in
	 * multiple ways. These computations would involve computing asin and acos
	 * values. This is problematic as these are ambiguous. Therefore, we integrate
	 * these computations in a first step by computing the quotient of the sine and
	 * cosine value which equals the tangens value. From this we can compute the
	 * unambiguous arc tangens value for the joint value.
	 */
    void compute_joints_and_integrate();
    
	/**	Compute the target vector.
	 * Application of basically the forward kinematics
	 * - but also includes recurrent connections (the old value is partially
	 * maintained).
	 */
    void compute_target();
    
    /**
	 * Compute of the projection length of the leg.
	 * The second and third joint work in a plane - spanned by the z-axis
	 * and rotated around this axis. The overall length of the segment in this plane
	 * is a projection of the x and y values onto this plane.
	 */
    void compute_projection_length();
    
    /**
	 * The MMC Method:
	 * - the multiple computations are computed for each variable
	 * - the mean for each variable is calculated
	 * The new values are appended to the list of element values.
	 * For each variable new values are calculated through
	 * different equations.
	 */
	void mmc_kinematic_iteration_step();
	
	/**
	 * Converge to inverse kinematic solution.
	 * Access method for the stance network - for a given target position
	 * the network converges over a couple of timesteps (around 5 sufficient, 10
	 * very good) to a solution.
	 */
	std::vector<double> compute_inverse_kinematics(std::vector<double> target);
};