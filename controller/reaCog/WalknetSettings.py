## General parameters for the walknet controller.

import numpy
# Select whether the neural networks should be evaluated in a parallel or serial way.
# If using the serial version, the neural networks responsible for the leg coordination will be evaluated after each other based on the order defined in walknet.py.
# Please note that the serial evaluation may not lead to fundamentally different results as compared to the parallel version. Also, there are concerns about the legitimacy of this concept!
use_serial_evaluation_of_leg_controllers=False

## Safety settings
# Setting this flag to True will enable stability checks. This makes sure that a leg that intends to change from stance to swing phase is not needed to sustain the static stability.
# If a leg is required to prohibit instability, its stance phase will be prolonged. 
# As a threshold for the prohibition of the swing phase, the "upper_stability_threshold" is used. 
prohibit_swing_if_unstable = False

##
# Allow for searching movements when not finding gc at end of swing:
searching_movements = False

# Setting this flag to True will enable a safety feature that reduces the stance speed (not the swing speed!) of the robot when the center of mass approaches the support polygon.
# The closer the center of mass comes to the support polygon, the slower the robot will stance. The safety margin for which the deacceleration will start, 
# is defined in "upper_stability_threshold". The safety margin for which the robot will come to a halt is defined in "lower_stability_threshold".
# If the stability margin is suddenly increased (for example due to a leg that switched from swing to stance movement), the robot will accelerate from halt to 
# full speed within the time defined in "acceleration_time_for_increased_stability". If the robot did not stand (stance velocity!=0), the acceleration time is 
# computed proportionately (it will take half the time to accelerate from half speed to full speed). 
# 
reduce_speed_for_reduced_stability=False


# These are the thresholds for the stability module. They are used both for the "prohibit_swing_if_unstable" and for the "reduce_speed_for_reduced_stability".
# For the functional description, see the comments of these modules.
upper_stability_threshold=0.1
lower_stability_threshold=0.05

# Setting this flag to True will activate a security feature that will reduce the stance speed if one of the legs comes close to the border of its working space.
# The "workspace_threshold" gives the minimum distance from the border for which full speed is possible. From this point till the border, the stance speed is reduced
# linearly. 
reduce_speed_if_close_to_workspace_border=False
workspace_threshold=0.05

# Time within which the robot will accelerate from halt to full speed after one of the security features has reduced the stance speed.
# The actual acceleration time is linearly depended of the speed reduction: If the robot walked with half speed due to a security feature, 
# it will take half the time to accelerate it to full speed.  
acceleration_time_for_increased_stability=1

# This flag determines whether the center of mass is computed in each iteration or whether a precomputed constant value will be used 
# (e.g. in order to reduce the computational load).
use_assumed_center_of_mass=False
# This is the constant position of the center of mass that will be used if "use_assumed_center_of_mass" is True. 
assumed_center_of_mass=numpy.array([0.215,0,0.05])

# When an instability is detected the whole walker is slowed down for a certain time
# This parameter defines the number of iterations for this slow down.
unstable_slow_down_window = 100
safe_stop_time = 50

#== Input simulation parameters =====
#=====================================
# Define a walking speed.
default_speed = 0.020 # 0.018
max_speed=0.1
# Give a direction (an angle in degrees, should be quite low = 15)
default_direction_angle = 0. # 12.
max_abs_direction_angle=10/360*numpy.pi

# Set the swing trajectory generator
#Options are: 'bezier', 'quadratic_spline'
swing_trajectory_generator='quadratic_spline'

# Set the stance trajectory generator
#Options are: 'bodymodel', 'springdampermass'
stance_trajectory_generator='bodymodel'
# The stability is determined in the following way:
# The connection between the two most hind legs touching the ground
# (one leg on each side) is constructing a vector to the back.
# The segment_factor is the fraction of the last body segment meeting this
# vector (counted from in between middle and hind segment):
# 	0 = between the two segments (0 part of the hind segment)
#	-1 = at the end of the hind segment_factor
# The robot is determined instable when the segment_factor is greater
# than this threshold.
# Was -0.26
# -0.1 = 10% of hind segment = 10 cm behind CoG
stability_threshold = -0.16#0.125 # On robot was -0.32, use -0.16
stability_threshold_front = 0.125 
stability_cog = 0.125
stability_cog_threshold = 0.28

# Settings for the bezier-based swing trajectory generator
default_swing_velocity=0.28 # was 0.4

# This defines the minimum distance between the AEP and the PEP. If the PEP_shift is bigger than |AEP-PEP|, is will be clipped to |AEP-PEP|-minimum_swing_distance
minimum_swing_distance=0.05
# This defines the minimum time a swing movement will be executed. The value is used be the behindPepSensorUnit as a minimum activation time:
# The unit will remain active once it has been activated by input for the specified duration. Therefore, even if the leg
# encounters ground contact, the leg will remain in swing phase!
minimum_swing_time=0.2 # was 0.2
default_apex_point_offset=numpy.array([0,0,0.2])

#== Ground Contact Parameters ========
#=====================================
# Parameter for prediction based method
# parameter describes predicted height value from which on 
# a leg is predicted as in gc - is given as a percentage
# (good value: 0.8, means when leg is having a height of 0.8 * the 
# intended height control value it is already assumed as having ground contact
predicted_ground_contact_height_factor = 0.85

# Parameter for tarsus pressure sensor
#ground contact (gC) detection by force threshold:
tarsusPressureGCThreshold = 2.

#== Body Model Parameters ============
#=====================================
""" Parameter for the stance network
	Defines how the stance movement itself is calculated on
	the leg level.
	The body model provides target vectors for the stance networks
	and in the stance network there are different types which can provide
	joint angles.
	0.	direct inverse kinematics (fastest, but not neural network)
	1.	MMC network using dual quaternions (slow)
	2.	angular MMC network (fast convergent network)
"""
stanceLegLevelMethod = 0

# Logging of the movement data 
logging_values = False

#=====================================

#userBodyModelToControll = True

#overwrites joystick input and forces the robot to walk in a circle
sidewaytest = False

sidewayenabled = False

#===== output simulation parameters =====

visualization = False

# Flag distinguishing robot or simulation 
# Only used for visualization purposes
real_robot = False

#===== leg walking parameters =====
stanceheight = 0.2#0.18
stancewidth = 0.27*0.9
default_stance_distance=0.16

# front_initial_aep = numpy.array([0.16,  -stancewidth, -stanceheight])  # for forward walking
# front_initial_pep = numpy.array([front_initial_aep[0]-0.20,  -stancewidth, -stanceheight]) # for forward walking
# middle_initial_aep =numpy.array([0.05,  -stancewidth, -stanceheight]) # for forward walking
# middle_initial_pep = numpy.array([middle_initial_aep[0]-0.15,  -stancewidth, -stanceheight]) # -0.07# for forward walking
# hind_initial_aep = numpy.array([-0.08,  -stancewidth, -stanceheight])
# hind_initial_pep =numpy.array([hind_initial_aep[0]-0.15,  -stancewidth, -stanceheight])  # -0.21

# front_initial_aep = numpy.array([0.12,  -stancewidth, -1.1*stanceheight])  # for forward walking
# front_initial_pep = numpy.array([front_initial_aep[0]-0.2,  -stancewidth, -1.1*stanceheight]) # for forward walking
# middle_initial_aep =numpy.array([0.08,  -stancewidth, -stanceheight]) # for forward walking
# middle_initial_pep = numpy.array([middle_initial_aep[0]-0.2,  -stancewidth, -stanceheight]) # -0.07# for forward walking
# hind_initial_aep = numpy.array([-0.08,  -stancewidth, -stanceheight])
# hind_initial_pep =numpy.array([hind_initial_aep[0]-0.2,  -stancewidth, -stanceheight]) 

# GOLD 2018-09-09
#front_initial_aep = numpy.array([0.08,  -stancewidth, -1.1*stanceheight])  # for forward walking
#front_initial_pep = numpy.array([front_initial_aep[0]-0.2,  -stancewidth, -1.1*stanceheight]) # for forward walking
#middle_initial_aep =numpy.array([0.1,  -stancewidth, -stanceheight]) # for forward walking
#middle_initial_pep = numpy.array([middle_initial_aep[0]-0.2,  -stancewidth, -stanceheight]) # -0.07# for forward walking
#hind_initial_aep = numpy.array([-0.08,  -stancewidth, -stanceheight])
#hind_initial_pep =numpy.array([hind_initial_aep[0]-0.2,  -stancewidth, -stanceheight]) 

front_initial_aep = numpy.array([0.12,  -stancewidth, -1.1*stanceheight])  # for forward walking
front_initial_pep = numpy.array([front_initial_aep[0]-default_stance_distance,  -stancewidth, -1.1*stanceheight]) # for forward walking
middle_initial_aep =numpy.array([0.08,  -stancewidth, -stanceheight]) # for forward walking
middle_initial_pep = numpy.array([middle_initial_aep[0]-default_stance_distance,  -stancewidth, -stanceheight]) # -0.07# for forward walking
hind_initial_aep = numpy.array([-0.0,  -stancewidth, -stanceheight])
hind_initial_pep =numpy.array([hind_initial_aep[0]-default_stance_distance,  -stancewidth, -stanceheight]) 

# Extreme positions used on robot.
# front_initial_aep = numpy.array([0.5*default_stance_distance,  -stancewidth, -stanceheight])  # for forward walking
# front_initial_pep = numpy.array([-0.5*default_stance_distance,  -stancewidth, -stanceheight]) # for forward walking
# middle_initial_aep =numpy.array([default_stance_distance/2,  -stancewidth, -stanceheight]) # for forward walking
# middle_initial_pep = numpy.array([-default_stance_distance/2,  -stancewidth, -stanceheight]) # -0.07# for forward walking
# hind_initial_aep = numpy.array([0.25*default_stance_distance,  -stancewidth, -stanceheight])
# hind_initial_pep =numpy.array([-0.75*default_stance_distance,  -stancewidth, -stanceheight])  # -0.21

#===== explicit swing controller ====
controlViaPI = False
kp = 0.95	# P-controller factor
ki = 0.005	# I-controller factor


#relative position of the dep between the aep and pep seen from the pep
deprelation = 1/3;
#~ depheight = 0.2; # IMO that's a bit too high.
#depheight = -0.05; # sufficient for flat terrain
#plane on which there is the dep relative to the aep pep axis
# depx = 0 # an x-shift makes no sense (fw vs bw); use deprelation
# depy = 0.30; # invert for right legs # will be set by WLeg>depcalc
depz = -0.08; #height of the dep point in a swing trajectory
#depheight = depz # for backwards compatibility, TODO: remove
dep_normal_vector_z_slope = -0.2 # indirectly determines depy, see depcalc

#===== swing-net related values ====
swingNet_hpGammaVelos = {"front_left_leg": 150, "middle_left_leg": 150, "hind_left_leg":1, "front_right_leg": 150, "middle_right_leg": 150, "hind_right_leg": 1} #  High-pass velocity factor for the gamma joint
    
# List of individual parameters for the swing net - depending
# on which leg is used.
# High-pass factors for the beta joint
swingNet_hpTimeConst = {"front_left_leg": 3, "middle_left_leg": 3, "hind_left_leg": 3, "front_right_leg": 3, "middle_right_leg": 3, "hind_right_leg": 3} 
swingNet_hpVelocity = {"front_left_leg": 150, "middle_left_leg": 150, "hind_left_leg":100, "front_right_leg": 150, "middle_right_leg": 150, "hind_right_leg": 100} 

# The swing movement is targeting a position shifted away from the real AEP
swingNet_aepShiftX = {"front_left_leg": 0.0, "middle_left_leg": -0.1, "hind_left_leg":0.2, "front_right_leg": -0.0, "middle_right_leg": 0.1, "hind_right_leg":-0.2} # for forward walking
swingNet_aepShiftZ = {"front_left_leg": 0.9, "middle_left_leg": 0.9, "hind_left_leg":0.7, "front_right_leg": 0.9, "middle_right_leg": 0.9, "hind_right_leg": 0.7} # for forward walking
swingNet_pepShiftX = {"front_left_leg": -0.0, "middle_left_leg": 0.1, "hind_left_leg":0., "front_right_leg": 0.0, "middle_right_leg": -0.1, "hind_right_leg":-0.}  # for backward walking
swingNet_pepShiftZ = {"front_left_leg": 0.9, "middle_left_leg": 0.9, "hind_left_leg":0.7, "front_right_leg": 0.9, "middle_right_leg": 0.9, "hind_right_leg": 0.7} # for backward walking

#===== Mental Simulation =======
# Defines number of iterations used for the second body model (used for planning).
# The number defines how many iterations are used for convergence of the model when 
# new data is provided (through sensory integration, application of velocities ...)
mmc_mental_iterations = 10

#===== Motivation Units ======
ws = 3


## Marc proposed these parameters:
# r1: -0.125 
# r2:  0.125
# r3c: 0.250
# r3i: 0.188

# Git parameter set
rule1_toAnterior = -0.2
rule1_toNeighbor = -0.13
rule2 = 0.08 # was 0.08
rule3_contralateral = 0.1# was 0.06
#NEW:
rul3_ipsilateral = 0.06
#rule3_sigmoid_toPosterior = 0.125

# The velocity depend parameters have been adapted. = divided/multiplied by 12 because of the new body model

# Changes:
# As no rule 5: Introduced rule 1
#   - between hind legs
#   - from middle to hind legs
# Adapted rule 3 ipsilateral to new velocity, and to new longer stance width
# as well as increased influence. 

#===== walking rules parameters =====
# The dictionary defines the constants that define the behaviour of the coordination rules. Using the "coordination_weights" attribute, for each combination of sender and receiver leg, the strength of the coordination rule can be set. The weights are set in the format coordination_weights[sender_leg_nr][receiver_leg_nr] with the leg numbers according to the order defined in RobotSettings.py (or RSTATIC, as it is most often imported). The rules can be activated/deactivated using the "active" attribute. 
# Please note that the names of the keys of the dictionary ('rule1', 'rule2', ...) must match EXACTLY the names of their instances in the MotivationNetLegs! If the names don't match, the automatic connection creation in the MotivationNetRobot won't work.
coordination_rules = {
	"rule1":
		{"active":True,
		"coordination_weights":(
			(		0,		rule1_toNeighbor,# front_left_leg -> other legs
					rule1_toNeighbor/2,		0,
					0,		0),
				 
			(		rule1_toNeighbor,		0,# front_right_leg -> other legs
					0,		rule1_toNeighbor/2,
					0,		0),
				
			( rule1_toAnterior,		0,# middle_left_leg -> other legs
					0,		0,
					rule1_toNeighbor,		0),
				
			(		0, rule1_toAnterior,# middle_right_leg -> other legs
					0,		0,
					0,		rule1_toNeighbor),
				
			(		0,		0,# hind_left_leg -> other legs
			  rule1_toAnterior,		0,
					0,		 rule1_toNeighbor),
				
			(		0,		0,# hind_right_leg -> other legs
					0, rule1_toAnterior,
					rule1_toNeighbor,		0)
			),
		 "swing_stance_pep_shift_ratio": 0.25, 
		 "velocity_threshold": 0.03,  # /12 because of new velocity fact, was 0.4
		 "velocity_threshold_delay_offset": 0.2, 
		 "lower_velocity_delay_factor": -6,   # *5 because of new velocity fact, was 0.5
		 "higher_velocity_delay_factor":-18,   # *5 because of new velocity fact, was 1.5
		 "max_delay": 0.4
		},
	"rule2":
		{"active":True,
		"coordination_weights":(
			(		0, rule2/4,# front_left_leg -> other legs
					0,		0,
					0,		0),
				 
			( rule2/4,		0,# front_right_leg -> other legs
					0,		0,
					0,		0),
				
			(   rule2,		0,# middle_left_leg -> other legs
					0, rule2/4,
					0,		0),
				
			(		0,  rule2,# middle_right_leg -> other legs
			  rule2/4,		0,
					0,		0),
				
			(		0,		0,# hind_left_leg -> other legs
			    rule2,		0,
					0, rule2/4),
				
			(		0,		0,# hind_right_leg -> other legs
					0,  rule2,
			  rule2/4,		0)
			),
		# ToDo start_delay was 0.27 (biological value)
		"start_delay": 0.37, 
		"duration": 0.05 
		},
	"rule3Ipsilateral":
		{"active":True,
		"coordination_weights":(
			(		0,		0,# front_left_leg -> other legs
			  0.75*rul3_ipsilateral,		0, # was 0.5
					0,		0),
				 
			(		0,		0,# front_right_leg -> other legs
					0, 0.75*rul3_ipsilateral,
					0,		0),
				
			(		0,		0,# middle_left_leg -> other legs
					0,		0,
			  rul3_ipsilateral,		0), # NEW: half, was 0.75
				
			(		0,		0,# middle_right_leg -> other legs
					0,		0,
					0, rul3_ipsilateral),
				
			(		0,		0,# hind_left_leg -> other legs
					0,		0,
					0,		0),
				
			(		0,		0,# hind_right_leg -> other legs
					0,		0,
					0,		0)

			),
		"active_distance": 0.05, 
#		"threshold_offset": 0.25, values from MS
#		"threshold_slope": 20 values from MS
		"threshold_offset": 0.236, 
		"threshold_slope": 2, #19.5,#scaled down by factor 10, new velocity
		"threshold_2_offset": 0.079, #scaled down by factor 10, new velocity
		"threshold_2_slope": 4 #36.6#scaled down by factor 10, new velocity
#		"active_distance": 0.03, 
#		"threshold_offset": 0.236, 
#		"threshold_slope": 19.5,
#		"threshold_2_offset": 0.079, 
#		"threshold_2_slope": 36.6
		},
	"rule3LinearThreshold":
		{"active":True,
		"coordination_weights":(
			(		0, rule3_contralateral,# front_left_leg -> other legs
					0,		0,
					0,		0),
				 
			( rule3_contralateral,	0,# front_right_leg -> other legs
					0,		0,
					0,		0),
				
			(		0,		0,# middle_left_leg -> other legs
					0,		0,
					0,		0),
				
			(		0,		0,# middle_right_leg -> other legs
					0,		0,
					0,		0),
				
			(		0,		0,# hind_left_leg -> other legs
					0,		0,
					0, rule3_contralateral/3),
				
			(		0,		0,# hind_right_leg -> other legs
					0,		0,
			  rule3_contralateral/3,		0)
			),
		"active_distance": 0.03,#was 0.03
		"threshold_offset": 0.5, 
		"threshold_slope": 6. # *15 because of new velocity fact, was 0.5
		},
	"rule4":
		{"active":False,
		"coordination_weights":(
			(		0,		0,# front_left_leg -> other legs
					1,		0,
					0,		0),
				 
			(		0,		0,# front_right_leg -> other legs
					0,		1,
					0,		0),
				
			(		0,		0,# middle_left_leg -> other legs
					0,		0,
					1,		0),
				
			(		0,		0,# middle_right_leg -> other legs
					0,		0,
					0,		1),
				
			(		0,		0,# hind_left_leg -> other legs
					0,		0,
					0,		0),
				
			(		0,		0,# hind_right_leg -> other legs
					0,		0,
					0,		0)
			)
		}
	}
