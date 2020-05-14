## Settings for the robot objects: defining legs, namings, motor parameters of Hector.

#====== simulation parameters ========
controller_frequency = 100

#========== Leg enabled =========
legs_enabled = (True, True,
				True, True,
				True, True)

##========== naming objects ========
leg_names=(	'front_left_leg', 'front_right_leg', 
			'middle_left_leg', 'middle_right_leg', 
			'hind_left_leg', 'hind_right_leg')

_gamma=1.2
joint_angle_limits=(    ((-0.6, 1.4), (-1.6, 0.8), (-_gamma, _gamma)),  ((-1.4, 0.6),(-0.8, 1.6),(-_gamma, _gamma)),# ((front_left_leg: (alpha: (lower, upper), beta: (lower, upper), gamma: (lower, upper)),front_right_leg: (alpha: (lower, upper), beta: (lower, upper), ga$
                        ((-1.0, 1.0), (-1.6, 0.8), (-_gamma, _gamma)),  ((-1.0, 1.0),(-0.8, 1.6),(-_gamma, _gamma)),# middle_left_leg, middle_right_leg, [see above for meaning of values]
                        ((-1.6, 0.5), (-0.8, 1.6), (-_gamma, _gamma)),  ((-0.5, 1.6),(-1.6, 0.8),(-_gamma, _gamma)))# hind_left_leg, hind_right_leg [see above for meaning of values]

