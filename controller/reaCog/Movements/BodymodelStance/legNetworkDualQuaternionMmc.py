'''
Created on 3.12.2011

@author: mschilling
'''
import controller.reaCog.Movements.BodymodelStance.pycgtypes.vec3 as vec3
import math
from controller.reaCog.Movements.BodymodelStance.dualQuaternion import dualQuaternion

class legNetworkDualQuaternionMmc:
    """ Three segmented manipulator with three rotational joints.
        Class for representing a Mean of Multiple Computation Network of an insect leg.
        It is restricted to three degrees of freedom (non-redundant) - the transformations
        are first computed and then projected onto the joint axes.
        As arguments the segment length can be set-up.
        Afterwards all the necessary variables are set-up as transformations,
        represented as Dual Quaternions.
        The leg can be set to an initial posture.

        Derived from armMMC.py
    """

    def __init__(self, *args):
        """ Setting up the leg:
            No arguments are used.
            All necessary variables for the MMC computation are
            initialised as list of Dual Quaternions representing the leg:
                l_x - Segment translations
                d_x - Diagonal translation
                theta_x - joint rotation
                gamma_x - diagonal rotation
                delta_x - additional rotations for aligning orientations
                alpha, r - target rotation and translation
            For each variable exists a list which contains:
                [current value,
                 last value,
                 2 or 3 different values computed from different equations]
            After setup the leg is fully outstretched.

            The graphic output is initialised.
        """
        # Initialisation of segments as translations
        self.l_0 = [dualQuaternion(),0,0,0]
        self.l_1 = [dualQuaternion(),0,0,0]
        self.l_2 = [dualQuaternion(),0,0,0]
        if len(args)==0:
            self.l_0[0].set_translation([1,0,0])
            self.l_1[0].set_translation([1,0,0])
            self.l_2[0].set_translation([1,0,0])
        else :
            self.l_0[0].set_translation([args[0][0],0,0])
            self.l_1[0].set_translation([args[0][1],0,0])
            self.l_2[0].set_translation([args[0][2],0,0])
        # Initialisation of Diagonals and End effector Translations
        self.d_0 = [self.l_0[0] * self.l_1[0],0,0,0]
        self.d_1 = [self.l_1[0] * self.l_2[0],0,0,0]
        self.r = [self.d_0[0] * self.l_2[0],0,0,0]
        # Initialisation of Angles: all set to zero = fully stretched
        self.theta_0 = [dualQuaternion(),0,0,0]
        self.theta_1 = [dualQuaternion(),0,0,0]
        self.theta_2 = [dualQuaternion(),0,0,0]
        self.gamma_0 = [dualQuaternion(),0,0,0]
        self.gamma_1 = [dualQuaternion(),0,0,0]
        self.delta_0 = [dualQuaternion(),0,0,0]
        self.delta_1 = [dualQuaternion(),0,0,0]
        self.delta_2 = [dualQuaternion(),0,0,0]
        self.alpha = [dualQuaternion(),0,0,0]
        # Initialisation of Velocities and Accelerations
        self.theta_0_vel = [dualQuaternion(),0,0,0]
        self.theta_1_vel = [dualQuaternion(),0,0,0]
        self.theta_2_vel = [dualQuaternion(),0,0,0]
        self.theta_0_acc = [dualQuaternion(),0,0,0]
        self.theta_1_acc = [dualQuaternion(),0,0,0]
        self.theta_2_acc = [dualQuaternion(),0,0,0]

        # Damping values
        self.mmc_damping = 10
        self.mmc_vel_damping = 0
        self.mmc_acc_damping = 0

        # Writing down the distance to target and the velocity
        self.swing_target_point = vec3(self.get_manipulator_coordinates()[0][3], \
                  self.get_manipulator_coordinates()[1][3], \
                  self.get_manipulator_coordinates()[2][3])
        self.data_end_point_velocity = []
        self.data_target_distance = []

    def get_joint_angles(self):
        """ Get current joint angles from the leg network.
        """
        angles = [0,0,0]
        angles[0] = self.theta_0[0].real_part.toAngleAxis()[0]
        if (self.theta_0[0].real_part.toAngleAxis()[1]).z < 0:
            angles[0] = -1*angles[0]
        angles[1] = self.theta_1[0].real_part.toAngleAxis()[0]
        if (self.theta_1[0].real_part.toAngleAxis()[1]).y < 0:
            angles[1] = -1*angles[1]
        angles[2] = self.theta_2[0].real_part.toAngleAxis()[0]
        if (self.theta_2[0].real_part.toAngleAxis()[1]).y < 0:
            angles[2] = -1*angles[2]
        return angles

    def get_target_coordinates(self):
        """ The coordinates of the target position are computed.
        """
        target = self.alpha[0] * self.r[0]
        target.compensate_rotation()
        return [[0, target.dual_part.x*2],
                [0, target.dual_part.y*2],
                [0, target.dual_part.z*2]]

    def compute_multiple_computations(self):
        """ For each variable new values are calculated through
            different equations. The values are pushed on the variable
            stack
        """
        # Theta 0:
        self.theta_0[2] = self.gamma_0[0] * self.d_0[0] * self.delta_0[0] * \
            self.l_1[0].inverse() * self.theta_1[0].inverse() * self.l_0[0].inverse()
        self.theta_0[3] = self.alpha[0] * self.r[0] * self.delta_2[0] * \
            self.delta_1[0].inverse() * self.d_1[0].inverse() * \
            self.gamma_1[0].inverse() * self.l_0[0].inverse()
        # Theta 1:
        self.theta_1[2] = self.l_0[0].inverse() * self.theta_0[0].inverse() * \
            self.gamma_0[0] * self.d_0[0] * self.delta_0[0] * self.l_1[0].inverse()
        self.theta_1[3] = self.gamma_1[0] * self.d_1[0] * self.delta_1[0] * \
            self.l_2[0].inverse() * self.theta_2[0].inverse() * self.l_1[0].inverse()
        # Theta 2:
        self.theta_2[2] = self.l_1[0].inverse() * self.theta_1[0].inverse() * \
            self.gamma_1[0] * self.d_1[0] * self.delta_1[0] * self.l_2[0].inverse()
        self.theta_2[3] = self.delta_0[0].inverse() * self.d_0[0].inverse() * \
            self.gamma_0[0].inverse() * self.alpha[0] * self.r[0] * \
            self.delta_2[0] * self.l_2[0].inverse()

        # Gamma 0:
        self.gamma_0[2] = self.theta_0[0] * self.l_0[0] * self.theta_1[0] * \
            self.l_1[0] * self.delta_0[0].inverse() * self.d_0[0].inverse()
        self.gamma_0[3] = self.alpha[0] * self.r[0] * self.delta_2[0] * \
            self.l_2[0].inverse() * self.theta_2[0].inverse() * \
            self.delta_0[0].inverse() * self.d_0[0].inverse()
        # Gamma 1:
        self.gamma_1[2] = self.theta_1[0] * self.l_1[0] * self.theta_2[0] * \
            self.l_2[0] * self.delta_1[0].inverse() * self.d_1[0].inverse()
        self.gamma_1[3] = self.l_0[0].inverse() * self.theta_0[0].inverse() * \
            self.alpha[0] * self.r[0] * self.delta_2[0] * \
            self.delta_1[0].inverse() * self.d_1[0].inverse()

        # Delta 0:
        self.delta_0[2] = self.d_0[0].inverse() * self.gamma_0[0].inverse() * \
            self.theta_0[0] * self.l_0[0] * self.theta_1[0] * self.l_1[0]
        self.delta_0[3] = self.d_0[0].inverse() * self.gamma_0[0].inverse() * \
            self.alpha[0] * self.r[0] * self.delta_2[0] * \
            self.l_2[0].inverse() * self.theta_2[0].inverse()
        # Delta 1:
        self.delta_1[2] = self.d_1[0].inverse() * self.gamma_1[0].inverse() * \
            self.l_0[0].inverse() * self.theta_0[0].inverse() * \
            self.alpha[0] * self.r[0] * self.delta_2[0]
        self.delta_1[3] = self.d_1[0].inverse() * self.gamma_1[0].inverse() * \
            self.theta_1[0] * self.l_1[0] * self.theta_2[0] * self.l_2[0]
        # Delta 2:
        self.delta_2[2] = self.r[0].inverse() * self.alpha[0].inverse() * \
            self.theta_0[0] * self.l_0[0] * self.gamma_1[0] * self.d_1[0] * self.delta_1[0]
        self.delta_2[3] = self.r[0].inverse() * self.alpha[0].inverse() * \
            self.gamma_0[0] * self.d_0[0] * self.delta_0[0] * \
            self.theta_2[0] * self.l_2[0]

        # Alpha:
        self.alpha[2] = self.theta_0[0] * self.l_0[0] * self.gamma_1[0] * \
            self.d_1[0] * self.delta_1[0] * \
            self.delta_2[0].inverse() * self.r[0].inverse()
        self.alpha[3] = self.gamma_0[0] * self.d_0[0] * self.delta_0[0] * \
            self.theta_2[0] * self.l_2[0] * \
            self.delta_2[0].inverse() * self.r[0].inverse()

        # D_0:
        self.d_0[2] = self.gamma_0[0].inverse() * self.theta_0[0] * self.l_0[0] * \
            self.theta_1[0] * self.l_1[0] * self.delta_0[0].inverse()
        self.d_0[3] = self.gamma_0[0].inverse() * self.alpha[0] * self.r[0] * \
            self.delta_2[0] * self.l_2[0].inverse() * self.theta_2[0].inverse() * \
            self.delta_0[0].inverse()
        # D_1:
        self.d_1[2] = self.gamma_1[0].inverse() * self.theta_1[0] * self.l_1[0] * \
            self.theta_2[0] * self.l_2[0] * self.delta_1[0].inverse()
        self.d_1[3] = self.gamma_1[0].inverse() * self.l_0[0].inverse()  * \
            self.theta_0[0].inverse() * self.alpha[0] * self.r[0] * \
            self.delta_2[0] * self.delta_1[0].inverse()

        # R:
        self.r[2] = self.alpha[0].inverse() * self.theta_0[0] * self.l_0[0] * \
            self.gamma_1[0] * self.d_1[0] * self.delta_1[0] * self.delta_2[0].inverse()
        self.r[3] = self.alpha[0].inverse() * self.gamma_0[0] * self.d_0[0] * \
            self.delta_0[0] * self.theta_2[0] * self.l_2[0] * self.delta_2[0].inverse()

    def acos_inbounds(self, cos_value):
        """ Compute the acos value if possible.
            If the cosine value is out of the range of the acos function,
            use the range limit instead.
        """
        if ( cos_value > -1.0 ) :
            if ( cos_value < 1.0 ) :
                return math.acos( cos_value );
            else :
                return 0
        else :
            return math.pi

    def compute_cosine_rule(self):
        """ As additional computations, for the second and third joint
            the cosine rule is used. This is necessary to flex the leg
            when the target is very nearby (the cosine rule computes the enclosed
            angle between the two joining segments depending on the length of the
            opposing diagonal).
        """
        l_a=abs(self.l_0[0].dual_part)
        l_b=abs(self.l_1[0].dual_part)
        diag=min([abs(self.d_0[0].dual_part),(l_a + l_b)])
        angle=math.copysign (math.pi - self.acos_inbounds( (l_a*l_a + l_b*l_b - diag*diag) / (2 * l_a * l_b) ),
                            self.theta_0[0].real_part.toAngleAxis()[0])
        if (len(self.theta_1) == 4):
            self.theta_1.append(dualQuaternion())
        self.theta_1[4].set_rotation(angle,self.theta_1[0].real_part.toAngleAxis()[1])
        l_a=abs(self.l_1[0].dual_part)
        l_b=abs(self.l_2[0].dual_part)
        diag=min([abs(self.d_1[0].dual_part),(l_a + l_b)])
        angle=math.copysign (math.pi - self.acos_inbounds( (l_a*l_a + l_b*l_b - diag*diag) / (2 * l_a * l_b) ),
                            self.theta_1[0].real_part.toAngleAxis()[0])
        if (len(self.theta_2) == 4):
            self.theta_2.append(dualQuaternion())
        self.theta_2[4].set_rotation(angle,self.theta_2[0].real_part.toAngleAxis()[1])

    def compensate_errors_in_multiple_computations(self):
        """ Align resulting dual quaternions:
            in rotational joints the translational share is compensated and
            in translational transformations the rotation part is compensated.
        """
        self.theta_0[2].compensate_translation(self.l_0[0],1)
        self.theta_0[3].compensate_translation(self.l_0[0],1)
        self.theta_1[2].compensate_translation(self.l_1[0],1)
        self.theta_1[3].compensate_translation(self.l_1[0],1)
        self.theta_2[2].compensate_translation(self.l_2[0],1)
        self.theta_2[3].compensate_translation(self.l_2[0],1)
        self.gamma_0[2].compensate_translation(self.d_0[0])
        self.gamma_0[3].compensate_translation(self.d_0[0])
        self.gamma_1[2].compensate_translation(self.d_1[0])
        self.gamma_1[3].compensate_translation(self.d_1[0])
        self.alpha[2].compensate_translation(self.r[0])
        self.alpha[3].compensate_translation(self.r[0])
        # Deltas are not compensated.
#       self.delta_0[2].compensateTranslation(self.d_0[0])
#       self.delta_0[3].compensateTranslation(self.d_0[0])
#       self.delta_1[2].compensateTranslation(self.d_1[0])
#       self.delta_1[3].compensateTranslation(self.d_1[0])
#       self.delta_2[2].compensateTranslation(self.r[0])
#       self.delta_2[3].compensateTranslation(self.r[0])
        self.delta_0[2].strip_translation()
        self.delta_0[3].strip_translation()
        self.delta_1[2].strip_translation()
        self.delta_1[3].strip_translation()
        self.delta_2[2].strip_translation()
        self.delta_2[3].strip_translation()
        self.d_0[2].compensate_rotation()
        self.d_0[3].compensate_rotation()
        self.d_1[2].compensate_rotation()
        self.d_1[3].compensate_rotation()
        self.r[2].compensate_rotation()
        self.r[3].compensate_rotation()

    def calculate_mean_values_of_multiple_computations(self, damp):
        """ Calculate the mean value for all the variables.
            This should only be done, after the multiple
            computations have been computed and saved in the
            variable list (starting from third element).
        """
        # Cosine rule is disabled
        self.theta_0[0].linear_blending(self.theta_0[1:4],[damp,1,1])
        self.theta_1[0].linear_blending(self.theta_1[1:5],[damp,1,1,0.6])
        self.theta_2[0].linear_blending(self.theta_2[1:5],[damp,1,1,0.5])
        self.gamma_0[0].linear_blending(self.gamma_0[1:4],[damp/4,1,1])
        self.gamma_1[0].linear_blending(self.gamma_1[1:4],[damp/4,1,1])
        self.delta_0[0].linear_blending(self.delta_0[1:4],[damp/4,1,1])
        self.delta_1[0].linear_blending(self.delta_1[1:4],[damp/4,1,1])
        self.delta_2[0].linear_blending(self.delta_2[1:4],[damp/4,1,1])
        self.alpha[0].linear_blending(self.alpha[1:4],[damp,1,1])
        self.r[0].linear_blending(self.r[1:4],[damp,1,1])
        self.d_0[0].linear_blending(self.d_0[1:4],[damp/4,1,1])
        self.d_1[0].linear_blending(self.d_1[1:4],[damp/4,1,1])

    def mmc_kinematic_iteration_step(self):
        """ The MMC Method:
            - the multiple computations are computed for each variable
            - afterwards: a normalisation is done
                for joints: translational share is compensated
                for translations: rotational share is compensated
            - the mean for each variable is calculated
            Before this method is called the input values has to be enforced on
            the variables every time:
                e.g., for inverse kinematic = r[0] and alpha[0] has to be set
            The old value is shifted to the second element in the lists.
        """
        # Remember the old values of the variables
        self.theta_0[1]=dualQuaternion(self.theta_0[0])
        self.theta_1[1]=dualQuaternion(self.theta_1[0])
        self.theta_2[1]=dualQuaternion(self.theta_2[0])
        self.gamma_0[1]=dualQuaternion(self.gamma_0[0])
        self.gamma_1[1]=dualQuaternion(self.gamma_1[0])
        self.delta_0[1]=dualQuaternion(self.delta_0[0])
        self.delta_1[1]=dualQuaternion(self.delta_1[0])
        self.delta_2[1]=dualQuaternion(self.delta_2[0])
        self.alpha[1]=dualQuaternion(self.alpha[0])
        self.d_0[1]=dualQuaternion(self.d_0[0])
        self.d_1[1]=dualQuaternion(self.d_1[0])
        self.r[1]=dualQuaternion(self.r[0])

        # Apply multiple computations:
        # First, the geometric relationships are used
        # - i.e. the equations describing the triangles.
        # Second, for the second and third joint the cosine rule
        # is in addition used. This is necessary to flex the moevment
        # when the target is very nearby (the cosine rule computes the enclosed
        # angle between the two joining segments depending on the length of the
        # opposing diagonal).
        self.compute_multiple_computations()
        self.compute_cosine_rule()
        # Align resulting dual quaternions:
        # in rotational joints the translational share is compensated and
        # in translational transformations the rotation part is compensated
        self.compensate_errors_in_multiple_computations()

        # Interpolate the multiple computations for each variable.
        # In addition, the current value weighted by the parameter
        # mmc_damping is also included in the calculation.
        # This prevents oscillations as the current value is fed back into
        # the system.
        self.calculate_mean_values_of_multiple_computations(self.mmc_damping)
        # Constraint application: restricting the joints to one degree of freedom
        self.theta_0[0].project_onto_fixed_rotation_around_z(self.l_0[0])
        self.theta_1[0].project_onto_fixed_rotation_around_y(self.l_1[0])
        self.theta_2[0].project_onto_fixed_rotation_around_y(self.l_2[0])

    def calculate_dynamic_values(self):
        """
            From the current joint angle values
            and the old joint angle values the current velocities
            and the accelerations are computed.
            Before this, the new joint values must have been set
            (mmcKinematicIterationStep)
        """
        # Remember the old values of the variables
        self.theta_0_vel[1]=dualQuaternion(self.theta_0_vel[0])
        self.theta_1_vel[1]=dualQuaternion(self.theta_1_vel[0])
        self.theta_2_vel[1]=dualQuaternion(self.theta_2_vel[0])
        self.theta_0_acc[1]=dualQuaternion(self.theta_0_acc[0])
        self.theta_1_acc[1]=dualQuaternion(self.theta_1_acc[0])
        self.theta_2_acc[1]=dualQuaternion(self.theta_2_acc[0])
        self.theta_0_vel[0].linear_blending([self.theta_0_vel[1], \
                                            (self.theta_0[1].get_conjugate() * self.theta_0[0])], \
                                            [self.mmc_vel_damping,1])
        self.theta_1_vel[0].linear_blending([self.theta_1_vel[1], \
                                            (self.theta_1[1].get_conjugate() * self.theta_1[0])], \
                                            [self.mmc_vel_damping,1])
        self.theta_2_vel[0].linear_blending([self.theta_2_vel[1], \
                                            (self.theta_2[1].get_conjugate() * self.theta_2[0])], \
                                            [self.mmc_vel_damping,1])

    def update_joints_by_including_dynamic_influences(self):
        """ For the dynamic MMC network including velocities.
            the joint positions depend on the velocity of the joints
            (which internally is calculated using the traditional
             MMC network approach and integrates additional influences)
            Before the network can be updated the
              calculate_dynamic_values function must be called.
            Usually, one simply calls mmc_dynamic_iteration_update
            after the classic MMC iteration step.
            The mmc_dynamic_iteration_update is doing all the steps:
                - calculate velocities
                - update positions
                - store the end position
        """
        self.theta_0[0]=self.theta_0[1] * self.theta_0_vel[0]
        self.theta_1[0]=self.theta_1[1] * self.theta_1_vel[0]
        self.theta_2[0]=self.theta_2[1] * self.theta_2_vel[0]

    def store_endpoint_data(self,target_point):
        """ Storing the endpoint positions for
            displaying afterwards the velocity profile.
        """
        current_end_point = vec3(self.get_manipulator_coordinates()[0][3], \
                                 self.get_manipulator_coordinates()[1][3], \
                                 self.get_manipulator_coordinates()[2][3])
        self.data_end_point_velocity.append(abs(current_end_point - self.swing_target_point))
        self.data_target_distance.append(abs(current_end_point - target_point ))
        self.swing_target_point = current_end_point

    def mmc_dynamic_iteration_update(self):
        """ The dynamic iteration step function -
            which is called after the classic MMC iteration step.
            The mmc_dynamic_iteration_update is doing all the steps:
                - calculate velocities
                - update positions
                - store the end position
            but has to be explicitly called after mmcKinematicIterationStep!
        """
        self.calculate_dynamic_values()
        self.update_joints_by_including_dynamic_influences()
        #self.store_endpoint_data()
