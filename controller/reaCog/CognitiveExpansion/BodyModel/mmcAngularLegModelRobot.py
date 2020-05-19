'''
Created on 22.1.2012

Changed 20.1.2014 - now adapted to the hector convention of joints
(gamma rotated ninety degrees, sign changes)

@author: mschilling
'''
import math
import numpy

from numpy.random import normal
from .wrapperJointAngle import wrapperJointAngle

import controller.reaCog.WalknetSettings as WSTATIC

##
#   Three segmented manipulator with three rotational joints.
#   Class for representing a Mean of Multiple Computation Network of an insect leg.
#   It is restricted to three degrees of freedom (non-redundant).
#
#   As arguments the segment length can be set-up.
#   Afterwards all the necessary variables are set-up.
#
#   Uses trigonometric relations to calculate the different variables.
#   In case of the joint computations, multiple values are integrated
#   using the arc-tangens function which is unambiguous (in contrast to asin, ...).
#
#   The network is quite performant - it needs only a few iteration steps for
#   relaxation (~ 6 for completely  new posture, for small movements two or three
#   can be sufficient). On a single core 1.6 Ghz this allowed for more then 30.000
#   iteration steps per second (a single iteration step costs about the same as
#   the direct calculation of the inverse kinematics).
#
#   The leg network can be initialised with a list (3 elements) of the segments
#   lengths - the arm is initialised in an outstretched position.
#
#   The network is considered in leg coordinate system = after application of the
#   phiPsi transformation.
#
#   The network is storing all changing variables in lists over time (iteration steps).
##
class mmcAngularLegModel:
    def __init__(self, wleg, logging = False):
        # Initialisation of the segment lengths (as argument list given to the object)
        self.wleg = wleg
        
        self.amputated = False
        
        # Copy some of the variables to make access faster
        # ! These values are not updated while the controller runs!
        self.name = self.wleg.leg.name
        
        self.leg_segm = [self.wleg.leg.segment_lengths[0], self.wleg.leg.segment_lengths[1], self.wleg.leg.segment_lengths[2]]
        self.leg_target = [numpy.array([(self.leg_segm[0] + self.leg_segm[1] + self.leg_segm[2]),0.0,0.0, 0.0])]
        self.projection_length = [numpy.linalg.norm(self.leg_target[0])]
        #self.joint = [[0.01, 0.01, -0.01]]
        self.alpha = wrapperJointAngle(0.01, logging)
        self.beta = wrapperJointAngle(0.01, logging)
        self.gamma = wrapperJointAngle(-0.01, logging)
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
        if (self.wleg.leg.beta_direction):
            self.betaDirectionFactor = 1.
        else:
            self.betaDirectionFactor = -1.
        
        self.new_target = numpy.array([0., 0., 0.])
        self.new_joints = numpy.array([0., 0., 0.])
        
        # Collecting the new control (velocity) signals
        self.controlVelocities = numpy.zeros(3)
        
        # Recursive nesting: as the leg structure for the robot is
        # wleg - leg this has to be reproduced for the internal model
        # Which is realized as a nested class.
        self.leg = self
        
    def init_leg_mmc_model(self):
        """ As the MMC networks have to be initialised this function has to be called in
            the beginning to drive the network into a valid configuration. This is
            important for the Dual Quaternion network, but for the single leg network
            this might be dropped as there the relaxation is very fast.
        """
        for _ in range(0,40) :
            self.set_joint_angles( self.wleg.leg.getInputPosition() )
            self.mmc_kinematic_iteration_step()
        
    def __getattr__(self,name):
        """ The MMC leg model reroutes all calls which can not be answered to
            the real robot leg (which was set in initialization).
            This is mainly done to handle all the variables associated with the Leg
            or the WLeg.
            The model leg is hiding the access functions for getting kinematic
            values and setting joint velocities (rerouted by the switch in SwitchRobot).
        """
        try :
            return getattr(self.wleg.leg, name)
        except AttributeError :
            try :
                return getattr(self.wleg, name)
            except AttributeError :
                print("Konnte ",name, "nicht in wleg.leg und wleg finden" )
                pass

    ##  Compute the joint angles.
    #   Exploiting trigonometric relations the joint angles are calculated in
    #   multiple ways. These computations would involve computing asin and acos
    #   values. This is problematic as these are ambiguous. Therefore, we integrate
    #   these computations in a first step by computing the quotient of the sine and
    #   cosine value which equals the tangens value. From this we can compute the
    #   unambiguous arc tangens value for the joint value.
    #
    #   The function is called with the iteration step on which the computations
    #   should be applied. Usually, it should be called with thisObject.step.
    def compute_joints_and_integrate(self, step):
        if not(self.amputated):
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
            mc_joint.append(self.damping * self.alpha.getInputPositionForIterationStep(step))
            self.new_joints[0] = sum(mc_joint)/(self.damping + len(mc_joint) - 1)

            # Computation of the beta angle
            # Integration of two trigonometric relations
            mc_joint = [(math.atan2( (self.leg_target[step][2]
                        + self.leg_segm[2] * math.cos(self.beta.getInputPositionForIterationStep(step) - self.gamma.getInputPositionForIterationStep(step)) ),
                    (self.projection_length[step] - self.leg_segm[0]
                        - self.leg_segm[2] * math.sin(self.beta.getInputPositionForIterationStep(step) - self.gamma.getInputPositionForIterationStep(step)) ) ))]
            # Integration of sensor data if given
            if (len(self.current_sensor_angles[1]) > 0) :
                mc_joint.append(sum(self.current_sensor_angles[1]))
            # Recurrent connection
            mc_joint.append(self.damping * self.beta.getInputPositionForIterationStep(step))
            self.new_joints[1] = sum(mc_joint)/(self.damping + len(mc_joint) - 1)
            # Enforce joint constraints
            if (self.new_joints[1] < 0.01):
                self.new_joints[1] = 0.01

            # Computation of the gamma angle
            # Integration of two trigonometric relations
            mc_joint = [-(math.atan2( (self.leg_target[step][2]
                        - self.leg_segm[1] * math.sin(self.beta.getInputPositionForIterationStep(step)) ),
                    (self.projection_length[step] - self.leg_segm[0]
                        - self.leg_segm[1] * math.cos(self.beta.getInputPositionForIterationStep(step)) )) - self.beta.getInputPositionForIterationStep(step)) - math.pi/2]
            # Integration of sensor data if given
            if (len(self.current_sensor_angles[2]) > 0) :
                mc_joint.append(sum(self.current_sensor_angles[2]))
            # Recurrent connection
            mc_joint.append(self.damping * self.gamma.getInputPositionForIterationStep(step))
            self.new_joints[2] = sum(mc_joint)/(self.damping + len(mc_joint) - 1)
            if (self.new_joints[2] < -1.5608):
                self.new_joints[2] = -1.5608

    ##  Compute the target vector.
    #   Application of basically the forward kinematics
    #   - but also includes recurrent connections (the old value is partially
    #   maintained).
    def compute_target(self, step):
        self.new_target = numpy.array([0., 0., 0., 0.])
        if (abs(self.alpha.getInputPositionForIterationStep(step)) > 0.1) :
            self.new_target[0] = (self.leg_target[step][1] / math.tan(-self.alpha.getInputPositionForIterationStep(step))
                + self.projection_length[step] * math.cos(self.alpha.getInputPositionForIterationStep(step))
                + self.damping * self.leg_target[step][0]) / (2 + self.damping)
        else:
            self.new_target[0] = (self.projection_length[step] * math.cos(self.alpha.getInputPositionForIterationStep(step))
                + self.damping * self.leg_target[step][0]) / (1 + self.damping)
        self.new_target[1] = (self.leg_target[step][0] * math.tan(-self.alpha.getInputPositionForIterationStep(step))
            + self.projection_length[step] * math.sin(-self.alpha.getInputPositionForIterationStep(step))
            + self.damping * self.leg_target[step][1]) / (2 + self.damping)
        self.new_target[2] = (self.leg_segm[1] * math.sin(self.beta.getInputPositionForIterationStep(step))
                    - self.leg_segm[2] * math.cos(self.beta.getInputPositionForIterationStep(step) - self.gamma.getInputPositionForIterationStep(step))
                + self.damping * self.leg_target[step][2]) / (1 + self.damping)

    ##  Compute of the projection length of the leg.
    #   The second and third joint work in a plane - spanned by the z-axis
    #   and rotated around this axis. The overall length of the segment in this plane
    #   is a projection of the x and y values onto this plane.
    def compute_projection_length(self, step):
        new_l = [(self.leg_segm[0] + self.leg_segm[1] * math.cos(self.beta.getInputPositionForIterationStep(step))
                    + self.leg_segm[2] * math.sin(self.beta.getInputPositionForIterationStep(step) - self.gamma.getInputPositionForIterationStep(step)) )]
        if (abs(self.alpha.getInputPositionForIterationStep(step)) > 0.1) :
            new_l.append( (self.leg_target[step][1] / math.sin(-self.alpha.getInputPositionForIterationStep(step)) ) )
        if (abs(self.alpha.getInputPositionForIterationStep(step)) < 1.47) :
            new_l.append( (self.leg_target[step][0] / math.cos(self.alpha.getInputPositionForIterationStep(step)) ) )
        new_l.append( self.damping * self.projection_length[step] )
        return (sum(new_l)/(self.damping + len(new_l) - 1))

    ##  The MMC Method:
    #   - the multiple computations are computed for each variable
    #   - the mean for each variable is calculated
    #   The new values are appended to the list of element values.
    #   For each variable new values are calculated through
    #   different equations.
    def mmc_kinematic_iteration_step(self):
        if not(self.amputated):
            self.compute_target(-1)
            new_projection_lengths = self.compute_projection_length(-1)
            self.compute_joints_and_integrate(-1)
            if self.logging_values :
                self.leg_target.append(self.new_target)
                self.projection_length.append(new_projection_lengths)
                self.alpha.addNewInputPosition(self.new_joints[0])
                self.beta.addNewInputPosition(self.new_joints[1])
                self.gamma.addNewInputPosition(self.new_joints[2])
                #self.joint.append(new_joints)
            else:
                self.leg_target[0] = self.new_target
                self.projection_length[0] = new_projection_lengths
                self.alpha.setInputPosition(self.new_joints[0])
                self.beta.setInputPosition(self.new_joints[1])
                self.gamma.setInputPosition(self.new_joints[2])
                #self.joint[0] = [self.alpha, self.beta, self.gamma]
            self.step += 1
            self.current_sensor_angles = [[], [], []]

    ##  Return the current joint angles.
    #   After iteration of the network the joint variables can be returned.
    def getInputPosition(self):
        if not(self.amputated):
            return numpy.array([self.alpha.getInputPositionForIterationStep(-1), self.betaDirectionFactor*self.beta.getInputPositionForIterationStep(-1), self.betaDirectionFactor*self.gamma.getInputPositionForIterationStep(-1)])
        else:
            return numpy.array([0., 0., 0.])

    ##  Set the leg target vector (as a vec3).
    #   When the network shall be used to compute the inverse kinematic function
    #   the target vector is enforced onto the network before an iteration step
    #   using this function.
    def set_leg_target(self, new_target):
        self.leg_target[-1] = new_target

    ##  Set new joint angles(as a list).
    #   When the network shall be used to compute the forward kinematic function
    #   the target joint angles are enforced onto the network before an iteration step
    #   using this function.
    def set_joint_angles(self, angles):
        self.alpha.setInputPosition(angles[0])
        self.beta.setInputPosition(self.betaDirectionFactor*angles[1])
        self.gamma.setInputPosition(self.betaDirectionFactor*angles[2])

    ##  Sets the sensor feedback values of the joints.
    #   These are integrated in the MMC network.
    def set_sensory_feedback_joint_values(self, angles):
        self.current_sensor_angles[0] = [angles[0]]
        self.current_sensor_angles[1] = [self.betaDirectionFactor*angles[1]]
        self.current_sensor_angles[2] = [self.betaDirectionFactor*angles[2]]

    ##  Adds sensory feedback for joints.
    #   When we want to do sensory integration we add sensory influences
    #   which are added to the list of sensory feedback and noise is applied.
    #   This function allows to introduce multiple sensor data which
    #   is integrated by the network: on a temporal scale and constrained
    #   by the actual kinematics.
    def add_sensory_joint_values(self, angles):
        if (self.noise_std_dev > 0):
            self.noise = normal(0,self.noise_std_dev, 3)
        self.current_sensor_angles[0].append(angles[0] + self.noise[0])
        self.current_sensor_angles[1].append(self.betaDirectionFactor*angles[1] + self.noise[1])
        self.current_sensor_angles[2].append(self.betaDirectionFactor*angles[2] + self.noise[2])
        
    def getFootPosition(self):
        if not(self.amputated):
            target_in_body = numpy.dot(self.wleg.leg._phi_psi_trans, self.leg_target[-1])
        else:
            target_in_body = numpy.array([0., 0., 0., 0.])
        return target_in_body[0:3]

    ##
    #   Return the positions of the leg segments - as points (zero, after coxa,
    #   femur and tibia). Is returned as in the body coordinate system
    def getLegPoints(self, coxa_pos):
        leg_points = [numpy.array([0.0, 0.0, 0.0])]
        leg_points.append(-coxa_pos)
        new_point = numpy.array([self.lc*math.cos(self.alpha.getInputPositionForIterationStep(-1)), \
            self.lc * math.sin(-self.alpha.getInputPositionForIterationStep(-1)), \
            0.0, 0.0])
        leg_points.append(new_point)
        new_point_2 = numpy.array(new_point)
        new_point_2[0] += self.lf * math.cos(self.beta.getInputPositionForIterationStep(-1)) * math.cos(self.alpha.getInputPositionForIterationStep(-1))
        new_point_2[1] += self.lf * math.cos(self.beta.getInputPositionForIterationStep(-1)) * math.sin(-self.alpha.getInputPositionForIterationStep(-1))
        new_point_2[2] += self.lf * math.sin(self.beta.getInputPositionForIterationStep(-1))
        leg_points.append(new_point_2)
                     
        leg_points.append(self.leg_target[-1])
        for i in range (2,5):
            leg_points[i] = (numpy.dot(self.wleg.leg._phi_psi_trans, leg_points[i]))[0:3]
            leg_points[i] -= coxa_pos
        return leg_points

    @property
    def input_foot_position(self):
        return self.getFootPosition()[0:3]

    ##
    #   Collect movement primitive control signals
    #   simply by adding those up.
    #   For all three joints of the leg.
    #   @param vector of the three joint velocities.
    def addControlVelocities(self, new_vel):
        self.controlVelocities += new_vel

    ##
    #   Sending the collected control signals to the three joints.
    #   Velocities are used as control signals.
    def sendControlVelocities(self):
        if not(self.amputated):
            current_angles = self.getInputPosition()
            new_angles = current_angles + 0.01*self.controlVelocities
            self.set_joint_angles( new_angles )
            for i in range(0, WSTATIC.mmc_mental_iterations):
                self.set_joint_angles( new_angles )
                self.mmc_kinematic_iteration_step()
            self.old_vel = self.controlVelocities
            self.controlVelocities = numpy.zeros(3)
        
    ##
    #   Estimate ground ground_contact:
    #   Predict current leg position (using fw kinematics) and
    #   simply decide if the leg should touch ground
    #   (very stable, but works only on flat terrain).
    def predictedGroundContact(self):
        if (self.wleg.leg.leg_enabled) and not(self.amputated):
            if (self.input_foot_position[2] < -(WSTATIC.stanceheight * WSTATIC.predicted_ground_contact_height_factor)):
                return 1
            else:
                return 0
        else:
            return 0
                
    def getCommentedGC(self):
        if (self.wleg.leg.leg_enabled):
            if (self.input_foot_position[2] < -(WSTATIC.stanceheight * WSTATIC.predicted_ground_contact_height_factor)):
                return 1
            else:
                return 0