#!/usr/bin/env python

import numpy, math
import Hector.RobotSettings as RSTATIC
import controller.reaCog.WalknetSettings as WSTATIC
from .SWIG_MMC_Lib.mmcBody import MmcBody

##
#   A Body Model for a hexapod walker, based on MMC computation.
#   Now extended to 3 dimensions, (body segments are 3D and have now an orientation).
#   In this version, the expensive computations are carried out in C++
#   which is called using SWIG.
#
#   Higher level of the hierarchical body model for a six legged robot:
#   On this level
#       - there are three segment vectors
#       - each leg is represented only as a vector to the tip of the leg
#           for each leg there are two vectors - one starting at the front of the segment
#           (the leg vector) and one starting at the back of the segment (diag vector)
#           both ending at the tip of the leg
#   The body model is constructed as a Mean of Multiple Computation network:
#       - each variable vector is described by multiple kinematic relations
#           = computations
#       - these different computations are integrated using a mean calculation
#   The resulting set of equations directly defines a neural network connection matrix.
#   This can be computed in an iterative fashion - introduction of a recurrent connection
#   damps the overall relaxation procedure and prevents oscillations.
#   The network is able to compute forward, inverse and any mixed kinematic problem and
#   is quite fast (it only needs a few iteration steps).
#   It acts as an autoassociator - so a problem is given to the network as an incomplete
#   set of variables and the network fills in complementing missing values.
#   The network used here is special as it uses no specific reference coordinate system.
#   Described in the network are only relative vectors between certain body parts.
#   The footdiag vectors connecting the feet of the standing legs are actually
#   constituting a fixed reference frame in which coordinate systems and
#   representation of the environment can be grounded.
#   To move the body actually, there are special vectors in the body model which
#   explicitly represent a disturbance of the dynamics of the network (the pull vector
#   which is driving the delta vectors of the segments). One can envision this as
#   a passive motion paradigm: the body model is pulled into direction of this vector
#   and the model is following this movement (one can think of this as a stick model which
#   legs are attached to the ground).
#
#   The body model is used in connection to leg models. The leg vectors are enforced onto
#   the lower leg networks which compute corresponding joint angles for the legs from
#   this. Overall this is embedded in a processing loop:
#       - the leg network is updated by sensory data
#           (iterates once to integrate the values)
#       - the leg network updates the body model
#       - the body model gets a movement command (usually constant)
#           and is iterated (one step is sufficient)
#       - the body model pushes down the new leg vectors for the standing legs into
#           the leg networks (these are iterated for a few iteration steps and provide
#           joint angles)
#       - the joint motors are controlled by the new motor commands
##

class mmcBodyModelStance:

    ### Initialisation of the body model.
    #   The segment vectors are encoded here, the leg vectors are initialised here
    #   but are actually set when a leg is really put on the ground (initially all are
    #   assumed in the air, so an update of the legs is forced in the first iteration)
    def __init__(self, motiv_net, stab_thr):
        self.motivationNetRobot = motiv_net
        self.height_hind = -WSTATIC.hind_initial_aep[2]
        self.height_middle = -WSTATIC.middle_initial_aep[2]#WSTATIC.stanceheight
        self.height_front = -WSTATIC.front_initial_aep[2] #WSTATIC.stanceheight + 0.02
        self.width = WSTATIC.middle_initial_aep[1]#WSTATIC.stancewidth
        self.swig_stance_body_model = MmcBody(self.height_front, self.height_middle, self.height_hind, self.width, 5, stab_thr)
        # These are operated through a pull at the front
        self.pull_front = numpy.array([0.1, 0.05,0.0])
        self.pull_back = numpy.array([0.0,0.0,0.0])
        self.old_stance_motivation = [False, False, False, False, False, False]
        self.pull_angle_front = 0.
        self.speed_fact_front = 0.
        self.pull_angle_back = 0.
        self.speed_fact_back = 0.
        
#CAUSE        self.old_swing_bm = [False for i in range(0,6)]
#CAUSE        self.leg_leading_to_problem = -1
        self.current_stability = True
        
        # Used for storing stability calculation in visualization
        self.temp_stability_fact_back = 0.
        self.temp_stability_fact_front = 0.
        # The robot is determined instable when the segment_factor is greater
        # than this threshold.
        self.stability_threshold = stab_thr
        #self.last_stability = [True, True, True, True, True, True]
        
    """ **** Set up methods and calculation of vector methods ***************************
    """     
    ##  Sets ground contact to false and removes this leg from the body
    #   model computations. As the leg is not part of the closed kinematic chains
    #   after being lifted from the ground it shall not participate.
    def lift_leg_from_ground(self, leg_nr) :
        self.swig_stance_body_model.lift_leg_from_ground(leg_nr)
            
    ##  Sets the ground contact of the leg and initiates the calculation of connection
    #   vectors to all other standing legs (footdiag) which are used by the
    #   network.
    def put_leg_on_ground(self, leg_name, leg_vec) :
        #print("PUT on ground ", leg_name)
        leg_nr = RSTATIC.leg_names.index(leg_name)
        #leg_vec[2] = -self.height
        if leg_nr < 2:
            leg_vec[2] = -self.height_front
        elif leg_nr < 4:
            leg_vec[2] = -self.height_middle
        else:
            leg_vec[2] = -self.height_hind
        self.swig_stance_body_model.put_leg_on_ground(leg_nr, leg_vec)
            
    ##  Update the current state of the legs.
    #   Only legs in stance mode are part of the body model (which is used for computation
    #   of the stance movement). When a leg switches between states it has to be added
    #   or removed from the body model.
    def updateLegStates(self):
        for motiv_leg, leg_nr in zip(self.motivationNetRobot.motivationNetLegs, range(len(self.motivationNetRobot.motivationNetLegs))):
            if (motiv_leg.inSwingPhase()):
            #if not(motiv_leg.wleg.predictedGroundContact()):
                if (self.old_stance_motivation[leg_nr]):
                    self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(leg_nr)
                self.old_stance_motivation[leg_nr] = False
            else:
                self.old_stance_motivation[leg_nr] = True

    """ **** Get, set methods - connection to the robot simulator ***********************
    """
    ##  Pull the body model into a direction relative to the current first
    #   segment. Takes an angle (0 os straight ahead) and a velocity factor
    #   (around 0.1-0.2 should be fine) to come up with a corresponding pull vector.
    def pullBodyModelAtFrontIntoRelativeDirection(self, pull_angle, speed_fact):
        if (self.pull_angle_front != pull_angle) or (self.speed_fact_front != speed_fact):
            self.swig_stance_body_model.set_pull_front( pull_angle, speed_fact )
            self.pull_angle_front = pull_angle
            self.speed_fact_front = speed_fact

    ##  Pull the body model into a direction relative to the last body
    #   segment. Takes an angle (0 means straight backwards) and a velocity factor
    #   (around 0.1 - positive means backwards walking!)
    #   to come up with a corresponding pull vector.
    def pullBodyModelAtBackIntoRelativeDirection(self, pull_angle, speed_fact):
        if (self.pull_angle_back != pull_angle) or (self.speed_fact_back != speed_fact):
            self.swig_stance_body_model.set_pull_back( pull_angle, speed_fact )
            self.pull_angle_back = pull_angle
            self.speed_fact_back = speed_fact

    def mmc_iteration_step(self):
        self.swig_stance_body_model.mmc_iteration_step()
        self.current_stability = self.update_static_stability_along_segment()
        
    def get_current_static_stability_caused_by_leg(self, leg_nr):
        old_stab = True
        if not(self.current_stability): 
            old_stab == self.get_ground_contact(leg_nr)
        if not(self.current_stability): #CAUSEself.leg_leading_to_problem :
            return (self.get_ground_contact(leg_nr) == True)
        return True

    ##  Get the angles between the inner segments which are used for the
    #   segment joints.
    def get_segment_angles(self):
        front_angle = math.atan2(self.segm_post_ant[0][1],self.segm_post_ant[0][0])
        middle_angle = math.atan2(self.segm_post_ant[1][1],self.segm_post_ant[1][0])
        back_angle = math.atan2(self.segm_post_ant[2][1],self.segm_post_ant[2][0])
        angle1=front_angle - middle_angle
        if angle1<-3.14:
            angle1+=2*math.pi
        elif angle1>3.14:
            angle1-=2*math.pi
        angle2=middle_angle - back_angle
        if angle2<-3.14:
            angle2+=2*math.pi
        elif angle2>3.14:
            angle2-=2*math.pi
        return ([angle1,angle2])
        
    def get_leg_vector(self, leg_name):
        leg_nr = RSTATIC.leg_names.index(leg_name)
        return self.swig_stance_body_model.get_leg_vector(leg_nr)
        
    def get_leg_triangle(self, leg_nr):
        return self.swig_stance_body_model.get_leg_triangle(leg_nr)
        
    def get_ground_contact(self, leg_nr):
        return self.swig_stance_body_model.get_ground_contact(leg_nr)
        
    ##
    #   Check if the BM configuration is static stable.
    #   In this version two line equations are used (in parametric version)
    #       - diagonal between left and right most hind leg with gc
    #       - line along the middle segment
    #   It is calculated where those two lines intersect, i.e. with respect to the 
    #   middle segment: the segment factor specifies this intersection as expressed
    #   in the line equation of this line: a negative value means that this 
    #   point is behind the middle segment (= the diagonal between the most hind legs
    #   lies behind the center of gravity). For a positive value the factor describes
    #   how CoG and the hind line of the polygon of static stability relate.
    def update_static_stability_along_segment(self):
        if not(self.motivationNetRobot.wrobot.switch.decoupled):
            stability = True
            left_leg, right_leg = 4, 5
        
            # Test if CoG moves moves behind the connecting line
            # connecting the leg on each side which
            # - has gc
            # - is the leg furthest backward having gc on that side
            # inSwingPhase(self)
            while left_leg>=0 and not((getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).wleg.predictedGroundContact()): # (not(self.get_ground_contact(left_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).inSwingPhase():
                left_leg -= 2
            while right_leg>0 and not((getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).wleg.predictedGroundContact()): # (not(self.get_ground_contact(right_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).inSwingPhase():
                right_leg -= 2

            self.left_leg = left_leg
            self.right_leg = right_leg
            # If there is no gc at all on one side it should be unstable
            if (left_leg < 0) or (right_leg < 0):
                stability = False
            else:
                self.temp_stability_fact_back = self.swig_stance_body_model.check_static_stability(left_leg, right_leg)          
                if self.temp_stability_fact_back > self.stability_threshold:
                    stability = False 
                    print("INSTABLE AT BACK")         
                    
            # Check stability at front:
            left_leg, right_leg = 0, 1
            # Test if CoG moves moves in front of the connecting line
            # connecting the leg on each side which
            # - has gc
            # - is the leg furthest forward having gc on that side
            while left_leg<=4 and not((getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).wleg.predictedGroundContact()): # (not(self.get_ground_contact(left_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).inSwingPhase():
                left_leg += 2
            while right_leg<6 and not((getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).wleg.predictedGroundContact()): # (not(self.get_ground_contact(right_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).inSwingPhase():
                right_leg += 2
            # If there is no gc at all on one side it should be unstable
            if not((left_leg > 4) or (right_leg >5)):
                self.temp_stability_fact_front = self.swig_stance_body_model.check_static_stability(left_leg, right_leg)          
                #print("Stability fact front: ", self.temp_stability_fact_front, left_leg, right_leg, self.motivationNetRobot.motivationNetLegs[left_leg].wleg.realLeg.leg.input_foot_position[2], self.motivationNetRobot.motivationNetLegs[right_leg].wleg.realLeg.leg.input_foot_position[2], [self.motivationNetRobot.motivationNetLegs[i].wleg.predictedGroundContact() for i in range(0,6)])
                if self.temp_stability_fact_front < WSTATIC.stability_threshold_front:
                    stability = False       
                    print("INSTABLE AT FRONT")         
                                
            return stability
        else:
            return True
