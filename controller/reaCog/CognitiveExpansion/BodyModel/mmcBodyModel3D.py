#!/usr/bin/env python

import numpy, math
import Hector.RobotSettings as RSTATIC
import controller.reaCog.WalknetSettings as WSTATIC

from .mmcAngularLegModelRobot import mmcAngularLegModel
from ...MotivationNetwork.MotivationUnit import executeNeuralNetStep
from decimal import Decimal as Decimal



##
#   A Body Model for a hexapod walker, based on MMC computation.
#   Now extended to 3 dimensions, (body segments are 3D and have now an orientation).
#
#   Higher level of the hierarchical body model for a six legged robot:
#   On this level
#       - there are three segments described through each six vectors
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

class mmcBodyModel:

    ### Initialisation of the body model.
    #   The segment vectors are encoded here, the leg vectors are initialised here
    #   but are actually set when a leg is really put on the ground (initially all are
    #   assumed in the air, so an update of the legs is forced in the first iteration)
    def __init__(self, wRobot, stab_thr):
        # Connect to the real robot - needed to establish 
        # connections to parts of walknet
        self.bodyModelDrawing = False
        # The stability is determined in the following way:
        # The connection between the two most hind legs touching the ground
        # (one leg on each side) is constructing a vector to the back.
        # The segment_factor is the fraction of the last body segment meeting this
        # vector (counted from in between middle and hind segment):
        #   0 = between the two segments (0 part of the hind segment)
        #   -1 = at the end of the hind segment_factor
        # The robot is determined instable when the segment_factor is greater
        # than this threshold.
        self.stability_threshold = stab_thr
        
        self.wRobot = wRobot
        self.motivationNetRobot = None
        
        temp_mmcLegs=[]
        for wleg in self.wRobot.wlegs:
#NEW leg_enabled            if wleg.leg.leg_enabled:
            temp_mmcLeg = mmcAngularLegModel(wleg)
            setattr(self, wleg.leg.name, temp_mmcLeg)
            temp_mmcLegs.append(temp_mmcLeg)
        self.mmcLegs=tuple(temp_mmcLegs)        
        
        # The height is currently set to a fixed value
        # Here a height net might be introduced?
        # The body model is now always enforcing this specific height.
        self.height = WSTATIC.stanceheight
        self.width = WSTATIC.stancewidth
                
        # Additional Vectors (from front of a segment to footpoint)
        self.front_vect = [numpy.array([-0.19, (self.width + 0.07),-self.height]),numpy.array([-0.19, -(self.width + 0.07),-self.height]),
                    numpy.array([-0.33,(self.width + 0.07),-self.height]),numpy.array([-0.33,-(self.width + 0.07),-self.height]),
                    numpy.array([-0.2, (self.width + 0.07),-self.height]),numpy.array([-0.2, -(self.width + 0.07),-self.height])]
        # Segment defining vectors: is constructed as a diamond for the mounting points of
        # the legs: segm_leg_ant = from coxa to front (anterior)
        #           segm_leg_post = from coxa to back (posterior)
        self.segm_post_ant = [numpy.array([0.22,0.0,0.0]),numpy.array([0.35, 0.0, 0.0]),numpy.array([0.36,0.0,0.0])]
        self.segm_post_ant_norm = [numpy.linalg.norm( segm_vect ) for segm_vect in self.segm_post_ant]          
        
        # Real Leg vectors 
        self.leg_vect = [numpy.array([ 0.  ,  self.width, -self.height]), numpy.array([ 0.  , -self.width, -self.height]), 
                numpy.array([ 0.  ,  self.width, -self.height]), numpy.array([ 0.  , -self.width, -self.height]), 
                numpy.array([ 0.  ,  self.width, -self.height]), numpy.array([ 0.  , -self.width, -self.height])]
    
        self.segm_leg_ant = [(self.leg_vect[i] - self.front_vect[i]) for i in range(0,6)]
        #self.segm_leg_ant = [(self.leg_vect[0] - self.front_vect[0]), (self.leg_vect[1] - self.front_vect[1])] 
        self.segm_leg_ant_norm = [numpy.linalg.norm( segm_vect ) for segm_vect in self.segm_leg_ant]
        #self.segm_leg_post = [(self.segm_leg_ant[0] - self.segm_post_ant[0]), (self.segm_leg_ant[1] - self.segm_post_ant[0])]
        self.segm_leg_post = [(self.segm_leg_ant[i] - self.segm_post_ant[i//2]) for i in range(0,6)]
        self.segm_leg_post_norm = [numpy.linalg.norm( segm_vect )  for segm_vect in self.segm_leg_post]
        self.segm_diag_to_right = [ (self.segm_leg_ant[i*2] - self.segm_leg_ant[i*2 + 1]) for i in range(0, len(self.segm_leg_ant)//2)]
        self.segm_diag_norm = [numpy.linalg.norm( segm_vect )  for segm_vect in self.segm_diag_to_right]
        
        # Help vectors for the drawing routines: We have to keep track of the feet
        # positions in a global coordinate system (and of one segment, too)
        self.segm1_in_global = numpy.array([-0.02,0.0,self.height])
        self.foot_global = [(self.segm1_in_global + self.segm_post_ant[1] + self.segm_post_ant[0] + self.front_vect[0]),
            (self.segm1_in_global + self.segm_post_ant[1] + self.segm_post_ant[0] + self.front_vect[1]),
            (self.segm1_in_global + self.segm_post_ant[1] + self.front_vect[2]),
            (self.segm1_in_global + self.segm_post_ant[1] + self.front_vect[3]),
            (self.segm1_in_global + self.front_vect[4]),
            (self.segm1_in_global + self.front_vect[5])]

        # Ground contact - which feet are on the ground
        self.gc = [False, False, False, False, False, False]
        self.old_stance_motivation = [False, False, False, False, False, False]
        
        # Vectors between footpoints - these are a fixed coordinate system which
        # shall not be altered. The standing feet are connected through the ground
        # and their relation shall be constant.
        # The footdiags table has to be adapted whenever the configuration of the walker
        # changes - when a leg is lifted from the ground this leg vector is not forming
        # closed kinematic chains with the other legs anymore and the footdiags can
        # not be exploited anymore.
        # When a leg is touching the ground, new footdiags to the other standing
        # legs have to be established.
        self.footdiag = [ [], [0], [0,0], [0,0,0], [0,0,0,0], [0,0,0,0,0] ]
        for i in range(0, len(self.footdiag) ) :
            for j in range(0,i) :
                self.footdiag[i][j] = self.set_up_foot_diag(i,j)
        # The explicit disturbance vectors of the network
        self.delta_front = [numpy.array([0,0,0]), numpy.array([0,0,0]), numpy.array([0,0,0])]
        self.delta_back = [numpy.array([0,0,0]), numpy.array([0,0,0]), numpy.array([0,0,0])]
        # These are operated through a pull at the front
        self.pull_front = numpy.array([0.0, 0.0,0.0])
        self.pull_back = numpy.array([0.0,0.0,0.0])
        self.step = 0
        self.damping = 5
        
        self.simulation = False
        self.Phase = None

        self.communication_interface = self.wRobot.robot.communication_interface
        self.simulation_sender=self.communication_interface.CreateBfbClient(14, ["SIMSERV_1_PROT"])
        if not(WSTATIC.real_robot):
            self.imu=self.communication_interface.CreateBfbClient(120, ["IMU_PROT"])
        
#CAUSE        self.old_swing_bm = [False for i in range(0,6)]
#CAUSE        self.leg_leading_to_problem = -1
        self.current_stability = True
        self.debug = False

    def appendPointList(self, start_point, end_point, pointList):
        pointList[0].append(start_point[0])
        pointList[1].append(start_point[1])
        pointList[2].append(start_point[2])
        pointList[3].append(end_point[0])
        pointList[4].append(end_point[1])
        pointList[5].append(end_point[2])

    ##
    #   Calculate the global position of the internal body model
    #   for the first segment of the body
    def getCurrentGlobalBodyPosition(self):
        # Search for one foot on the ground:
        foot_on_ground = 0
        while not(self.gc[foot_on_ground]) and not(foot_on_ground>4):
            foot_on_ground += 1
        segm1_vect = self.foot_global[foot_on_ground] - self.front_vect[foot_on_ground]
        if foot_on_ground > 1:
            segm1_vect += self.segm_post_ant[0]
        if foot_on_ground > 3:
            segm1_vect += self.segm_post_ant[1]
        return (segm1_vect)

    def turnOnCurrentBodyModelDrawing(self):
        if not(WSTATIC.real_robot):
            self.bodyModelDrawing = True
            self.initial_relative_translation = self.getCurrentGlobalBodyPosition()
            self.initial_robot_translation = numpy.array([self.imu.position[0]+0.77, self.imu.position[1],self.height-0.02])
            self.simulation_sender.robotTransparency = 0.4
        
    def turnOffCurrentBodyModelDrawing(self):
        self.bodyModelDrawing = False
        self.simulation_sender.robotTransparency = 1.
        self.simulation_sender.internalModelGlobalPosition = str("")

    def rotate_vector_around_z(self, rot_angle, input_v):
        return numpy.array([ (math.cos(rot_angle) * input_v[0] - math.sin(rot_angle) * input_v[1]), \
            (math.sin(rot_angle) * input_v[0] + math.cos(rot_angle) * input_v[1]), input_v[2]])

    def sendCurrentBodyModelDrawing(self):
        if (self.bodyModelDrawing) and not(WSTATIC.real_robot):
            relative_rotation = self.imu.rotation[2] - math.atan2(self.segm_post_ant[0][1], self.segm_post_ant[0][0])
            # Each line is represented by two points: Start and end point.
            # Each point consists of three dimensions. 
            # Data on lines is collected in one six-dimensional list of all lines:
            # each dimension represents one dimension - 
            #   start_x, start_y, start_z, end_x, end_y, end_z
            points = [[],[],[],[],[],[]]
        
            # initial_robot_position
            # initial_model_position
            # ??? HERE THE RELATIVE TRANSLATION SHOULD BE ROTATED TOO! initial_robot + rotate(relative_translation)
            relative_translation = self.initial_robot_translation + self.rotate_vector_around_z(relative_rotation, (self.getCurrentGlobalBodyPosition() - self.initial_relative_translation))
#2            print("Rel TL: ", relative_translation, " - rot: ", relative_rotation, self.getCurrentGlobalBodyPosition() )
            
#1            foot_on_ground = 0
#1            while not(self.gc[foot_on_ground]) and not(foot_on_ground>4):
#1                foot_on_ground += 1
#1            print("FOOT on ground: ", foot_on_ground, " = ", self.foot_global[foot_on_ground], self.front_vect[foot_on_ground])
#1            segm_test_vect = self.foot_global[foot_on_ground] - self.front_vect[foot_on_ground]
#1            if foot_on_ground > 1:
#1                segm_test_vect += self.segm_post_ant[0]
#1            if foot_on_ground > 3:
#1                segm_test_vect += self.segm_post_ant[1]
            
            current_rel_transl = relative_translation + self.rotate_vector_around_z(relative_rotation, (-self.segm_post_ant[0]))
        
            # Add front legs to visualization
            leg_points = self.front_right_leg.getLegPoints(self.segm_leg_post[1])
            for i in range(0,5):
                leg_points[i] = self.rotate_vector_around_z(relative_rotation, leg_points[i])
            for i in range (0,4):
                self.appendPointList((current_rel_transl + leg_points[i]), \
                    (current_rel_transl + leg_points[i+1]), points)
            leg_points = self.front_left_leg.getLegPoints(self.segm_leg_post[0])
            for i in range(0,5):
                leg_points[i] = self.rotate_vector_around_z(relative_rotation, leg_points[i])
            for i in range (0,4):
                self.appendPointList((current_rel_transl + leg_points[i]), (current_rel_transl + leg_points[i+1]), points)
        
            # Add middle legs to visualization
            current_rel_transl = current_rel_transl + self.rotate_vector_around_z(relative_rotation, (-self.segm_post_ant[1]))
            leg_points = self.middle_right_leg.getLegPoints(self.segm_leg_post[3])
            for i in range(0,5):
                leg_points[i] = self.rotate_vector_around_z(relative_rotation, leg_points[i])
            for i in range (0,4):
                self.appendPointList((current_rel_transl + leg_points[i]), (current_rel_transl + leg_points[i+1]), points)
            leg_points = self.middle_left_leg.getLegPoints(self.segm_leg_post[2])
            for i in range(0,5):
                leg_points[i] = self.rotate_vector_around_z(relative_rotation, leg_points[i])
            for i in range (0,4):
                self.appendPointList((current_rel_transl + leg_points[i]), (current_rel_transl + leg_points[i+1]), points)
            
            # Add hind legs to visualization
            current_rel_transl = current_rel_transl + self.rotate_vector_around_z(relative_rotation, (-self.segm_post_ant[2]))
            leg_points = self.hind_right_leg.getLegPoints(self.segm_leg_post[5])
            for i in range(0,5):
                leg_points[i] = self.rotate_vector_around_z(relative_rotation, leg_points[i])
            for i in range (0,4):
                self.appendPointList((current_rel_transl + leg_points[i]), (current_rel_transl + leg_points[i+1]), points)
            leg_points = self.hind_left_leg.getLegPoints(self.segm_leg_post[4])
            for i in range(0,5):
                leg_points[i] = self.rotate_vector_around_z(relative_rotation, leg_points[i])
            for i in range (0,4):
                self.appendPointList((current_rel_transl + leg_points[i]), (current_rel_transl + leg_points[i+1]), points)
            
            # Lines visualizing the main body are constructed
            self.appendPointList(relative_translation, (relative_translation + self.rotate_vector_around_z(relative_rotation, \
                (-self.segm_post_ant[0] - self.segm_post_ant[1] - self.segm_post_ant[2]) ) ), points)
    #       self.appendPointList(rel_translation, (rel_translation-self.segm[0][-1]), points)
    #       self.appendPointList((rel_translation-self.segm[0][-1]), (rel_translation-self.segm[0][-1] - self.segm[1][-1]), points)
    #       self.appendPointList((rel_translation-self.segm[0][-1] - self.segm[1][-1]), (rel_translation-self.segm[0][-1] - self.segm[1][-1] - self.segm[2][-1]), points)
        
            # Lines to the foot points of the internal model
    #       self.appendPointList((rel_translation - 0.9 * self.segm[0][-1]), (rel_translation + self.front_vect[0][-1]), points)
    #       self.appendPointList((rel_translation - 0.9 * self.segm[0][-1]), (rel_translation + self.front_vect[1][-1]), points)
    #       self.appendPointList((rel_translation - self.segm[0][-1] - 0.9 * self.segm[1][-1]), (rel_translation - self.segm[0][-1] + self.front_vect[2][-1]), points)
    #       self.appendPointList((rel_translation - self.segm[0][-1] - 0.9 * self.segm[1][-1]), (rel_translation - self.segm[0][-1] + self.front_vect[3][-1]), points)
    #       self.appendPointList((rel_translation - self.segm[0][-1] - self.segm[1][-1] - 0.2 * self.segm[2][-1]), (rel_translation - self.segm[0][-1] - self.segm[1][-1] + self.front_vect[4][-1]), points)
    #       self.appendPointList((rel_translation - self.segm[0][-1] - self.segm[1][-1] - 0.2 * self.segm[2][-1]), (rel_translation - self.segm[0][-1] - self.segm[1][-1] + self.front_vect[5][-1]), points)
            #self.bodyModel_tmp.append(points)
            #with open("/Users/mschilling/Desktop/body_model.txt", "wb") as myfile:
             #   numpy.save(myfile, self.bodyModel_tmp)
                #myfile.write(str(points)+'\n')
            self.simulation_sender.internalModelGlobalPosition = str(points)

    def set_motivation_net(self, motiv_net):
        self.motivationNetRobot = motiv_net

    def init_leg_mmc_models(self):
        for mmc_leg in self.mmcLegs:
            #print(mmc_leg.getInputPosition())
            mmc_leg.init_leg_mmc_model()
            #print("MMC LEG: ", mmc_leg.getInputPosition(), mmc_leg.getFootPosition())
            #print("WLEG   : ", mmc_leg.wleg.leg.getInputPosition() )
        
    def __getattr__(self,name):
        #print("Robot MMC Lookup: ", name)
        """ The MMC body model reroutes all calls which can not be answered to
            the real robot (which was set in initialization).
            This is mainly done to handle all the variables associated with the robot
            or the WRobot.
            The model is hiding the access functions for getting kinematic
            values and setting joint velocities (rerouted by the switch in SwitchRobot).
        """
        try :
            return getattr(self.wRobot, name)
        except AttributeError :
            print("Konnte ",name, "nicht in WRobot und Robot finden" )
            pass
        
    """ **** Set up methods and calculation of vector methods ***************************
    """
    ##  Calculation of the vector connection the feet of two standing legs.
    #   When a leg is put on the ground a new connecting vector has to be established.
    #   This is calculated as the difference between the leg vectors.
    def set_up_foot_diag(self, start, target) :
        diag_vec = self.leg_vect[target] - self.leg_vect[start] + \
            self.get_segm_vectors_between_legs(start, target)
        return diag_vec

    ##  Providing the segment vectors connecting two legs.
    #   An equation is described by a series of vectors forming a closed kinematic
    #   chain (two leg vectors, the footdiag and -possibly- segment vectors in
    #   between). This function is calculating the segment vectors between the
    #   given legs.
    def get_segm_vectors_between_legs(self, start_leg, end_leg):
        leg_diff = (end_leg//2-start_leg//2)
        if leg_diff == 0 :
            return ( self.segm_leg_post[start_leg] - self.segm_leg_post[end_leg] )
        elif leg_diff == 1:
            return ( self.segm_leg_post[start_leg] - self.segm_leg_ant[end_leg] )
        elif leg_diff == -1:
            return ( self.segm_leg_ant[start_leg] - self.segm_leg_post[end_leg] )
        elif leg_diff == 2 :
            return ( self.segm_leg_post[start_leg] - self.segm_post_ant[1] - self.segm_leg_ant[end_leg] )
        elif leg_diff == -2:
            return ( self.segm_leg_ant[start_leg] + self.segm_post_ant[1] - self.segm_leg_post[end_leg] )

    ##  Providing the segment vectors connecting two front (the additional) vectors.
    #   An equation is described by a series of vectors forming a closed kinematic
    #   chain (two front vectors, the footdiag and -possibly- segment vectors in
    #   between). This function is calculating the segment vectors between the
    #   given legs.
    def get_segm_vectors_between_front(self, start_leg, end_leg):
        leg_diff = (end_leg//2-start_leg//2)
        if leg_diff == 0 :
            return numpy.array([0., 0., 0.])
        elif abs(leg_diff) == 1 :
            return numpy.array(-1 * leg_diff * self.segm_post_ant[min(start_leg, end_leg)//2])
        elif abs(leg_diff) == 2 :
            return ( -0.5 * leg_diff * (self.segm_post_ant[0]
                + self.segm_post_ant[1]))
                
    ##  Sets ground contact to false and removes this leg from the body
    #   model computations. As the leg is not part of the closed kinematic chains
    #   after being lifted from the ground it shall not participate.
    def lift_leg_from_ground(self, leg_nr) :
        #print("Lift leg ", leg_nr, " from ground in internal model")
        if (self.gc[leg_nr]) :
            self.gc[leg_nr] = False
            
    ##  Sets the ground contact of the leg and initiates the calculation of connection
    #   vectors to all other standing legs (footdiag) which are used by the
    #   network.
    def put_leg_on_ground(self, leg_name, leg_vec) :
        #print("Put leg ", leg_name, " on ground in internal model ", leg_vec)
        leg_nr = RSTATIC.leg_names.index(leg_name)
        i = 0
        if (self.gc[leg_nr] == False) :
            #print("... done")
            # Set leg and diag vector
            self.leg_vect[leg_nr] = numpy.array(leg_vec)
            self.front_vect[leg_nr] = self.leg_vect[leg_nr] - self.segm_leg_ant[leg_nr]

            # Construction of all foot vectors - the ones to legs in the air are not used!
            for i in range(0,leg_nr) :
                self.footdiag[leg_nr][i] = self.set_up_foot_diag(leg_nr, i)
            for i in range(leg_nr+1,6):
                self.footdiag[i][leg_nr] = self.set_up_foot_diag(i, leg_nr)
            # Derive the global position (needed for the graphics output)
            for i in range(0,6):
                if (self.gc[i]):
                    if (i > leg_nr):
                        self.foot_global[leg_nr] = self.foot_global[i] + self.footdiag[i][leg_nr]
                    else:
                        self.foot_global[leg_nr] = self.foot_global[i] - self.footdiag[leg_nr][i]
                    break
            self.gc[leg_nr] = True
            self.old_stance_motivation[leg_nr] = True

    def updateSingleLegState(self, motiv_leg, leg_nr):
        if not(self.mmcLegs[leg_nr].predictedGroundContact()):
            #if (motiv_leg.inSwingPhase()):
            #print("SWING ", leg_nr)
            if (self.old_stance_motivation[leg_nr]):
                #self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(leg_nr)
                self.lift_leg_from_ground(leg_nr)
                #print(motiv_leg.wleg.leg.name, " starts swing - ", self.motivationNetRobot.pD_unstableCheck_HL.iteration)
                #print("SWING Motiv: ", motiv_leg.swing_motivation.output_value, " / ", motiv_leg.swing_motivation_toFront.output_value,
                # " - stance: ", motiv_leg.stance_motivation.output_value, " / ", motiv_leg.stance_motivation_toBack.output_value,
                # " BehPEP: ", motiv_leg.behindPEP.output_value, " / ", motiv_leg.swing_starter.output_value)
            self.old_stance_motivation[leg_nr] = False
            motiv_leg.stance_net.resetStanceTrajectory()
                    
            # Also for the swing phase update the leg positions index
            # the body model - only needed for visualization
            self.updateLegVectorsFromLegModel(motiv_leg.wleg.leg.name)
                    
            #print(self.gc)
        else:
            if not(self.old_stance_motivation[leg_nr]):
                self.put_leg_on_ground(motiv_leg.wleg.leg.name, self.mmcLegs[leg_nr].getFootPosition() )
                #print(motiv_leg.wleg.leg.name, " starts stance - ", self.motivationNetRobot.pD_unstableCheck_HL.iteration)
            else:
                self.updateLegVectorsFromLegModel(motiv_leg.wleg.leg.name)
            self.old_stance_motivation[leg_nr] = True

    ##  Update the current state of the legs.
    #   Only legs in stance mode are part of the body model (which is used for computation
    #   of the stance movement). When a leg switches between states it has to be added
    #   or removed from the body model.
    def updateLegStates(self, debug_inf=False):
        self.sendCurrentBodyModelDrawing()
        #print("HR Pos: ", self.mmcLegs[5].input_foot_position)
        if (self.Phase.MU_Sim.output_value > 0.) or (self.Phase.MU_Beh.output_value > 0.) or (self.Phase.MU_TestBeh.output_value > 0.):
            for motiv_leg, leg_nr in zip(self.motivationNetRobot.motivationNetLegs, range(len(self.motivationNetRobot.motivationNetLegs))):
                self.updateSingleLegState(motiv_leg, leg_nr)
#                 if (motiv_leg.inSwingPhase()):
#                 #if (motiv_leg.inSwingPhase()):
#                     #print("SWING ", leg_nr)
#                     if (self.old_stance_motivation[leg_nr]):
#                         #self.motivationNetRobot.bodyModelStance.lift_leg_from_ground(leg_nr)
#                         self.lift_leg_from_ground(leg_nr)
#                         #print(motiv_leg.wleg.leg.name, " starts swing")
#                         #print("SWING Motiv: ", motiv_leg.swing_motivation.output_value, " / ", motiv_leg.swing_motivation_toFront.output_value,
#                         # " - stance: ", motiv_leg.stance_motivation.output_value, " / ", motiv_leg.stance_motivation_toBack.output_value)
#                     self.old_stance_motivation[leg_nr] = False
#                     motiv_leg.stance_net.resetStanceTrajectory()
#                     
#                     # Also for the swing phase update the leg positions index
#                     # the body model - only needed for visualization
#                     self.updateLegVectorsFromLegModel(motiv_leg.wleg.leg.name)
#                     
#                     #print(self.gc)
#                 else:
#                     if not(self.old_stance_motivation[leg_nr]):
#                         self.put_leg_on_ground(motiv_leg.wleg.leg.name, self.mmcLegs[leg_nr].getFootPosition() )
#                         print(motiv_leg.wleg.leg.name, " starts stance")
#                     else:
#                         self.updateLegVectorsFromLegModel(motiv_leg.wleg.leg.name)
#                     self.old_stance_motivation[leg_nr] = True
# #            print("After Update gc: ", self.gc)
                    
    def updateLegVectorsFromLegModel(self, leg_name):
        leg_nr = RSTATIC.leg_names.index(leg_name)
        leg_vec = self.mmcLegs[leg_nr].getFootPosition()
        i = 0
        # Set leg and diag vector

        self.leg_vect[leg_nr] = numpy.array(leg_vec)
        self.front_vect[leg_nr] = -self.segm_leg_ant[leg_nr] + leg_vec

        #print("MMC LEG: ", self.mmcLegs[leg_nr].getFootPosition(), self.segm_post_ant[2])
        # Construction of all foot vectors - the ones to legs in the air are not used!
#       for i in range(0,leg_nr) :
#           self.footdiag[leg_nr][i] = self.set_up_foot_diag(leg_nr, i, -1)
#       for i in range(leg_nr+1,6):
#           self.footdiag[i][leg_nr] = self.set_up_foot_diag(i, leg_nr, -1)


    """ **** Graphic methods: Simple drawing of the body model **************************
    """
    ##  Extract the global positions of the feet, the segments, diagonals ...
    def get_leg_triangle(self, leg):
        # Initially get the global segment pos:
        if ((getattr(self.motivationNetRobot, RSTATIC.leg_names[leg])).inSwingPhase()):
            if (leg%2 == 0):
                stance_leg = leg + 1
            else:
                stance_leg = leg - 1
        else:
            stance_leg = leg
        segm = [ (self.foot_global[stance_leg][0] - self.leg_vect[stance_leg][0] + self.segm_leg_post[stance_leg][0] + self.segm_post_ant[stance_leg//2][0]), \
            (self.foot_global[stance_leg][1] - self.leg_vect[stance_leg][1] + self.segm_leg_post[stance_leg][1] + self.segm_post_ant[stance_leg//2][1]), \
            (self.foot_global[stance_leg][2] - self.leg_vect[stance_leg][2] + self.segm_leg_post[stance_leg][2] + self.segm_post_ant[stance_leg//2][2])]
        return([ [ (segm[0] - self.segm_leg_ant[leg][0] + self.leg_vect[leg][0]), \
                (segm[0] - self.segm_leg_ant[leg][0]), \
                (segm[0]), \
                (segm[0] - self.segm_post_ant[leg//2][0]), \
                (segm[0] - self.segm_leg_ant[leg][0])], \
                [ (segm[1] - self.segm_leg_ant[leg][1] + self.leg_vect[leg][1]), \
                (segm[1] - self.segm_leg_ant[leg][1]), \
                (segm[1]), \
                (segm[1] - self.segm_post_ant[leg//2][1]), \
                (segm[1] - self.segm_leg_ant[leg][1])], \
                [ (segm[2] - self.segm_leg_ant[leg][2] + self.leg_vect[leg][2]), \
                (segm[2] - self.segm_leg_ant[leg][2]), \
                (segm[2]), \
                (segm[2] - self.segm_post_ant[leg//2][2]), \
                (segm[2] - self.segm_leg_ant[leg][2])] ] )
                
    ##  Extract the global positions of the feet, the segments, diagonals ...
    def get_manipulator_coordinates(self, leg):
        print(self.foot_global[leg], self.segm[leg//2])
        print(self.foot_global[leg][0], (self.foot_global[leg][0] - self.front_vect[leg][0]) )
        print((self.foot_global[leg][0] - self.front_vect[leg][0] - self.segm[leg//2][0]) )
        return [[self.foot_global[leg][0], (self.foot_global[leg][0] - self.front_vect[leg][0]), \
                 (self.foot_global[leg][0] - self.front_vect[leg][0] - self.segm[leg//2][0]), \
                 (self.foot_global[leg][0] - self.front_vect[leg][0] - self.segm[leg//2][0] + self.rear_vect[leg][0])], \
                [self.foot_global[leg][1], (self.foot_global[leg][1] - self.front_vect[leg][1]), \
                 (self.foot_global[leg][1] - self.front_vect[leg][1] - self.segm[leg//2][1]), \
                 (self.foot_global[leg][1] - self.front_vect[leg][1] - self.segm[leg//2][1] + self.rear_vect[leg][1])], \
                [self.foot_global[leg][2], (self.foot_global[leg][2] - self.front_vect[leg][2]), \
                 (self.foot_global[leg][2] - self.front_vect[leg][2] - self.segm[leg//2][2]), \
                 (self.foot_global[leg][2] - self.front_vect[leg][2] - self.segm[leg//2][2] + self.rear_vect[leg][2])]]
        
        
#       return([ [ self.foot_global[leg][0], \
#           (self.foot_global[leg][0] - self.leg_vect[leg][0]), \
#           (self.foot_global[leg][0] - self.leg_vect[leg][0] + self.segm_leg_post[leg][0]), \
#           (self.foot_global[leg][0] - self.leg_vect[leg][0] + self.segm_leg_post[leg][0] + self.segm_post_ant[leg//2][0]), \
#           (self.foot_global[leg][0] - self.leg_vect[leg][0])], \
#           [ self.foot_global[leg][1], \
#           (self.foot_global[leg][1] - self.leg_vect[leg][1]), \
#           (self.foot_global[leg][1] - self.leg_vect[leg][1] + self.segm_leg_post[leg][1]), \
#           (self.foot_global[leg][1] - self.leg_vect[leg][1] + self.segm_leg_post[leg][1] + self.segm_post_ant[leg//2][1]), \
#           (self.foot_global[leg][1] - self.leg_vect[leg][1])], \
#           [ self.foot_global[leg][2], \
#           (self.foot_global[leg][2] - self.leg_vect[leg][2]), \
#           (self.foot_global[leg][2] - self.leg_vect[leg][2] + self.segm_leg_post[leg][2]), \
#           (self.foot_global[leg][2] - self.leg_vect[leg][2] + self.segm_leg_post[leg][2] + self.segm_post_ant[leg//2][2]), \
#           (self.foot_global[leg][2] - self.leg_vect[leg][2])] ])
            
    """ **** Computation of the MMC equations *******************************************
    """
    ##  Compute the leg vectors: For all standing legs
    #   the new leg vectors are computed, summed and the mean is calculated
    #   (the old value is also integrated, weighted by the damping value)
    def compute_leg_computations_and_integrate(self, leg_nr):
        if self.gc[leg_nr]:
            #print("Iterate")
            equation_counter = 1
#B      segm_leg_vect = self.segm_leg_ant[leg_nr] + self.front_vect[leg_nr]
            segm_leg_vect = -self.delta_back[leg_nr//2] + self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr//2] + self.front_vect[leg_nr]
            segm_leg_vect += self.damping * self.leg_vect[leg_nr]
            equation_counter += self.damping
            for target_leg in range(0, len(self.gc)):
                if self.gc[target_leg] and target_leg != leg_nr:
                    part_vec = numpy.array(self.leg_vect[target_leg]) \
                        + self.get_segm_vectors_between_legs( leg_nr, target_leg)
                    if target_leg < leg_nr:
                        part_vec -= self.footdiag[leg_nr][target_leg]
                    elif target_leg > leg_nr:
                        part_vec += self.footdiag[target_leg][leg_nr]
                    segm_leg_vect += part_vec
                    equation_counter += 1               
            return (segm_leg_vect/equation_counter)
        else:
            return numpy.array(self.leg_vect[leg_nr])

    ##  Compute the front vectors: For all standing legs
    #   the new front vectors are computed, summed and the mean is calculated
    #   (the old value is also integrated, weighted by the damping value)
    def compute_front_computations_and_integrate(self, leg_nr):
        if self.gc[leg_nr]:
            equation_counter = 1
            new_front_vect = -self.delta_front[leg_nr//2] - self.segm_post_ant[leg_nr//2] + self.leg_vect[leg_nr] - self.segm_leg_post[leg_nr]
            new_front_vect += self.damping * self.front_vect[leg_nr] #NEW
            equation_counter += self.damping #NEW
            # Computations of connections between legs in ground contacts
            # Compute equations to all other standing legs using the footdiag - has been in compute_leg!
            for target_leg in range(0, len(self.gc)):
                if self.gc[target_leg] and target_leg != leg_nr:
                    part_vec = numpy.array(self.front_vect[target_leg]) \
                        + self.get_segm_vectors_between_front( leg_nr, target_leg)
                    if target_leg < leg_nr:
                        part_vec -= self.footdiag[leg_nr][target_leg]
                    elif target_leg > leg_nr:
                        part_vec += self.footdiag[target_leg][leg_nr]
                    new_front_vect += part_vec
                    equation_counter += 1
            return (new_front_vect/equation_counter)
        else:
            return numpy.array(self.front_vect[leg_nr])
        
    ##  Compute the segment vectors:
    #   Using equations including the two legs connected to the segment,
    #   integrating the explicit displacement given as delta
    #   and the recurrent old value of the vector.
    def compute_segment_leg_ant_computations_and_integrate(self, leg_nr):
        if self.gc[leg_nr]:
            equation_counter = 1
            new_segm_leg_ant = self.segm_leg_post[leg_nr] + self.segm_post_ant[leg_nr//2]
            # Neighboring leg with respect to leg_nr
#       leg_neighbor = leg_nr + (1 - 2*(leg_nr%2))
#       new_segm_leg_ant += ((-1)**leg_nr) * self.segm_diag_to_right[0] + \
#           self.segm_leg_post[leg_neighbor] + self.segm_post_ant[leg_nr//2]
#       equation_counter += 1
            new_segm_leg_ant += self.segm_leg_ant[leg_nr] * self.damping
            equation_counter += self.damping
            new_segm_leg_ant = new_segm_leg_ant/equation_counter
            return ((self.segm_leg_ant_norm[leg_nr]/numpy.linalg.norm(new_segm_leg_ant))*new_segm_leg_ant)
        else:
            return numpy.array(self.segm_leg_ant[leg_nr])

    ##  Compute the segment vectors:
    #   Using equations including the two legs connected to the segment,
    #   integrating the explicit displacement given as delta
    #   and the recurrent old value of the vector.      
    def compute_segment_leg_post_computations_and_integrate(self, leg_nr):
        if self.gc[leg_nr]:
            equation_counter = 1
            new_segm_leg_post = self.segm_leg_ant[leg_nr] - self.segm_post_ant[leg_nr//2]
#       leg_neighbor = leg_nr + (1 - 2*(leg_nr%2))
#       new_segm_leg_post += ((-1)**leg_nr) * self.segm_diag_to_right[0] + \
#           self.segm_leg_ant[leg_neighbor] - self.segm_post_ant[leg_nr//2]
#       equation_counter += 1
            new_segm_leg_post += self.segm_leg_post[leg_nr] * self.damping
            equation_counter += self.damping
            new_segm_leg_post = new_segm_leg_post/equation_counter
            return ((self.segm_leg_post_norm[leg_nr]/numpy.linalg.norm(new_segm_leg_post))*new_segm_leg_post)
        else:
            return numpy.array(self.segm_leg_post[leg_nr])

    ##  Compute the segment vectors:
    #   Using equations including the two legs connected to the segment,
    #   integrating the explicit displacement given as delta
    #   and the recurrent old value of the vector.      
    def compute_segm_post_ant_computations_and_integrate(self, seg_nr):
        equation_counter = 3
        new_segm_post_ant = self.segm_post_ant[seg_nr] + self.delta_front[seg_nr] - self.delta_back[seg_nr]
        new_segm_post_ant += -self.segm_leg_post[2*seg_nr] + self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[1+seg_nr*2]
        new_segm_post_ant += -self.segm_leg_post[1 + 2*seg_nr] - self.segm_diag_to_right[seg_nr] + self.segm_leg_ant[seg_nr*2]
        
        new_segm_post_ant += self.damping * self.segm_post_ant[seg_nr]
        equation_counter += self.damping

        new_segm_div = new_segm_post_ant/equation_counter
        return ((self.segm_post_ant_norm[seg_nr]/numpy.linalg.norm(new_segm_div))*new_segm_div)

    ##  Compute the segment vectors:
    #   Using equations including the two legs connected to the segment,
    #   integrating the explicit displacement given as delta
    #   and the recurrent old value of the vector.      
    def compute_segm_diag_computations_and_integrate(self, seg_nr):
        equation_counter = 2
        
        new_segm_diag = self.segm_leg_post[2*seg_nr] + self.segm_post_ant[seg_nr] - self.segm_leg_ant[1+seg_nr*2]
        new_segm_diag -= self.segm_leg_post[1+2*seg_nr] + self.segm_post_ant[seg_nr] - self.segm_leg_ant[seg_nr*2]
        
        new_segm_diag += self.damping * self.segm_diag_to_right[seg_nr]
        equation_counter += self.damping
        new_segm_diag = new_segm_diag/equation_counter
        return ((self.segm_diag_norm[seg_nr]/numpy.linalg.norm(new_segm_diag))*new_segm_diag)

    ##  The MMC Method:
    #   - the multiple computations are computed for each variable
    #   - the mean for each variable is calculated
    #   The new values are appended to the list of element values.
    #   For each variable new values are calculated through
    #   different equations.
    ##          
    def mmc_iteration_step(self):
        self.delta_front[0] = self.pull_front
        if (self.gc[0]):
            self.delta_front[1] = (self.leg_vect[0] - self.segm_leg_post[0] - self.front_vect[0] - self.segm_post_ant[0])
        elif (self.gc[1]):
            self.delta_front[1] = (self.leg_vect[1] - self.segm_leg_post[1] - self.front_vect[1] - self.segm_post_ant[0])
        if (self.gc[2]):
            self.delta_front[2] = (self.leg_vect[2] - self.segm_leg_post[2] - self.front_vect[2] - self.segm_post_ant[1])
        elif (self.gc[3]):
            self.delta_front[2] = (self.leg_vect[3] - self.segm_leg_post[3] - self.front_vect[3] - self.segm_post_ant[1])
        #self.delta_back[2] = self.pull_back
        if (self.gc[4]):
            self.delta_back[1] = -(self.leg_vect[4] - self.segm_leg_post[4] - self.front_vect[4] - self.segm_post_ant[2])
        elif (self.gc[5]):
            self.delta_back[1] = -(self.leg_vect[5] - self.segm_leg_post[5] - self.front_vect[5] - self.segm_post_ant[2])
        if (self.gc[2]):
            self.delta_back[0] = -(self.leg_vect[2] - self.segm_leg_post[2] - self.front_vect[2] - self.segm_post_ant[1])
        elif (self.gc[3]):
            self.delta_back[0] = -(self.leg_vect[3] - self.segm_leg_post[3] - self.front_vect[3] - self.segm_post_ant[1])

        front_vect = [self.compute_front_computations_and_integrate(i) for i in range(0,6)]
        leg_vect = [self.compute_leg_computations_and_integrate(i) for i in range(0,6)]
        segm_leg_ant = [self.compute_segment_leg_ant_computations_and_integrate(i) for i in range(0,6)]
        segm_leg_post = [self.compute_segment_leg_post_computations_and_integrate(i) for i in range(0,6)]
        segm_post_ant = [self.compute_segm_post_ant_computations_and_integrate(i) for i in range(0,3)]
        # To make the whole body stiff (one body segment) simply align the three body segments.
        #segm_post_ant[1] = ((self.segm_post_ant_norm[1]/numpy.linalg.norm(segm_post_ant[0]))*segm_post_ant[0])
        #segm_post_ant[2] = ((self.segm_post_ant_norm[2]/numpy.linalg.norm(segm_post_ant[0]))*segm_post_ant[0])
        segm_diag_to_right = [self.compute_segm_diag_computations_and_integrate(i) for i in range(0,3)]
        for i in range(0, 6):
            self.segm_leg_ant[i] = segm_leg_ant[i]
            self.segm_leg_post[i] = segm_leg_post[i]
            self.front_vect[i] = front_vect[i]
            self.leg_vect[i] = leg_vect[i]
        for i in range(0, 3):
            self.segm_diag_to_right[i] = segm_diag_to_right[i]
            self.segm_post_ant[i] = segm_post_ant[i]
        self.step += 1
        self.current_stability = self.update_static_stability_along_segment()
        
    def get_current_static_stability_caused_by_leg(self, leg_nr):
        if not(self.current_stability): #CAUSEself.leg_leading_to_problem :
            return (self.get_ground_contact(leg_nr) == True)
        return True

    """ **** Get, set methods - connection to the robot simulator ***********************
    """
    ##  Pull the body model into a direction relative to the current first
    #   segment. Takes an angle (0 os straight ahead) and a velocity factor
    #   (around 0.1-0.2 should be fine) to come up with a corresponding pull vector.
    def pullBodyModelAtFrontIntoRelativeDirection(self, pull_angle, speed_fact):
        pull_angle_BM = pull_angle + math.atan2( self.segm_post_ant[0][1], self.segm_post_ant[0][0])
        self.pull_front[0] = speed_fact * math.cos(pull_angle_BM) # pull x
        self.pull_front[1] = speed_fact * math.sin(pull_angle_BM) # pull y

    ##  Pull the body model into a direction relative to the last body
    #   segment. Takes an angle (0 means straight backwards) and a velocity factor
    #   (around 0.1 - positive means backwards walking!)
    #   to come up with a corresponding pull vector.
    def pullBodyModelAtBackIntoRelativeDirection(self, pull_angle, speed_fact):
        pull_angle_BM = pull_angle + math.atan2( -self.segm_post_ant[2][1], -self.segm_post_ant[2][0])
        self.pull_back[0] = speed_fact * math.cos(pull_angle_BM) # pull x
        self.pull_back[1] = speed_fact * math.sin(pull_angle_BM) # pull y

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
        target_vec_wn = [self.leg_vect[leg_nr][0], self.leg_vect[leg_nr][1], self.leg_vect[leg_nr][2], 0]
        return target_vec_wn
        
    def get_ground_contact(self, leg_nr):
        return self.gc[leg_nr]
        
    def get_leg_in_stance(self, leg_nr):
        stance = not((getattr(self.motivationNetRobot, RSTATIC.leg_names[leg_nr])).inSwingPhase())
        return stance
        
    ##
    # Applying velocities to joints and leg models.
    # Method is reimplementation of the control signal send method for real robot
    # - here applied for internal simulation.
    def sendAllAngleVelocity(self):
        # Right now, the simulator is reset to the current state of the real body.
        if (self.Phase.MU_Sim.output_value > 0.5):
            if (self.simulation):
                #print("MMC Velocities: ", self.front_right_leg.controlVelocities, \
                #   "ST: ", self.motivationNetRobot.front_right_leg.stance_motivation.output_value, \
                #   " - SW: ", self.motivationNetRobot.front_right_leg.swing_motivation.output_value)
                for leg in self.mmcLegs:
                    leg.sendControlVelocities()
            else:           
                print("RESET MMC MODEL")
                self.motivationNetRobot.wrobot.switch.resetSystemToInitialStateOfInternalSimulation()
                # 5. Update stability calculation once
                self.update_static_stability_along_segment()
                self.simulation = True
        else:
            if ( (self.Phase.MU_TestBeh.output_value > 0.25) and self.simulation):
                print("RESET ALL VELOCITIES AND MMC NET FOR TEST OF BEHAVIOR")
                self.motivationNetRobot.cognitive_expansion.refreshInternalModelFromRobotData() 
                #print(self.gc)
                self.update_static_stability_along_segment()    
#           print("RESET ALL VELOCITIES")
            for motiv_leg in self.motivationNetRobot.motivationNetLegs:
                motiv_leg.wleg.controlVelocities = numpy.zeros(3)
            self.simulation = False

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
    
    # TODO: Wird 10 mal durchlaufen pro Iteration?
    def update_static_stability_along_segment(self):
        stability = True
        
        # Used for storing stability calculation in visualization
        self.temp_stability_fact_back = 0.5
        
#       print("STABILITY")
        left_leg, right_leg = 4, 5
        
        # Test if CoG moves moves behind the connecting line
        # connecting the leg on each side which
        # - has gc
        # - is the leg furthest backward having gc on that side
        # inSwingPhase(self)
#7        while left_leg>=0 and (not(self.get_ground_contact(left_leg))):
#7            left_leg -= 2
#7        while right_leg>0 and (not(self.get_ground_contact(right_leg))):
#7            right_leg -= 2
            
        while left_leg>=0 and not(self.mmcLegs[left_leg].predictedGroundContact()): # (not(self.get_ground_contact(left_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).inSwingPhase():
            left_leg -= 2
        while right_leg>0 and not(self.mmcLegs[right_leg].predictedGroundContact()): # (not(self.get_ground_contact(right_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).inSwingPhase():
            right_leg -= 2
#        while left_leg>=0 and (getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).inSwingPhase(): # not((getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).wleg.predictedGroundContact()): # (not(self.get_ground_contact(left_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[left_leg])).inSwingPhase():
 #           left_leg -= 2
  #      while right_leg>0 and (getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).inSwingPhase(): # not((getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).wleg.predictedGroundContact()): # (not(self.get_ground_contact(right_leg))): #(getattr(self.motivationNetRobot, RSTATIC.leg_names[right_leg])).inSwingPhase():
   #         right_leg -= 2
            
#        print("Middle left leg: ", self.mmcLegs[2].predictedGroundContact(), 
 #           " - ", self.mmcLegs[2].input_foot_position[2], 
  #          " / Swing: ",  self.motivationNetRobot.motivationNetLegs[2].swing_motivation.output_value,
   #         " / Stance: ",  self.motivationNetRobot.motivationNetLegs[2].stance_motivation.output_value)
            #" / ",  self.motivationNetRobot.motivationNetLegs[2].wleg.controlVelocities)
            #" vect: ", self.get_leg_vector('middle_left_leg'))
            
        # If there is no gc at all on one side it should be unstable
        if (left_leg < 0) or (right_leg < 0):
#           print("INSTABLE")
            stability = False
        else:
            #left_leg_obj = getattr(self.wRobot, RSTATIC.leg_names[left_leg])
            #right_leg_obj = getattr(self.wRobot, RSTATIC.leg_names[right_leg])
            #print(left_leg, right_leg, self.motivationNet.wrobot.hind_right_leg.predictedGroundContact(), self.motivationNet.hind_right_leg.inStancePhase() )
            
            #print(left_leg_obj.input_foot_position, right_leg_obj.input_foot_position)
            #print(-self.mmcStanceModel.front_vect[left_leg][-1], self.mmcStanceModel.front_vect[right_leg][-1])
            diag_vect = -self.front_vect[left_leg] + self.front_vect[right_leg] \
                        + self.get_segm_vectors_between_front(left_leg, right_leg)
            left_foot_cog_vect = -self.front_vect[left_leg]
            if (left_leg == 2):
                left_foot_cog_vect = -self.front_vect[left_leg] - self.segm_post_ant[1]
            if (left_leg == 0):
                left_foot_cog_vect = -self.front_vect[left_leg] - self.segm_post_ant[1] - self.segm_post_ant[0]
            left_foot_cog_vect[2] = 0.
#           print("Stability: ", left_leg, right_leg, diag_vect, left_foot_cog_vect)
            segment_factor = (diag_vect[1]*left_foot_cog_vect[0] - diag_vect[0]*left_foot_cog_vect[1]) \
                / (diag_vect[0]*self.segm_post_ant[1][1] - diag_vect[1]*self.segm_post_ant[1][0]) 
            # Correction factor of the parameter:
            # If the most hind leg is a middle leg, the factor has to be increased by one
            # - if both are front legs, it has to be increased by two.
# For legs further to the front: is now already counteracted above = in left_foot_cog_vect
#           segment_factor += (2-max(left_leg,right_leg)//2)
#           print("Stability Problem Detector ", segment_factor)

            # Used for storing stability calculation in visualization
            self.temp_stability_fact_back = segment_factor
#           if self.temp_stability_fact > self.stability_threshold:
#               print("Instable in COG EXP", self.temp_stability_fact)
#DEBUG              gc = [0, 0, 0, 0, 0, 0]
#DEBUG              for i in range(0, 6):
#DEBUG                  gc[i] = not((getattr(self.motivationNetRobot, RSTATIC.leg_names[i])).inSwingPhase())
#DEBUG              print(gc, left_leg, right_leg)
#               input()
            
            if segment_factor > self.stability_threshold:
                #print("Stability along middle segment - legs: ", left_leg, right_leg, " - factor ", segment_factor)
                stability = False
                #input("Press Enter to continue...")
#           print("Internal Model stability: ", segment_factor)
        #print("IM stability: ", self.temp_stability_fact, " - ", left_leg, " / ", right_leg)
        
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
            diag_vect = -self.front_vect[left_leg] + self.front_vect[right_leg] \
                        + self.get_segm_vectors_between_front(left_leg, right_leg)
            left_foot_cog_vect = -self.front_vect[left_leg]
            if (left_leg == 2):
                left_foot_cog_vect = -self.front_vect[left_leg] - self.segm_post_ant[1]
            if (left_leg == 0):
                left_foot_cog_vect = -self.front_vect[left_leg] - self.segm_post_ant[1] - self.segm_post_ant[0]
            left_foot_cog_vect[2] = 0.
            #print("Stability: ", left_leg, right_leg, diag_vect, left_foot_cog_vect)
            segment_factor = (diag_vect[1]*left_foot_cog_vect[0] - diag_vect[0]*left_foot_cog_vect[1]) \
                / (diag_vect[0]*self.segm_post_ant[1][1] - diag_vect[1]*self.segm_post_ant[1][0]) 
            self.temp_stability_fact_front = segment_factor
            #print("Stability fact front: ", self.temp_stability_fact_front, left_leg, right_leg, self.motivationNetRobot.motivationNetLegs[left_leg].wleg.realLeg.leg.input_foot_position[2], self.motivationNetRobot.motivationNetLegs[right_leg].wleg.realLeg.leg.input_foot_position[2], [self.motivationNetRobot.motivationNetLegs[i].wleg.predictedGroundContact() for i in range(0,6)])
            if self.temp_stability_fact_front < WSTATIC.stability_threshold_front:
                stability = False       
                print("INSTABLE AT FRONT in internal simulation")         
        
        return stability
