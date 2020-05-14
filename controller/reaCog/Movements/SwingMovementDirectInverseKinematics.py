#!/usr/bin/env python
# -*- coding: utf-8 -*-

##@author Jan, Marc

from ..MotivationNetwork.ModulatedRoutine import ModulatedRoutine
import numpy
import controller.reaCog.WalknetSettings as WSTATIC
import Hector.RobotSettings as RSTATIC
import math
import copy

##
#   Swing movement generation using direct inverse kinematics.
##
class SwingMovementDirectInverseKinematics(ModulatedRoutine):

    ## Initialisation of the swing controller
    def __init__(self, motivleg):
        ModulatedRoutine.__init__(self)
        self.motivationLeg = motivleg
        self.trajectory = None
        self.iteration =0
        self.target_point = numpy.zeros(3)
        self.__iControllerValue = numpy.zeros(3)
        
        self.iteration_steps_swing = 50
        
        self.saved_trajectory = numpy.zeros(3)
        self.saved_iteration = 0
        self.saved_iControllerValue = numpy.zeros(3)

        # Introduced slow down  for swing movement
        # is used to test probehandeln and to slow down the movement one time
        # The overall movement is stretched over a longer time period (additional steps)
        self.slow_down_additional_points = 0

    def __del__(self):
        pass

    ##
    # The function computes the trajectory for the swing movement of a single leg. 
    # @param aep The anterior extreme position of the leg (The desired touch-down position)
    # @param pep The posterior extreme position of the leg (The lift-up position)
    # @param dep The distal extreme position (The highest point [at least in the 2D coordinate system] of the trajectory)
    # In order to create the trajectory, first of all, a new 2D-coordinate system is introduced. The axes for this coordinate system 
    # are called 'a' and 'b'. Therefore, the coordinate system is called ab-system hereafter. 
    # The constraint for the creation of the ab-system is that the PEP lies at the point of origin, the AEP
    # must lie on the a-axis and the DEP must lie in the plane spanned by the two axes (in the positive quadrant). 
    # In this coordinate system, the trajectory is computed using a quadratic spline: One quadratic function connects PEP and DEP
    # while the other connects DEP and AEP. The first derivatives of the two function must be equal at the DEP. 
    ##
    def computeTrajectory(self, pep, dep, aep, numOfPoints ):   
#       if (self.motivationLeg.wleg.leg.name == "middle_left_leg"):
#           print("TRAJECTORY # ", numOfPoints)
        # crop to 1x3 vectors
#       print("PEP ", self.motivationLeg.wleg.leg.name, " : ", pep, " - leg: ", self.motivationLeg.wleg.leg.input_foot_position, self.motivationLeg.gc.output_value)
        pep = pep[0:3]
        dep = dep[0:3]
        aep = aep[0:3]
        #print("Traj ", pep, dep, aep)
        
        diffAepPep = aep - pep # Compute the vector between PEP and AEP
        normDiffAepPep = numpy.linalg.norm(diffAepPep) # Compute the length of the vector
        diffDepPep = dep - pep # Compute the vector that connects PEP and DEP
        projection = numpy.dot(diffDepPep, diffAepPep) / (numpy.power(normDiffAepPep,2))* diffAepPep # this is the projection of the vector diffDepPep onto the vector diffAepPEp. 
        # The result can be seen as the point of an intersection between the AEP-PEP-vector and a vector that is orthogonal to the AEP-PEP-vector and that points in the direction of the DEP. 
        normProjection=numpy.linalg.norm(projection)
        normDiffAepPep=numpy.linalg.norm(diffAepPep)
        normDiffProjectionPep=numpy.linalg.norm(projection-diffAepPep)
        
        # TODO: Currently the leg can start a swing movement while hanging in the air
        # = after an internal simulation the model is reset to the real state
        #   but no movement is initiated 
        # this leads to a problem in the trajectory calculation
        # SOLVE: put all legs on ground before planning
        if (pep[2] > -0.18):
            pep[2] = -0.18
#TODO: WHAT DOES THIS PROJECTION CHECK DO???        
#       if not (normProjection<normDiffAepPep>normDiffProjectionPep):
#           print(pep,dep,aep, numOfPoints)
#           print(self.motivationLeg.wleg.leg.name)
            #~ from code import interact; interact('The given interim position is not compatible with this trajectory computation method.',local=locals().copy())
#           raise Exception('The given interim position is not compatible with this trajectory computation method.')
        aAxisVec = diffAepPep/ normDiffAepPep # The a-axis of the new coordinate system
        bAxisVec = (diffDepPep - projection)/ numpy.linalg.norm(diffDepPep - projection) # The b-axis of the new coordinate system
        relDep = numpy.array([numpy.linalg.norm(projection),numpy.linalg.norm(diffDepPep -projection) ]) # This is the DEP in the ab-system
        relAep = numpy.array([normDiffAepPep , 0]) # This is the AEP in the ab-system. 
        # Now, the parameters of the two quadratic functions f and g are computed. 
        # The constraints are:
        # f(aPEP)==bPEP
        # f(aDEP)==bDEP
        # g(aDEP)==bDEP
        # g(aAEP)==bAEP
        # f'(aDEP)==g'(aDEP)
        # The quadratic functions are constructed in this way: f(x):=f1*x^2+f2*x+f3 and g(x):=g1*x^2+g2*x+g3
        f1 = -relDep[1] / (numpy.power(relDep[0],2))
        f2 = 2* relDep[1] / relDep[0]
        f3=0
        g1= -relDep[1] / numpy.power((relAep[0]- relDep[0]),2)
        g2 = 2* relDep[0]*relDep[1] / (numpy.power((relAep[0]-relDep[0]),2))
        g3 = relAep[0] * (relAep[0]-2*relDep[0])* relDep[1]/ numpy.power(relAep[0]-relDep[0],2)

        aPoints = numpy.arange(0,relAep[0],1 /((numOfPoints+self.slow_down_additional_points)-1)* relAep[0] ) # The a-coordinates (except the very last point) for the trajectory points
        aPoints =numpy.append(aPoints,relAep[0] ) # As a last point, the AEP is appended
        temp1 = (numpy.power(aPoints,2)*f1+aPoints*f2+f3) * (numpy.less_equal(aPoints, relDep[0])) # The b-coordinate for those points that will be represented by function f
        temp2 = (numpy.power(aPoints, 2)*g1+aPoints*g2+g3) * (numpy.greater(aPoints, relDep[0]) ) # The b-coordinate for those points that will be represented by function g
        bPoints  = temp1+temp2 # The b-coordinate for all points in aPoints
        temp1 = numpy.transpose(numpy.tile(pep,(len(aPoints),1))) # This temporary array is used only for convenience. It holds as many copies of the PEP as there are points in aPoints
        temp2 = numpy.tile(aPoints,(3,1))* numpy.tile(aAxisVec, (len(aPoints),1)).conj().T # This vector represents the position of all the points along the a-axis in three dimensional space relative to the PEP
        temp3 = numpy.tile(bPoints,(3,1))* numpy.tile(bAxisVec,(len(bPoints),1)).conj().T # This vector represents the position of all the points along the b-axis in three dimensional space relative to the PEP
        result = temp1+temp2 + temp3 # By summation of the PEP-, the aPoints- and the bPoints-representing vectors, the positions of the points on the trajectory in 3D-space are computed.
        if (self.slow_down_additional_points > 0):
            print("Slowed down leg " , self.motivationLeg.wleg.leg.name)
        self.slow_down_additional_points = 0
        return result

    def saveCurrentSwingState(self):
#       print("SAVED SWING NETWORK")
        self.saved_trajectory = copy.copy(self.trajectory)
        self.saved_iteration = self.iteration
        self.saved_iControllerValue = copy.copy(self.__iControllerValue)
        
    def resetToSavedSwingState(self):
        #print("RESETED SWING NETWORK", self.motivationLeg.wleg.leg.name, " - ", self.saved_iteration)
        self.trajectory = self.saved_trajectory
        self.iteration = self.saved_iteration
        self.__iControllerValue = self.saved_iControllerValue

    def resetSwingTrajectory(self):
#       if (self.motivationLeg.wleg.leg.name == "middle_left_leg"):
#           print("RESET SWING ", self.motivationLeg.wleg.leg.name)
        self.trajectory = None
        self.iteration = 0

    ##  Function called by the ModulatingMotivationUnit when active.
    #   Invokes the execution of the routine which has to be defined by the derived 
    #   classes.
    def modulatedRoutineFunctionCall(self, weight, target_point):
        #print("SWING CALL ", self.motivationLeg.wleg.leg.name, self.motivationLeg.wleg.leg.input_foot_position)
        self.target_point = target_point()
        #if weight <0.5 :
        #   self.trajectory = None
        #   self.iteration = 0
        if (self.motivationLeg.wleg.leg.leg_enabled) :
#           self.target_point = self.motivationLeg.aep_shifted
            if isinstance(self.trajectory, (type(None))):
#               if (self.motivationLeg.wleg.leg.name == "middle_left_leg"):
#                   print("TARGET POINT ", self.target_point, self.motivationLeg.wleg.leg.name)
                # we'll calculate a new swing trajectory from this point
                
                # reset temporary (per swing) variables 
                self.__iControllerValue = numpy.zeros(3)
#               current_pos = [0.1,  0.29, -0.2]
                current_pos = self.motivationLeg.wleg.leg.input_foot_position
                
                self.dep = self.motivationLeg.wleg.depcalc(current_pos, self.target_point)
                
                #~ print('DEBUG:',self.wleg.leg.name)
                #~ print('DEBUG: pep ',current_pos)
                #~ print('DEBUG: dep ',self.dep)
                #~ print('DEBUG: aep ',self.target_point)
                #~ print('DEBUG: aepA',self.wleg.aep_shifted)
                
                # This is the minimum distance between the current foot position of the leg, the DEP and the AEP. The real distance, however, will
                # be higher since the trajectory is curved, but the real distance cannot be computed (algebraically)
                min_distance = numpy.linalg.norm(self.dep-current_pos) + numpy.linalg.norm(self.target_point-self.dep)
                
#               print( current_pos, self.dep, self.target_point )
                self.target_point_high = numpy.array(self.target_point)
                #self.target_point_high[2] = -0.1
                #print(self.target_point_high)
                
                self.iteration_steps_swing = math.ceil((min_distance/WSTATIC.default_swing_velocity)*RSTATIC.controller_frequency)

                self.trajectory = self.computeTrajectory(current_pos,self.dep,self.target_point_high,math.ceil((min_distance/WSTATIC.default_swing_velocity)*RSTATIC.controller_frequency) )

                search_distance = 0.1
                search_dep = numpy.array(self.target_point_high)
                search_dep[0] += search_distance*0.5
                #search_dep[1] = 0.24
                search_dep[2] += 0.1
                search_aep = numpy.array(self.target_point_high)
                search_aep[0] += search_distance
                #search_aep[1] = 0.265
                search_trajectory = self.computeTrajectory(self.target_point_high, search_dep, search_aep, 60)
                self.trajectory = numpy.concatenate((self.trajectory, search_trajectory), axis=1)
                #print(self.trajectory.shape)

#                print("Swing net init ", self.motivationLeg.wleg.leg.name, self.motivationLeg.wleg.leg.input_foot_position , " - ", self.trajectory[0][-1] , self.trajectory[1][-1], self.trajectory[2][-1])
#SW             
#               import matplotlib.pylab as py
#               import matplotlib.pyplot as plt
#               plt.ion()
#               traject= plt.figure(figsize=(6, 6))
#               traject_plot = traject.add_subplot(111)
#               print( len(range(len(self.trajectory[0]))), len(self.trajectory[0] ) )
#               traject_plot.plot(range(len(self.trajectory[0])), self.trajectory[0])
#               self.footfall.canvas.draw()
#               plt.ioff()
#               input()
                
                #print('new trajectory:')
                #print(self.trajectory)
            try :
                nextpoint = self.trajectory[:,self.iteration]
            except IndexError :
                # The end of the trajectory has been reached, but it has
                # not yet been switched back to stance, usually because
                # there was no gc detected yet. Maybe there's a hole? ;)
                # A: don't move, wait for the ground to come..
                nextpoint = self.trajectory[:,-1]
                # B: search for ground at a lower position (1cm/iter.)
                #~ self.trajectory[:,-1] = self.trajectory[:,-1]-[0,0,0.01]
                #~ nextpoint = self.trajectory[:,-1]
            nextpoint = numpy.append(nextpoint,0)
            self.iteration +=1
            #~ print('DEBUG: nextpoint',nextpoint)
            
            
            # now it's just a matter of moving the leg to the next pos
            
            # what are the current input angles
            currentInputAngles = self.motivationLeg.wleg.leg.getInputPosition()
            # what value should they have in the next iteration
            nextangles = self.motivationLeg.wleg.leg.computeInverseKinematics(nextpoint)
            # what's the difference
            delta = nextangles - numpy.array(currentInputAngles)
            if WSTATIC.controlViaPI:
                # use a PI-controller
                self.__motorControlPI(delta)
            else:
                # go for nextpoint directly
                self.__motorControlSimple(delta)


    #@brief sets angle velocity to reach target under ideal conditions
    #@param angleDiff   difference of current to desired input angles
    def __motorControlSimple(self, angle_diff):
        # considering the time that the motors have to move to the new
        # position, at which speed do they have to rotate
        angle_vel = angle_diff * RSTATIC.controller_frequency
        
        if self.motivationLeg.wleg.leg.leg_enabled:
            self.motivationLeg.wleg.addControlVelocities(angle_vel)
#           if(self.motivationLeg.wleg.leg.name == "hind_right_leg"):
#               print("Current SWING HR: ", self.motivationLeg.wleg.leg.getInputPosition(), angle_vel )
            
            
    #@brief updates the angle velocity for one leg using PI controller
    #@param angleDiff   difference of current to desired input angles
    def __motorControlPI(self, angle_diff):
#       print(self.wleg.leg.name, " motor control PI")
        # add the angular difference to the I controller
        self.__iControllerValue += angle_diff[0:3]
        
        # compute the result of the PI-controller
        P = angle_diff * WSTATIC.kp
        I = self.__iControllerValue * WSTATIC.ki
        angle_vel = (P+I) * RSTATIC.controller_frequency
        
        if self.motivationLeg.wleg.leg.leg_enabled:
            self.motivationLeg.wleg.addControlVelocities(angle_vel)
