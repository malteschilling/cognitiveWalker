#from Hector.LegF import Leg
import numpy
import math
from .. import WalknetSettings as WSTATIC
from tools.FreezableF import Freezable as Freezable


#from Bodymodel.legNetworkDualQuaternionMmc import legNetworkDualQuaternionMmc
#from controller.reaCog.Movements.BodymodelStance.mmcAngularLegModelRobot import mmcAngularLegModelStance

#import scipy.io

##
#   Walknet leg structure.
#   The leg structures are hierarchical and nested.
#   In the normal leg structure (in the Hector directory) the Hector
#   specific definitions, access and help functions are defined (e.g. joints, kinematics).
#   In the WLeg specific variables for Walknet are defined, e.g. AEP, PEP.
#   The same holds true for the Leg structure (in WRobot there are WLegs).
##
class WLeg (Freezable):

    ##
    #   Initialization. Setting basic constants and variables.
    def __init__(self, leg):
        self.leg = leg

#       self.aep = numpy.array([0.0,0.0,0.0], dtype=float)
#       self.pep = numpy.array([0.0,0.0,0.0], dtype=float)
        self.dep = numpy.array([0.0,0.0,WSTATIC.depz], dtype=float)

#       self._pep_shift         = numpy.array([0.0,0.0,0.0], dtype=float)
#       self._pep_shift_is_up_to_date=True
#       self._pep_shifts=dict()

#       self._aep_shift         = numpy.array([0.0,0.0,0.0], dtype=float)
#       self._aep_shift_is_up_to_date=True
#       self._aep_shifts=dict()

        self.rule2starttime     = 0
        self.rule2stoptime      = 0
        self.rule1starttimeB    = 0
        self.rule1stoptimeB     = 0
        self.stancespeedshift   = 1


        # Parameter for the swing net 3b
        # Should be moved there.
        self.swingNet_hpVelocity = WSTATIC.swingNet_hpVelocity[self.leg.name]
        self.swingNet_hpTimeConst = WSTATIC.swingNet_hpTimeConst[self.leg.name]
        self.swingNet_hpGammaVelos = WSTATIC.swingNet_hpGammaVelos[self.leg.name]
        self.swingNet_aepShiftX = WSTATIC.swingNet_aepShiftX[self.leg.name] # for forward walking
        self.swingNet_aepShiftZ = WSTATIC.swingNet_aepShiftZ[self.leg.name]

        self.swingNet_pepShiftX = WSTATIC.swingNet_pepShiftX[self.leg.name] # for backward walking
        self.swingNet_pepShiftZ = WSTATIC.swingNet_pepShiftZ[self.leg.name]

        self.lift_counter = 0


        # Collecting the new control (velocity) signals
        self.controlVelocities = numpy.zeros(3)

        self.frozen=True

#   @property
#   def pep_shifted(self):
#       tempPep = self.pep+self.pep_shift
#       if tempPep[0]>=self.aep_shifted[0]-WSTATIC.minimum_swing_distance:
#           tempPep[0]=self.aep_shifted[0]-WSTATIC.minimum_swing_distance
#       return tempPep
# 
#   @property
#   def pep_shift(self):
#       if not self._pep_shift_is_up_to_date:
#           self._pep_shift=sum(self._pep_shifts.values())
#           self._pep_shift_is_up_to_date=True
#       return self._pep_shift
# 
#   def shiftPep(self, source_name, pep_shift):
#       try:
#           temp_pep_shift=numpy.array((float(pep_shift), 0, 0))
#       except TypeError:
#           if len(pep_shift)==3:
#               temp_pep_shift=numpy.array((float(i) for i in pep_shift))
#       if temp_pep_shift.any():
#           self._pep_shifts[source_name]=temp_pep_shift
#       else:
#           try:
#               del self._pep_shifts[source_name]
#           except Exception:
#                   pass
#       self._pep_shift_is_up_to_date=False
# 
#   def resetPepShift(self):
#       self._pep_shifts=dict()
# 
# 
#   @property
#   def aep_shifted(self):
#       return self.aep+self.aep_shift
# 
#   @property
#   def aep_shift(self):
#       if not self._aep_shift_is_up_to_date:
#           self._aep_shift=sum(self._aep_shifts.values())
#           self._aep_shift_is_up_to_date=True
#       return self._aep_shift
# 
#   def shiftAep(self, source_name, aep_shift):
#       try:
#           temp_aep_shift=numpy.array((float(aep_shift), 0, 0))
#       except TypeError:
#           if len(aep_shift)==3:
#               temp_aep_shift=numpy.array((float(i) for i in aep_shift))
#       if temp_aep_shift.any():
#           self._aep_shifts[source_name]=temp_aep_shift
#       else:
#           try:
#               del self._aep_shifts[source_name]
#           except Exception:
#                   pass
#       self._aep_shift_is_up_to_date=False
# 
#   def resetAepShift(self):
#       self._aep_shifts=dict()


#=========== Walknet Methods ====================
    ##
    #   Estimate ground ground_contact:
    #   Predict current leg position (using fw kinematics) and
    #   simply decide if the leg should touch ground
    #   (very stable, but works only on flat terrain).
    def predictedGroundContact(self):
        if "front" in self.leg.name:
            if (self.leg.leg_enabled):
                if (self.leg.input_foot_position[2] < (WSTATIC.front_initial_aep[2] * WSTATIC.predicted_ground_contact_height_factor)):
                    return 1
        if "middle" in self.leg.name:
            if (self.leg.leg_enabled):
                if (self.leg.input_foot_position[2] < (WSTATIC.middle_initial_aep[2] * WSTATIC.predicted_ground_contact_height_factor)):
                    return 1
        if "hind" in self.leg.name:
            if (self.leg.leg_enabled):
                if (self.leg.input_foot_position[2] < (WSTATIC.hind_initial_aep[2] * WSTATIC.predicted_ground_contact_height_factor)):
                    return 1
        
        return 0

    ##
    #   Ground Contact using Tarsus Pressure Sensor:
    #   Each leg is equipped (right now only in simulation) with a ground ground_contact
    #   sensor which provides the ground interaction force:
    #   positive value means the pressure which is acting on the sensor,
    #   when in the air the sensor just delivers a 0.
    def tarsusPressureSensorGroundContact(self):
        if (self.leg.tarsus_pressure_sensor.highestPressure >= WSTATIC.tarsusPressureGCThreshold):
            return 1
        return 0

    ##
    #   Calculation of DEP = point between AEP and PEP used as target for swing
    #   trajectory generation (Dorsal Extreme Position).
    #   Called by explicit swing trajectory generator
    def depcalc(self, start, target):
        vec = target - start
        basepoint = start+(vec*WSTATIC.deprelation)

        # the normal vector y-direction depends on the side of the robot
        if "left" in self.leg.name:
            norm_vec = [0, 1, WSTATIC.dep_normal_vector_z_slope]
        else:
            norm_vec = [0,-1, WSTATIC.dep_normal_vector_z_slope]
        direction_from_basepoint = numpy.cross(vec, numpy.array(norm_vec))
        
        # find factor for the directionVec. to meet the required z-value
        factor = (WSTATIC.depz - basepoint[2])/direction_from_basepoint[2]
        self.dep = basepoint + factor * direction_from_basepoint
        
        if not(start[0] < self.dep[0] < target[0]):
            #print("DEP has to be corrected ", self.dep, start, target, basepoint, self.leg.name)
            self.dep[0] = basepoint[0]
        
        return self.dep

# Called by sidewalk - what is this doing?
#   def turnwalk(self,sidewayangle, total):
#       def stancecorrector(stepwidth, additionaldist):
#           newdist = stepwidth - additionaldist
#           return newdist/stepwidth
# 
#       additional = (math.tan(sidewayangle) * (self.pep[1]+self.leg.xmlAnchors[1] ))*2
#       stepwidth = self.aep[0] - self.pep[0]
#       #~ print('steps', stepwidth)
#       expsteps = total/stepwidth
#       additionalstep = additional/expsteps
#       #~ print("addi",additionalstep)
#       self._pep_shift += (additionalstep*0.5)
#       self.aep_shifted -= (additionalstep*0.5)

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
        self.leg.setAngleVelocity(self.controlVelocities)
        self.controlVelocities = numpy.zeros(3)
