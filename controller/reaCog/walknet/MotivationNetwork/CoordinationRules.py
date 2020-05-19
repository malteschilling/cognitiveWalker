import abc
from tools.FreezableF import Freezable as Freezable
from ProcessOrganisation.SimulatorModule.SimulatorTimerModule import getCurrentTime as getCurrentTime
from ... import WalknetSettings as WSTATIC
from decimal import Decimal as Decimal
import math
import numpy

##
#	CoordinationRule
#
#	Coordination rules affect the posterior extreme positions of legs. This determines
#	switching from stance to swing movement (when the leg is behind the PEP, 
#	swing will be started). 
#	A coordination rule object realizes an object that 
#	- acts on the PEP and shifts it (using the evaluateShift method)
#	- depending on certain conditions (which are described in the derived classes),
#		always connected to a sender leg
##
class CoordinationRule(metaclass=abc.ABCMeta):
    def __init__(self, sender):
        self.sender=sender
        self.time_precision=Decimal('0.0001')
        
    @abc.abstractmethod
    def evaluateShift(self, mod_weight=None, coord_weight=None):
        return

##
# 	Rule1
#
#	Is active when the sender leg is in swing
##    
class Rule1(CoordinationRule, Freezable):
    """
    
    """
    def __init__(self, sender, swing_stance_pep_shift_ratio=0, velocity_threshold=0, velocity_threshold_delay_offset=0, lower_velocity_delay_factor=0, higher_velocity_delay_factor=0, max_delay=float('Inf')):
        CoordinationRule.__init__(self, sender)
        self.swing_stance_pep_shift_ratio=swing_stance_pep_shift_ratio
        self.velocity_threshold=velocity_threshold
        self.velocity_threshold_delay_offset=velocity_threshold_delay_offset
        self.lower_velocity_delay_factor=lower_velocity_delay_factor
        self.higher_velocity_delay_factor=higher_velocity_delay_factor
        self.max_delay=Decimal(max_delay).quantize(self.time_precision)
        self.swing_stance_pep_shift_ratio=swing_stance_pep_shift_ratio
        
        self.sender_is_swinging_old=False
        self.stance_start_time=Decimal('-Infinity')
        
        self.frozen=True
        
    def evaluateShift(self, mod_weight=None, coord_weight=1):
        if self.sender.isSwinging(): # if the leg is in swing phase
            self.sender_is_swinging_old=True
            return coord_weight
        else: # if the leg is in stance phase
            if self.sender_is_swinging_old==True:
                self.stance_start_time=Decimal(getCurrentTime()).quantize(self.time_precision)
                self.sender_is_swinging_old=False
            
            # velocity=numpy.linalg.norm(self.sender.wleg.leg.computeTarsusVelocity())
            velocity=WSTATIC.default_speed
            
            if velocity<=self.velocity_threshold:
                delay= Decimal(self.velocity_threshold_delay_offset+self.lower_velocity_delay_factor*(velocity-self.velocity_threshold)).quantize(self.time_precision)
            else:
                delay= Decimal(self.velocity_threshold_delay_offset+self.higher_velocity_delay_factor*(velocity-self.velocity_threshold)).quantize(self.time_precision)
            if delay>self.max_delay:
                delay=self.max_delay
            elif delay<0:
                delay=0
            
            if Decimal(getCurrentTime()).quantize(self.time_precision)< (self.stance_start_time+delay):
                return coord_weight*self.swing_stance_pep_shift_ratio
        return 0.

##
# 	Rule2
#
#	Becomes briefly activated after the sender leg ends swing movement.
##          
class Rule2(CoordinationRule, Freezable):
    def __init__(self, sender, start_delay, duration):
        CoordinationRule.__init__(self, sender)
        self.start_delay=Decimal(start_delay).quantize(self.time_precision)
        self.duration=Decimal(duration).quantize(self.time_precision)
        
        self.sender_is_swinging_old=False
        self.stance_start_time=Decimal('-Infinity')
        
        self.frozen=True
        
    def evaluateShift(self, mod_weight=None, coord_weight=1):
        if self.sender.isSwinging():
            self.sender_is_swinging_old=True
        else:
            if self.sender_is_swinging_old==True:
                self.stance_start_time=Decimal(getCurrentTime()).quantize(self.time_precision)
                self.sender_is_swinging_old=False
                
            if (self.stance_start_time+self.start_delay ) <= Decimal(getCurrentTime()).quantize(self.time_precision) < (self.stance_start_time+self.start_delay+self.duration):
                return coord_weight
        return 0.

##
# 	Rule3
#
#	Is active when the sender leg is in a certain range near to the front of the PEP.
##  
class Rule3(CoordinationRule):
    def __init__(self, sender, active_distance):
        CoordinationRule.__init__(self, sender)
        self.active_distance=active_distance
        
    def evaluateShift(self, mod_weight=None, coord_weight=1):
        if not self.sender.isSwinging():
            lpep = self.sender.pep[0]
            laep = self.sender.aep[0]
            xpos = self.sender.wleg.leg.input_foot_position[0]
            vel = WSTATIC.default_speed
            
            position_threshold=math.fabs(lpep-laep)*self.threshold_function(vel)
            if lpep+position_threshold-self.active_distance < xpos < lpep+position_threshold:
                return coord_weight
        return 0.
        
    @abc.abstractmethod
    def threshold_function(self, velocity):
        return
    
class Rule3LinearThreshold(Rule3, Freezable):
    def __init__(self, sender, active_distance, threshold_offset, threshold_slope):
        Rule3.__init__(self, sender, active_distance)
        self.threshold_offset=threshold_offset
        self.threshold_slope=threshold_slope
        
        self.frozen=True
                
    def threshold_function(self, velocity):
        return self.threshold_offset+self.threshold_slope*velocity

class Rule3Ipsilateral(Rule3, Freezable):
    def __init__(self, sender, active_distance, threshold_offset, threshold_slope, threshold_2_offset, threshold_2_slope):
        Rule3.__init__(self, sender, active_distance)
        self.threshold_offset=threshold_offset
        self.threshold_slope=threshold_slope
        self.threshold_2_offset=threshold_2_offset
        self.threshold_2_slope=threshold_2_slope
        
        self.frozen=True
        
    def threshold_function(self, velocity):
        if (velocity > 0.01):
            threshold = self.threshold_offset+self.threshold_slope*velocity
        else:
            threshold = self.threshold_2_offset+self.threshold_2_slope*velocity
        return threshold

    def printCoordRule3(self):
        lpep = self.sender.pep[0]
        laep = self.sender.aep[0]
        xpos = self.sender.wleg.leg.input_foot_position[0]
        vel = WSTATIC.default_speed
        print("COORD RULE 3 ", self.sender.pep[0], self.sender.aep[0], self.sender.wleg.leg.input_foot_position[0])
        print(math.fabs(lpep-laep)*self.threshold_function(WSTATIC.default_speed))
        print(self.evaluateShift())

class Rule3SigmoidThreshold(Rule3, Freezable):
    def __init__(self, sender, active_distance, threshold_turning_point, threshold_slope):
        Rule3.__init__(self, sender, active_distance)
        self.threshold_turning_point=threshold_turning_point
        self.threshold_slope=threshold_slope
        
        self.frozen=True
        
    def threshold_function(self, velocity):
        # Velocity changed as has been newly scaled in new body model
        return 1/(1+numpy.exp(-4*self.threshold_slope*(5*velocity-self.threshold_turning_point)))
        
class Rule4(CoordinationRule):
    def __init__(self, sender):
        CoordinationRule.__init__(self, sender)     
        self.frozen=True
    
    def evaluateShift(self, mod_weight=None, coord_weight=None):
        return coord_weight*(self.sender.wleg.leg.input_foot_position[0]-self.sender.pep[0])
    
    