#Static values for Movement class

import numpy
import Hector.RobotSettings as RSTATIC

#===== timing ====
stepwidth = 1/RSTATIC.controller_frequency
standardDuration4oneMove = 3

#===== controller ====
kp = 1 #0.95
ki = 0 #0.005

#===== standing robot ====
standardFootDistanceY = -0.3
standardRobotHeightStanding = 0.2
standardRobotHeightLaying = 0.09

#===== moving robot ====
standardSwingHeightRel = 0.15 # relative to ground
maxSwingHeightAbs   = 0.1      # absolute height in rotated CS


#===== input angle tolerance ====
maxAngularDeviation = [numpy.pi/180,numpy.pi/180,numpy.pi/180]
# = 1° tolerance for final joint angles (input position of joint)


#===== performance ====
debugPerformance = False
preOrderLegAngleData = True 


#===== other =======
defaultTalkValue = 1
